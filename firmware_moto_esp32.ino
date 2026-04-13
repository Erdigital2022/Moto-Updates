/*
 * MotoControl Pro — ESP32
 * Versão: 3.2 — NimBLE v2.x + WiFi + MQTT TLS
 *
 * Compatível com NimBLE-Arduino by h2zero versão 2.x
 *
 * MUDANÇAS DA API v2.x vs v1.x:
 *   - Callbacks usam parâmetros diferentes (sem ponteiro de servidor)
 *   - setScanResponse() → removido (não necessário)
 *   - NimBLECharacteristicCallbacks recebe ConnInfo no onWrite
 */

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HardwareSerial.h>

// =============================================================================
// CONFIGURAÇÕES
// =============================================================================

#define PIN_RELAY_IGNITION  12
#define PIN_RELAY_STARTER   13
#define PIN_RELAY_KILL      14
#define PIN_SENSE_NEUTRAL   27
#define PIN_SENSE_BATTERY   34
#define RX2_PIN             16
#define TX2_PIN             17

#define BLE_SERVICE_UUID     "12345678-1234-1234-1234-123456789abc"
#define BLE_CHAR_CMD_UUID    "12345678-1234-1234-1234-123456789abd"
#define BLE_CHAR_STATUS_UUID "12345678-1234-1234-1234-123456789abe"

const char* MQTT_BROKER  = "207e698bbf4b4a0bb7ba88c530587131.s1.eu.hivemq.cloud";
const int   MQTT_PORT    = 8883;
const char* MQTT_USER    = "MotoControlESP32";
const char* MQTT_PASS    = "@MotoControlESP32";
const char* TOPIC_CMD    = "moto/cmd";
const char* TOPIC_STATUS = "moto/status";
const char* TOPIC_GPS    = "moto/gps";
const char* TOPIC_BAT    = "moto/battery";

const char* AP_SSID    = "MotoControl";
const char* AP_PASS    = "12345678";
const int   AP_TIMEOUT = 300000;

const unsigned long GPS_INTERVAL = 15000;
const unsigned long BAT_INTERVAL = 30000;
const unsigned long BLE_INTERVAL = 5000;

// =============================================================================
// VARIÁVEIS GLOBAIS
// =============================================================================

NimBLEServer*          bleServer     = nullptr;
NimBLECharacteristic*  bleCharCmd    = nullptr;
NimBLECharacteristic*  bleCharStatus = nullptr;
bool                   bleConnected  = false;

WiFiClientSecure espClient;
PubSubClient     mqtt(espClient);

WebServer   apServer(80);
Preferences prefs;
String      storedSSID = "";
String      storedPass = "";
volatile int  testStatus  = 0;
unsigned long testStart   = 0;
unsigned long portalStart = 0;

bool  is_neutral      = true;
float battery_voltage = 0.0;

unsigned long lastGpsPoll = 0;
unsigned long lastBatSend = 0;
unsigned long lastBleSend = 0;

const int MPU_ADDR = 0x68;
int16_t   AcX, AcY, AcZ;
float     base_tilt_x = 0, base_tilt_y = 0;
bool      alarm_armed = false;

HardwareSerial simSerial(2);

// Forward declarations
void remoteStart();
void remoteStop();
void armAlarm();
void disarmAlarm();
void bleNotify(const String& msg);
void initMPU();

// =============================================================================
// BLE — API NimBLE v2.x
// =============================================================================

// Na v2.x os callbacks de servidor não recebem o ponteiro do servidor
class BLEConnCallback : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    bleConnected = true;
    Serial.printf("[BLE] Conectado. Heap: %d\n", ESP.getFreeHeap());
    bleNotify("STATUS:CONNECTED");
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    bleConnected = false;
    Serial.println("[BLE] Desconectado. Reiniciando advertising...");
    NimBLEDevice::startAdvertising();
  }
};

// Na v2.x onWrite recebe também NimBLEConnInfo
class BLECmdCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    String cmd = c->getValue().c_str();
    cmd.trim();
    Serial.print("[BLE] Cmd: "); Serial.println(cmd);

    if      (cmd == "START")  remoteStart();
    else if (cmd == "STOP")   remoteStop();
    else if (cmd == "ARM")    armAlarm();
    else if (cmd == "DISARM") disarmAlarm();
    else if (cmd == "STATUS") {
      String s = "BAT="    + String(battery_voltage, 2);
      s += ";ALARM="  + String(alarm_armed ? "ON" : "OFF");
      s += ";HEAP="   + String(ESP.getFreeHeap());
      bleNotify(s);
    }
    else bleNotify("ERROR:UNKNOWN_CMD");
  }
};

void bleNotify(const String& msg) {
  if (!bleConnected || !bleCharStatus) return;
  bleCharStatus->setValue(msg.c_str());
  bleCharStatus->notify();
}

void initBLE() {
  NimBLEDevice::init("MotoControl Pro");
  NimBLEDevice::setPower(3); // potência +3dBm

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BLEConnCallback());

  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  bleCharCmd = svc->createCharacteristic(
    BLE_CHAR_CMD_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  bleCharCmd->setCallbacks(new BLECmdCallback());

  bleCharStatus = svc->createCharacteristic(
    BLE_CHAR_STATUS_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  bleCharStatus->setValue("IDLE");

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();

  Serial.printf("[BLE] NimBLE v2 pronto. Heap: %d bytes\n", ESP.getFreeHeap());
}

// =============================================================================
// PORTAL CATIVO
// =============================================================================

const char PORTAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="pt-BR"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MotoControl - WiFi</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:system-ui,sans-serif}
body{background:#0b0f1a;color:#e2e8f0;display:flex;align-items:center;justify-content:center;min-height:100vh;padding:20px}
.card{background:#111827;border-radius:14px;padding:28px;width:100%;max-width:380px;border:1px solid #1e2d45}
h1{text-align:center;color:#3b82f6;margin-bottom:4px;font-size:1.3em}
p{text-align:center;color:#64748b;font-size:.85em;margin-bottom:22px}
label{display:block;margin-bottom:4px;color:#64748b;font-size:.82em;text-transform:uppercase}
input{width:100%;padding:10px 12px;margin-bottom:14px;border:1px solid #1e2d45;border-radius:8px;background:#161d2e;color:#e2e8f0;font-size:.95em}
input:focus{outline:none;border-color:#3b82f6}
button{width:100%;padding:12px;border:none;border-radius:8px;background:#185fa5;color:#fff;font-size:1em;cursor:pointer;font-weight:600}
button:disabled{opacity:.4;cursor:wait}
.msg{text-align:center;margin-top:14px;font-size:.88em;padding:12px;border-radius:8px;min-height:18px}
.ok{background:#0f6e56;color:#9fe1cb}.err{background:#a32d2d;color:#f7c1c1}.info{background:#0c447c;color:#b5d4f4}
</style></head><body>
<div class="card">
  <h1>MotoControl Pro</h1>
  <p>Configure a rede WiFi do dispositivo</p>
  <form id="f">
    <label>Rede WiFi</label>
    <input type="text" name="ssid" placeholder="Nome da rede" required autocomplete="off">
    <label>Senha</label>
    <input type="password" name="pass" placeholder="Senha da rede" required>
    <button type="submit" id="btn">SALVAR E CONECTAR</button>
  </form>
  <div class="msg" id="msg"></div>
</div>
<script>
document.getElementById('f').addEventListener('submit',function(e){
  e.preventDefault();
  var btn=document.getElementById('btn'),msg=document.getElementById('msg');
  btn.disabled=true;msg.className='msg info';msg.textContent='Testando conexao...';
  fetch('/test',{method:'POST',body:new FormData(this)}).then(function(){
    var t=setInterval(function(){
      fetch('/status').then(function(r){return r.text();}).then(function(s){
        if(s==='CONNECTED'){clearInterval(t);msg.className='msg ok';msg.textContent='Conectado! Salvando...';fetch('/save',{method:'POST'});}
        else if(s==='FAILED'){clearInterval(t);msg.className='msg err';msg.textContent='Falha. Verifique a senha.';btn.disabled=false;}
      }).catch(function(){});
    },1500);
  }).catch(function(){msg.className='msg err';msg.textContent='Erro.';btn.disabled=false;});
});
</script></body></html>
)rawliteral";

void handleTest() {
  String ssid = apServer.arg("ssid");
  String pass = apServer.arg("pass");
  if (ssid.isEmpty() || pass.isEmpty()) { apServer.send(400,"text/plain","MISSING"); return; }
  storedPass = pass; storedSSID = ssid;
  testStatus = 1; testStart = millis();
  WiFi.disconnect(false,false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  apServer.send(202,"text/plain","OK");
}

void handlePortalStatus() {
  if (testStatus == 1) {
    if (WiFi.status() == WL_CONNECTED) testStatus = 2;
    else if (millis() > testStart + 18000) { testStatus = 3; WiFi.mode(WIFI_AP); }
  }
  if      (testStatus == 2) apServer.send(200,"text/plain","CONNECTED");
  else if (testStatus == 3) apServer.send(200,"text/plain","FAILED");
  else if (testStatus == 0) apServer.send(200,"text/plain","IDLE");
  else                      apServer.send(200,"text/plain","TESTING");
}

void handleSave() {
  prefs.begin("mcontrol",false);
  prefs.putString("ssid", storedSSID);
  prefs.putString("pass", storedPass);
  prefs.end();
  apServer.send(200,"text/plain","OK");
  delay(200);
  ESP.restart();
}

void startPortal() {
  Serial.println("[PORTAL] Iniciando...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  apServer.on("/",       [](){ apServer.send(200,"text/html",PORTAL_HTML); });
  apServer.on("/test",   handleTest);
  apServer.on("/status", handlePortalStatus);
  apServer.on("/save",   handleSave);
  apServer.begin();
  portalStart = millis();
  Serial.printf("[PORTAL] AP: %s | IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  while (WiFi.status() != WL_CONNECTED) {
    apServer.handleClient();
    if (millis() - portalStart > AP_TIMEOUT) ESP.restart();
    delay(10);
  }
  apServer.stop();
}

void connectWiFi() {
  Serial.printf("[WiFi] Conectando a '%s'...\n", storedSSID.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(storedSSID.c_str(), storedPass.c_str());
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] IP: %s | Heap: %d\n",
      WiFi.localIP().toString().c_str(), ESP.getFreeHeap());
  } else {
    Serial.println("\n[WiFi] Falha. Limpando credenciais.");
    prefs.begin("mcontrol",false); prefs.clear(); prefs.end();
  }
}

// =============================================================================
// MQTT
// =============================================================================

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  Serial.print("[MQTT] Cmd: "); Serial.println(msg);
  if      (msg == "START")  remoteStart();
  else if (msg == "STOP")   remoteStop();
  else if (msg == "ARM")    armAlarm();
  else if (msg == "DISARM") disarmAlarm();
}

void mqttReconnect() {
  static int tentativas = 0;
  tentativas++;
  Serial.printf("[MQTT] Tentativa %d | Heap: %d\n", tentativas, ESP.getFreeHeap());
  String clientId = "MotoESP32_";
  clientId += String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    tentativas = 0;
    mqtt.subscribe(TOPIC_CMD);
    mqtt.publish(TOPIC_STATUS, "ONLINE");
    Serial.printf("[MQTT] Conectado! Heap: %d bytes\n", ESP.getFreeHeap());
  } else {
    Serial.printf("[MQTT] Falha rc=%d\n", mqtt.state());
    delay(min(5000 + tentativas * 2000, 30000));
  }
}

// =============================================================================
// CONTROLE DA MOTO
// =============================================================================

void remoteStart() {
  Serial.println("[CMD] Partida remota");
  digitalWrite(PIN_RELAY_IGNITION, HIGH); delay(1500);
  digitalWrite(PIN_RELAY_STARTER,  HIGH); delay(2000);
  digitalWrite(PIN_RELAY_STARTER,  LOW);
  if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "SUCCESS:ENGINE_STARTED");
  bleNotify("SUCCESS:ENGINE_STARTED");
}

void remoteStop() {
  Serial.println("[CMD] Desligando");
  digitalWrite(PIN_RELAY_IGNITION, LOW);
  digitalWrite(PIN_RELAY_STARTER,  LOW);
  if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "SUCCESS:ENGINE_STOPPED");
  bleNotify("SUCCESS:ENGINE_STOPPED");
}

void armAlarm() {
  alarm_armed = true; initMPU();
  if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "ALARM:ARMED");
  bleNotify("ALARM:ARMED");
  Serial.println("[CMD] Alarme armado");
}

void disarmAlarm() {
  alarm_armed = false;
  digitalWrite(PIN_RELAY_KILL, LOW);
  if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "ALARM:DISARMED");
  bleNotify("ALARM:DISARMED");
  Serial.println("[CMD] Alarme desarmado");
}

// =============================================================================
// GPS
// =============================================================================

void initGPS() {
  simSerial.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  simSerial.println("AT+CGPS=1");
  delay(500);
  Serial.println("[GPS] Iniciado.");
}

void updateGPS() {
  simSerial.println("AT+CGPSINFO");
  delay(200);
  String data = "";
  while (simSerial.available()) data += (char)simSerial.read();
  if (data.indexOf(",") != -1 && mqtt.connected())
    mqtt.publish(TOPIC_GPS, data.c_str());
}

// =============================================================================
// MPU6050
// =============================================================================

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);
  delay(50);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
  base_tilt_x = AcX; base_tilt_y = AcY;
  Serial.println("[MPU] Calibrado.");
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  AcX = Wire.read()<<8|Wire.read();
  AcY = Wire.read()<<8|Wire.read();
  AcZ = Wire.read()<<8|Wire.read();
}

void checkAlarm() {
  if (!alarm_armed) return;
  readMPU();
  if (abs(AcX - base_tilt_x) > 4000 || abs(AcY - base_tilt_y) > 4000) {
    Serial.println("[ALARM] Movimento detectado!");
    if (mqtt.connected()) mqtt.publish(TOPIC_STATUS, "ALARM:MOTION");
    bleNotify("ALARM:MOTION_DETECTED");
    digitalWrite(PIN_RELAY_KILL, HIGH); delay(500);
    digitalWrite(PIN_RELAY_KILL, LOW);
  }
}

void readBattery() {
  battery_voltage = (analogRead(PIN_SENSE_BATTERY) / 4095.0) * 3.3 * 5.0;
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== MotoControl Pro v3.2 ===\n");
  Serial.printf("[BOOT] Heap inicial: %d bytes\n", ESP.getFreeHeap());

  pinMode(PIN_RELAY_IGNITION, OUTPUT);
  pinMode(PIN_RELAY_STARTER,  OUTPUT);
  pinMode(PIN_RELAY_KILL,     OUTPUT);
  pinMode(PIN_SENSE_NEUTRAL,  INPUT_PULLUP);
  digitalWrite(PIN_RELAY_IGNITION, LOW);
  digitalWrite(PIN_RELAY_STARTER,  LOW);
  digitalWrite(PIN_RELAY_KILL,     LOW);

  Wire.begin();

  // NimBLE primeiro — nunca chamar deinit()
  initBLE();

  // WiFi
  prefs.begin("mcontrol", true);
  storedSSID = prefs.getString("ssid", "");
  storedPass = prefs.getString("pass", "");
  prefs.end();

  if (storedSSID.length() > 0) connectWiFi();
  if (WiFi.status() != WL_CONNECTED) startPortal();

  // MQTT TLS
  espClient.setInsecure();
  espClient.setTimeout(15);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);

  initGPS();
  initMPU();

  Serial.printf("[BOOT] Sistema pronto. Heap: %d bytes\n", ESP.getFreeHeap());
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
  }

  checkAlarm();
  readBattery();

  if (now - lastGpsPoll > GPS_INTERVAL) { updateGPS(); lastGpsPoll = now; }

  if (now - lastBatSend > BAT_INTERVAL && mqtt.connected()) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", battery_voltage);
    mqtt.publish(TOPIC_BAT, buf);
    lastBatSend = now;
  }

  if (bleConnected && now - lastBleSend > BLE_INTERVAL) {
    String s = "BAT=" + String(battery_voltage,2);
    s += ";ALARM=" + String(alarm_armed ? "ON":"OFF");
    s += ";HEAP=" + String(ESP.getFreeHeap());
    bleNotify(s);
    lastBleSend = now;
  }

  delay(10);
}
