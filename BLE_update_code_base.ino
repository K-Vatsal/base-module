/*********************  Includes  *********************/
#include <LoRa_E220.h>
#include <WiFi.h>
#include <Hash.h>
#include <sha256.h>
#include <Update.h>
#include <String.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <NimBLEDevice.h>
#include <FastLED.h>
#include <cert.h>   // keep certs & your globals in cert.h

/*********************  Safe DEBUG macros  *********************/
#ifndef DEBUG_PRINT
#define DEBUG_PRINT(x)       // no-op
#define DEBUG_PRINTLN(x)     // no-op
#endif
#define SHA256_HASH_SIZE 32

/*********************  Forward Declarations  *********************/
// Visual
void motor_led(int led, bool state);
void tank1(int level);
void tank2(int level);
void test_led();
void updateTankLEDs(const String& sensor, int distance, int range);

// BLE
void blue_set();

// LoRa
void Lora_receive();                         // parse JSON, update range/distance, run primaries, LEDs & publish %
void Lora_send(byte addh, byte addl, byte chan, String datas);

// OTA
void firmwareUpdate();

// IO
void readInputs();
void handleInputChanges();
void onWrite(bool status, int pin);
void storeRelayStates();
int  Measure();

// AWS/WiFi/MQTT
void connectAWS();
void reconnect();
void wifiSetup();
void messageHandler(char* topic, byte* payload, unsigned int length);

// Publish helpers
void Publish_Statusresponse(String sen);
void Publish_Slaveresponse(String sen, String id, int rangeVal);
void Publish_Health();
void Publish_Alive();
void Publish_Update(bool state, int number);
void Publish_Updates(int percent, String number);
void Publish_OTA_Valid(String transaction, String secret, String versions, String reply);
void Publish_OTA_Valids(String thingId, String OTA, String versions, String reply);

// Utils
void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
bool isVersionNewer(String current, String ota);
void reseting();
void reseting_slave(String id, String sensor);

/********** Debug-only helpers (prototypes) **********/
void debug_print_arr_entry(int idx, const char* where);
void debug_dump_arr_table(const char* where);
void debug_print_primary1(const char* tag);
void debug_print_primary2(const char* tag);
void debug_print_parsed_action(const char* raw, const String& devCode, int sNo, bool on, bool ok);

/*********************  Drivers / Clients  *********************/
LoRa_E220 e220ttl(16, 17, &Serial2, M0, M1, AUX, UART_BPS_RATE_9600);
CRGB leds[NUM_LEDS];

/*** Preferences: separate handles ***/
Preferences prefDatas; // "datas" -> ssid, password, flags, mode, etc.
Preferences prefSlots; // "slots" -> slave table (arr[20])

WiFiClientSecure net;
PubSubClient client(net);

/*********************  App flags  *********************/
bool initial = 0;
bool states  = 0;
int  modes   = 0;
bool flag    = 0;
byte fan     = 0;
int  net_aws = 0;
int  net_w   = 0;
int  net_b   = 0;
String valor = "";
int  aleatorio;
String received = "";
String alea = "3";
unsigned long Login_Time = 0;
String macs = "";
int input_counter = 0;
unsigned long input_time = 0;
unsigned long reconnect_time = 0;

/********** NEW: Action gating flags **********/
bool Trigger_flag = false;   // raise when <= MIN% (trigger)
bool Stop_flag    = false;   // raise when >= MAX% (stop)

/*********************  BLE  *********************/
#define SERVICE_UUID        "7fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "7eb5483e-36e1-4688-b7f5-ea07361b26a8"
NimBLEServer*         pServer          = nullptr;
NimBLECharacteristic* pCharacteristic  = nullptr;
bool deviceConnected = false;
static volatile bool notifEnabled = false;

/*********************  Data Structures (only these 3)  *********************/
struct Primary1 {
  String slave;
  String parameter;
  int data1 = 0;   // max (%) for TM, or value for switch
  int data2 = 0;   // min (%) for TM, or value for switch
  String A1;
  String A2;
  String A3;
};

struct Primary2 {
  String slave;
  String parameter;
  int data1 = 0;
  int data2 = 0;
  String A1;
  String A2;
  String A3;
};

// ------- Action parsing type (used for A1/A2/A3) -------
struct ParsedAction {
  String devCode;      // "MC" / "TM" / "BM" / ...
  int    switchNo = 1;
  bool   on = false;
  bool   valid = false;
};

struct salve {
  String  device_name;   // e.g., "TM1","MC1"
  String  device_id;     // unique id from slave
  int     mode;          // 1=LoRa, 2=BLE, 3=ESP-NOW
  uint8_t channel;       // LoRa channel
  uint16_t address;      // (ADDH<<8 | ADDL)
  uint8_t addr_high;     // ADDH
  uint8_t addr_low;      // ADDL
  int     range;         // total distance of the tank (cm)
  int     distance;      // present value of the tank (cm)
  int     Ph;
  int     TDS;
  int     turbulity;
  bool    sw_no[8];
};

/*********************  Globals using those structs  *********************/
salve    arr[20];
Primary1 gPrimary1;  // instance of Primary1
Primary2 gPrimary2;  // instance of Primary2

/*********************  Name helpers  *********************/
static inline int tank_index_from_name(const String& name) {
  if (name == "TM1") return 0;
  if (name == "TM2") return 1;
  if (name == "TM3") return 2;
  if (name == "TM4") return 3;
  return -1;
}
static inline bool parse_sensor_code(const String& code, String& prefix, int& numZeroBased) {
  if (code.length() < 3) return false;
  prefix = code.substring(0, 2);
  int n = code.substring(2).toInt();  // 1..4
  if (n <= 0) return false;
  numZeroBased = n - 1;
  return true;
}
static inline int index_from_code(const String& code) {
  String p; int k;
  if (!parse_sensor_code(code, p, k) || k < 0 || k > 3) return -1;
  if (p == "TM") return 0  + k;
  if (p == "MC") return 4  + k;
  if (p == "VC") return 8  + k;
  if (p == "WC") return 12 + k;
  if (p == "BM") return 16 + k;   // allow BM mapping
  return -1;
}

/*********************  Slave persistence (NVS)  *********************/
static const char* NVS_NS = "slots";

static inline String make_slave_key(int idx) { return "SLAVE_" + String(idx); }

static inline String serialize_slave_entry(const salve& s) {
  DynamicJsonDocument d(512);
  d["name"]      = s.device_name;
  d["id"]        = s.device_id;
  d["mode"]      = s.mode;
  d["ch"]        = s.channel;
  d["ah"]        = s.addr_high;
  d["al"]        = s.addr_low;
  d["addr"]      = s.address;
  d["range"]     = s.range;
  d["distance"]  = s.distance;
  d["Ph"]        = s.Ph;
  d["TDS"]       = s.TDS;
  d["turbulity"] = s.turbulity;
  String out; serializeJson(d, out);
  return out;
}
bool deserialize_slave_entry(const String& in, salve& s) {
  if (in.length() == 0) return false;
  DynamicJsonDocument d(768);
  if (deserializeJson(d, in)) return false;
  s.device_name = d["name"]      | "";
  s.device_id   = d["id"]        | "";
  s.mode        = d["mode"]      | 0;
  s.channel     = d["ch"]        | 0;
  s.addr_high   = d["ah"]        | 0;
  s.addr_low    = d["al"]        | 0;
  s.address     = d["addr"]      | 0;
  s.range       = d["range"]     | 0;
  s.distance    = d["distance"]  | 0;
  s.Ph          = d["Ph"]        | 0;
  s.TDS         = d["TDS"]       | 0;
  s.turbulity   = d["turbulity"] | 0;
  for (int b=0;b<8;b++) s.sw_no[b] = false;
  return true;
}

/*** IMPORTANT: use prefSlots for slave-table I/O ***/
void save_slave_entry_nvs(int idx) {
  if (idx < 0 || idx >= 20) return;
  prefSlots.begin(NVS_NS, false);
  prefSlots.putString(make_slave_key(idx).c_str(), serialize_slave_entry(arr[idx]));
  prefSlots.end();
  Serial.printf("[NVS] Saved arr[%d] -> name=%s id=%s mode=%d ch=%u ah=%u al=%u addr=0x%04X dist=%d range=%d\n",
                idx,
                arr[idx].device_name.c_str(), arr[idx].device_id.c_str(),
                arr[idx].mode, arr[idx].channel,
                arr[idx].addr_high, arr[idx].addr_low, arr[idx].address,
                arr[idx].distance, arr[idx].range);
}
bool load_slave_entry_nvs(int idx) {
  if (idx < 0 || idx >= 20) return false;
  prefSlots.begin(NVS_NS, true);
  String blob = prefSlots.getString(make_slave_key(idx).c_str(), "");
  prefSlots.end();
  if (blob.isEmpty()) return false;
  salve tmp;
  if (!deserialize_slave_entry(blob, tmp)) return false;
  arr[idx] = tmp;
  Serial.printf("[NVS] Loaded arr[%d] <- name=%s id=%s mode=%d ch=%u ah=%u al=%u addr=0x%04X dist=%d range=%d\n",
                idx,
                arr[idx].device_name.c_str(), arr[idx].device_id.c_str(),
                arr[idx].mode, arr[idx].channel,
                arr[idx].addr_high, arr[idx].addr_low, arr[idx].address,
                arr[idx].distance, arr[idx].range);
  return true;
}
void save_slave_table_nvs() {
  prefSlots.begin(NVS_NS, false);
  for (int i = 0; i < 20; i++) {
    prefSlots.putString(make_slave_key(i).c_str(), serialize_slave_entry(arr[i]));
  }
  prefSlots.end();
  Serial.println("[NVS] Full arr[] table saved.");
}
void init_slave_table() {
  for (int i = 0; i < 20; i++) {
    arr[i].device_name = "";
    arr[i].device_id   = "";
    arr[i].mode        = 0;
    arr[i].channel     = 0;
    arr[i].addr_high   = 0;
    arr[i].addr_low    = 0;
    arr[i].address     = 0;
    arr[i].range       = 0;
    arr[i].distance    = 0;
    arr[i].Ph          = 0;
    arr[i].TDS         = 0;
    arr[i].turbulity   = 0;
    for (int b=0;b<8;b++) arr[i].sw_no[b] = false;
  }
  Serial.println("[ARR] Table initialized to defaults.");
}

/*********************  Slave convenience  *********************/
void set_slave(
  int idx, const String& name, const String& id,
  int mode, uint8_t ch, uint8_t ah, uint8_t al,
  int ran = 0, int dis = 0, int PH = 0, int tds = 0, int tur = 0, bool sw0 = false
) {
  if (idx < 0 || idx >= 20) return;
  arr[idx].device_name = name;
  arr[idx].device_id   = id;
  arr[idx].mode        = mode;
  arr[idx].channel     = ch;
  arr[idx].addr_high   = ah;
  arr[idx].addr_low    = al;
  arr[idx].address     = (uint16_t(ah) << 8) | al;
  if (ran > 0) arr[idx].range = ran;     // only update when provided
  arr[idx].distance    = dis;
  arr[idx].Ph          = PH;
  arr[idx].TDS         = tds;
  arr[idx].turbulity   = tur;
  arr[idx].sw_no[0]    = sw0;
  Serial.printf("[ARR] set_slave idx=%d name=%s id=%s mode=%d ch=%u ah=%u al=%u addr=0x%04X range=%d dist=%d PH=%d TDS=%d Turb=%d sw0=%d\n",
                idx, name.c_str(), id.c_str(), mode, ch, ah, al, arr[idx].address,
                arr[idx].range, arr[idx].distance, PH, tds, tur, sw0);
  save_slave_entry_nvs(idx);
}

/*********************  Measure (ultrasonic)  *********************/
int Measure() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  int distance = duration * 0.034 / 2;
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  return distance;
}

/*********************  BLE (fixed NimBLE setValue overloads) *********************/
// Helper: set characteristic value from Arduino String via bytes overload
static inline void setValueText(NimBLECharacteristic* c, const String& s) {
  c->setValue(reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
}

// Build {"deviceid":"...","thingId":"..."} using your globals
static String buildInfoJson() {
  StaticJsonDocument<128> doc;
  doc["deviceid"] = DeviceId;
  doc["thingId"]  = ThingId;
  String out; serializeJson(doc, out);
  return out;
}

class MyServerCallbacks: public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* p) override {
    deviceConnected = true;
    Serial.println("[BLE] Central connected");
  }
  void onDisconnect(NimBLEServer* p) override {
    deviceConnected = false;
    notifEnabled = false;
    Serial.println("[BLE] Central disconnected");
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted");
  }
};

class MyCallbacks: public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *c) override {
    const String js = buildInfoJson();
    setValueText(c, js);
    Serial.printf("[BLE] onRead -> %s\n", js.c_str());
  }

  void onWrite(NimBLECharacteristic *c) override {
    std::string value = c->getValue();
    valor = String(value.c_str(), value.length());
    if (valor.length() > 0) {
      Serial.println("*********");
      Serial.print("valor = "); Serial.println(valor);
    }
    received = valor;
    Serial.print("received = "); Serial.println(received);

    setValueText(c, alea); // echo back
  }

  void onSubscribe(NimBLECharacteristic* c, ble_gap_conn_desc* desc, uint16_t subValue) override {
    notifEnabled = (subValue & 0x0001);
    Serial.printf("[BLE] Notifications %s by %s\n",
                  notifEnabled ? "ENABLED" : "DISABLED",
                  NimBLEAddress(desc->peer_ota_addr).toString().c_str());

    if (notifEnabled) {
      const String js = buildInfoJson();
      setValueText(c, js);
      c->notify();
      Serial.printf("[BLE] Initial Notify -> %s\n", js.c_str());
    }
  }
};

void blue_set() {
  NimBLEDevice::init(BLE_name.c_str());
  NimBLEDevice::setMTU(200); // phone will negotiate final MTU

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  pCharacteristic->setCallbacks(new MyCallbacks());

  const String initJson = buildInfoJson();
  setValueText(pCharacteristic, initJson);
  Serial.printf("[BLE] Initial JSON: %s\n", initJson.c_str());

  pService->start();
  delay(100);

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  NimBLEDevice::startAdvertising();
  Serial.printf("[BLE] Advertising as \"%s\"\n", BLE_name.c_str());
}

/*********************  Visual helpers  *********************/
void motor_led(int led, bool state) {
  if (led < 0 || led >= NUM_LEDS) return;
  leds[led] = (state == HIGH) ? CRGB::Red : CRGB::Green;
  FastLED.show();
  delay(50);
}
void tank1(int level) {
  for (int i = 7; i >= 7 - level && i >= 3; i--) leds[i] = CRGB::Green;
  for (int i = 7 - level; i >= 3; i--)          leds[i] = CRGB::Black;
  FastLED.show();
  delay(50);
}
void tank2(int level) {
  for (int i = 8; i <= level + 8 && i <= 12; i++) leds[i] = CRGB::Green;
  for (int i = level + 8; i <= 12; i++)           leds[i] = CRGB::Black;
  FastLED.show();
  delay(50);
}
void test_led() {
  for (int i = 0; i < min(NUM_LEDS, 14); i++) { leds[i] = CRGB::Green; FastLED.show(); delay(80); }
  delay(300);
  for (int i = 0; i < min(NUM_LEDS, 14); i++) { leds[i] = CRGB::Black; FastLED.show(); delay(80); }
}
void updateTankLEDs(const String& sensor, int distance, int range) {
  if (range <= 0) return;
  int percent_full = (int)((long)(range - distance) * 100L / range);
  percent_full = constrain(percent_full, 0, 100);
  int level = map(percent_full, 0, 100, 1, 5);   // 1..5 bars
  if (sensor == "TM1")      tank1(level);
  else if (sensor == "TM2") tank2(level);
}

/*********************  OTA  *********************/
static void LOGI(const char* tag, const char* fmt, ...) {
  va_list args; va_start(args, fmt);
  Serial.printf("[%8lu ms][%s][I] ", millis(), tag);
  Serial.vprintf(fmt, args); Serial.println();
  va_end(args);
}
static void LOGW(const char* tag, const char* fmt, ...) {
  va_list args; va_start(args, fmt);
  Serial.printf("[%8lu ms][%s][W] ", millis(), tag);
  Serial.vprintf(fmt, args); Serial.println();
  va_end(args);
}
static void LOGE(const char* tag, const char* fmt, ...) {
  va_list args; va_start(args, fmt);
  Serial.printf("[%8lu ms][%s][E] ", millis(), tag);
  Serial.vprintf(fmt, args); Serial.println();
  va_end(args);
}
// Robust OTA downloader: long timeouts + resume via HTTP Range.
// Drop-in replacement for firmwareUpdate()

// Safer OTA: heap buffer (not stack), smaller chunk, same resume logic.

#include "esp_ota_ops.h"

// Drop-in replacement for firmwareUpdate()
// - HEAD to get total size -> Update.begin(total)
// - Resume with Range
// - Heap buffer (1 KB)
// - end(true) to avoid false "Aborted" on size accounting

void firmwareUpdate() {
  const char* TAG = "OTA";
  const int   MAX_RETRIES         = 3;
  const size_t CHUNK              = 1024;      // small heap buffer
  const uint32_t HTTP_TIMEOUT_MS  = 120000;    // 120s HTTP read timeout
  const uint32_t STREAM_IDLE_MS   = 15000;     // 15s stall => resume

  uint8_t* buf = (uint8_t*)ps_malloc(CHUNK);
  if (!buf) buf = (uint8_t*)malloc(CHUNK);
  if (!buf) { Serial.println("[OTA] FATAL: no RAM"); return; }

  WiFi.setSleep(false); // avoid dozing during OTA

  size_t totalExpected = 0;
  size_t offset        = 0;
  bool   begun         = false;

  LOGI(TAG, "Starting OTA: url=%s", OTA_host.c_str());

  WiFiClientSecure tls;
  if (rootCACertificate.length()) tls.setCACert(rootCACertificate.c_str());
  else                            tls.setInsecure();
  tls.setTimeout(15000);

  // 0) HEAD to learn Content-Length (best) and ensure partition big enough
  {
    HTTPClient meta;
    if (meta.begin(tls, OTA_host)) {
      int hc = meta.sendRequest("HEAD");
      if (hc > 0) {
        totalExpected = meta.header("Content-Length").toInt();
        LOGI(TAG, "HEAD total size: %u", (unsigned)totalExpected);
        // Optional MD5 (do NOT use ETag for multi-part)
        String md5 = meta.header("x-MD5");
        if (md5.isEmpty()) md5 = meta.header("x-amz-meta-md5");
        if (md5.length() == 32) {
          if (!Update.setMD5(md5.c_str())) LOGW(TAG, "setMD5 failed; continuing");
          else LOGI(TAG, "Using MD5: %s", md5.c_str());
        }
      }
      meta.end();
    }
    // Check partition size if we know total
#ifdef ESP_ARDUINO_VERSION
    if (totalExpected) {
      const esp_partition_t* p = esp_ota_get_next_update_partition(NULL);
      if (!p) { LOGE(TAG, "No OTA partition"); free(buf); return; }
      if (totalExpected > p->size) {
        LOGE(TAG, "Image too big (%u) for OTA partition (%u)", (unsigned)totalExpected, (unsigned)p->size);
        free(buf); return;
      }
    }
#endif
  }

  for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setUserAgent("ESP32-OTA");
    http.setTimeout(HTTP_TIMEOUT_MS);

    if (!http.begin(tls, OTA_host)) { LOGE(TAG, "HTTP begin() failed"); break; }

    if (offset > 0) {
      String range = "bytes=" + String(offset) + "-";
      http.addHeader("Range", range);
      LOGI(TAG, "Resuming with %s", range.c_str());
    }

    int code = http.GET();
    LOGI(TAG, "HTTP GET -> code=%d (%s)", code, HTTPClient::errorToString(code).c_str());
    if (!((code == HTTP_CODE_OK && offset == 0) ||
          (code == HTTP_CODE_PARTIAL_CONTENT && offset > 0))) {
      http.end();
      continue; // try again
    }

    // If totalExpected is still unknown and server tells us now, record it
    if (!totalExpected) {
      if (offset == 0) {
        int cl = http.getSize();
        if (cl > 0) totalExpected = cl;
      } else {
        String cr = http.header("Content-Range"); // bytes a-b/total
        int slash = cr.lastIndexOf('/');
        if (slash >= 0) {
          size_t t = cr.substring(slash + 1).toInt();
          if (t > 0) totalExpected = t;
        }
      }
    }

    // Begin Update once, with total size if known
    if (!begun) {
      size_t beginSize = totalExpected ? totalExpected : UPDATE_SIZE_UNKNOWN;
      if (!Update.begin(beginSize)) {
        LOGE(TAG, "Update.begin() failed");
        Update.printError(Serial);
        LOGE(TAG, "Update error: %s", Update.errorString());
        http.end(); free(buf); return;
      }
      LOGI(TAG, "Update.begin() OK. Writing image...");
      begun = true;
    }

    WiFiClient* stream = http.getStreamPtr();
    stream->setTimeout(15000);

    unsigned long lastActivity = millis();
    int32_t partLen = http.getSize(); // may be -1
    int32_t remaining = partLen;

    while (remaining != 0) {
      size_t avail = stream->available();
      if (avail) {
        size_t toRead = (remaining > 0) ? (size_t)min((int32_t)avail, remaining) : (size_t)avail;
        if (toRead > CHUNK) toRead = CHUNK;

        int n = stream->readBytes(buf, toRead);
        if (n <= 0) { LOGE(TAG, "Read error"); break; }

        size_t w = Update.write(buf, n);
        if (w != (size_t)n) {
          LOGE(TAG, "Flash write mismatch (%u/%d)", (unsigned)w, n);
          Update.printError(Serial);
          LOGE(TAG, "Update error: %s", Update.errorString());
          http.end(); Update.abort(); free(buf); return;
        }
        offset += n;
        if (remaining > 0) remaining -= n;
        lastActivity = millis();

        client.loop(); // keep Wi-Fi/MQTT alive
        delay(0);
      } else {
        if (millis() - lastActivity > STREAM_IDLE_MS) {
          LOGW(TAG, "Idle > %lu ms; will resume at %u", (unsigned long)STREAM_IDLE_MS, (unsigned)offset);
          break;
        }
        delay(1);
      }
    }

    http.end();

    // If we finished this part (or unknown length ended), decide if done
    if ((remaining == 0) || (partLen < 0)) {
      if (totalExpected && offset >= totalExpected) {
        LOGI(TAG, "All bytes received: %u / %u", (unsigned)offset, (unsigned)totalExpected);
        break; // exit retry loop, finalize below
      }
      // else loop again to fetch remainder
    }
  }

  // Finalize (force=true avoids false "Aborted" when size bookkeeping disagrees)
  if (!Update.end(true)) {
    LOGE(TAG, "Update.end() failed.");
    Update.printError(Serial);
    LOGE(TAG, "Update error: %s", Update.errorString());
    free(buf);
    return;
  }
  if (!Update.isFinished()) {
    LOGE(TAG, "Update not finished (bytes=%u)", (unsigned)Update.progress());
    free(buf);
    return;
  }

  free(buf);
  LOGI(TAG, "OTA successful (bytes=%u). Rebooting...", (unsigned)Update.size());
  ESP.restart();
}


/*********************  Action helpers & evaluation  *********************/
static bool parseAction(const String& in, ParsedAction& out) {
  out = ParsedAction();
  String s = in; s.trim();
  if (s.length() == 0) {
    debug_print_parsed_action(in.c_str(), "", 0, false, false);
    return false;
  }
  s.toUpperCase();

  bool tailOn  = s.endsWith("ON");
  bool tailOff = s.endsWith("OFF");
  if (!tailOn && !tailOff) { debug_print_parsed_action(in.c_str(), "", 0, false, false); return false; }
  out.on = tailOn;

  String core = tailOn ? s.substring(0, s.length()-2) : s.substring(0, s.length()-3);

  int spos = core.indexOf('S');
  String dev = core, sNum = "";
  if (spos >= 0) {
    dev  = core.substring(0, spos);
    sNum = core.substring(spos+1);
    if (sNum.length() == 0) { debug_print_parsed_action(in.c_str(), "", 0, out.on, false); return false; }
    out.switchNo = sNum.toInt();
    if (out.switchNo <= 0) { debug_print_parsed_action(in.c_str(), "", 0, out.on, false); return false; }
  } else {
    out.switchNo = 1;
  }

  if (dev.length() < 3) { debug_print_parsed_action(in.c_str(), "", out.switchNo, out.on, false); return false; }
  String prefix = dev.substring(0,2);
  int idxNum    = dev.substring(2).toInt();
  if (idxNum < 0) { debug_print_parsed_action(in.c_str(), "", out.switchNo, out.on, false); return false; }

  if (!(prefix == "TM" || prefix == "MC" || prefix == "VC" || prefix == "WC" || prefix == "BM")) {
    debug_print_parsed_action(in.c_str(), "", out.switchNo, out.on, false);
    return false;
  }

  out.devCode = prefix;
  out.valid   = true;
  debug_print_parsed_action(in.c_str(), out.devCode, out.switchNo, out.on, true);
  return true;
}
static int findArrIndexByDeviceCode(const String& code) {
  for (int i=0;i<20;i++) if (arr[i].device_name == code) return i;
  return index_from_code(code);
}
static void sendLoRaSwitchCmd(int arrIdx, int switchNo, bool turnOn) {
  if (arrIdx < 0 || arrIdx >= 20) return;
  if (arr[arrIdx].addr_high == 0 && arr[arrIdx].addr_low == 0) {
    Serial.printf("[Action] sendLoRaSwitchCmd: arr[%d] has no address, skipping.\n", arrIdx);
    return;
  }
  DynamicJsonDocument doc(256);
  doc["deviceid"]   = arr[arrIdx].device_id;
  doc["switch_no"]  = "S" + String(switchNo);
  doc["status"]     = turnOn ? "on" : "off";
  String payload; serializeJson(doc, payload);
  Serial.printf("[Action] -> LoRa to %s (idx=%d) ah=%u al=%u ch=%u switch=S%d state=%s payload=%s\n",
                arr[arrIdx].device_name.c_str(), arrIdx,
                arr[arrIdx].addr_high, arr[arrIdx].addr_low, arr[arrIdx].channel,
                switchNo, turnOn ? "on" : "off", payload.c_str());
  Lora_send(arr[arrIdx].addr_high, arr[arrIdx].addr_low, arr[arrIdx].channel, payload);
}
static int pickSenseSwitchFromActions(const String& A1, const String& A2, const String& A3) {
  ParsedAction p;
  if (parseAction(A1, p) && p.switchNo > 0) return p.switchNo;
  if (parseAction(A2, p) && p.switchNo > 0) return p.switchNo;
  if (parseAction(A3, p) && p.switchNo > 0) return p.switchNo;
  return 1;
}
/********** local base-module S1/S2 handler for BM0 **********/
static bool handleBM0Local(const String& raw, bool invert) {
  String s = raw; s.trim(); s.toUpperCase();
  if (!(s.startsWith("BM0S1") || s.startsWith("BM0S2"))) return false;

  const bool reqOn  = s.endsWith("ON");              // requested state from action
  const bool finalOn = invert ? !reqOn : reqOn;      // APPLY INVERSION HERE
  const bool isS1   = s.startsWith("BM0S1");
  String target = isS1 ? "S1" : "S2";

  for (int i = 0; i < pinLength; i++) {
    if (relays[i] == target) {
      if (finalOn) { onWrite(LOW,  pins[i]); power_status[i] = 1; motor_led(i, LOW);  }
      else         { onWrite(HIGH, pins[i]); power_status[i] = 0; motor_led(i, HIGH); }
      Publish_Update(power_status[i], i);
      storeRelayStates();
      Serial.printf("[Action][BM0] Local %s -> %s (invert=%d, requested=%s)\n",
                    target.c_str(), finalOn ? "ON" : "OFF", (int)invert, reqOn ? "ON" : "OFF");
      return true;
    }
  }
  Serial.printf("[Action][BM0] Relay label %s not found in relays[] mapping.\n", target.c_str());
  return true; // format recognized even if label missing
}

static void runActions(const String& A1, const String& A2, const String& A3, bool invert) {
  ParsedAction acts[3];
  String raw[3] = {A1, A2, A3};
  for (int k=0;k<3;k++) {
    // BM0 local actions now honor inversion:
    if (handleBM0Local(raw[k], invert)) continue;

    if (!parseAction(raw[k], acts[k])) {
      if (raw[k].length()) Serial.printf("[Action] Skipping invalid action \"%s\"\n", raw[k].c_str());
      continue;
    }
    int idx = findArrIndexByDeviceCode(acts[k].devCode);
    if (idx < 0) {
      Serial.printf("[Action] Device code %s not found in arr[], skipping.\n", acts[k].devCode.c_str());
      continue;
    }
    bool finalState = invert ? !acts[k].on : acts[k].on;
    Serial.printf("[Action] Executing %s S%d %s (invert=%d) => idx=%d\n",
                  acts[k].devCode.c_str(), acts[k].switchNo, finalState ? "ON" : "OFF", invert, idx);
    sendLoRaSwitchCmd(idx, acts[k].switchNo, finalState);
  }
}


/* Evaluate one Primary block with PERCENT thresholds mapped to absolute via range.
   NEW: Actions gated by Trigger_flag / Stop_flag. */
static void evalPrimaryBlock(const String& slaveName,
                             const String& parameter,
                             int stopPercent, int triggerPercent,
                             const String& A1, const String& A2, const String& A3) {
  int srcIdx = -1;
  for (int i = 0; i < 20; i++) {
    if (arr[i].device_name == slaveName) { srcIdx = i; break; }
  }
  if (srcIdx < 0) {
    Serial.printf("[Eval] Primary for %s: device not found in arr[]\n", slaveName.c_str());
    return;
  }

  // Convenience: address & device id for this slave
  auto sendNoActionStatus = [&](const char* why){
    String deviceId = (arr[srcIdx].device_id.length() ? arr[srcIdx].device_id : slaveName);
    StaticJsonDocument<128> doc;
    doc["deviceid"] = DeviceId;
    doc["flag"]     = true;   // working_flag == 1
    doc["reset"]    = false;  // as requested
    String payload; serializeJson(doc, payload);

    uint8_t addh = arr[srcIdx].addr_high;
    uint8_t addl = arr[srcIdx].addr_low;
    uint8_t chan = arr[srcIdx].channel;

    Serial.printf("[LoRa][NoAction:%s] TX to %02X:%02X ch=%d : %s\n",
                  why, addh, addl, chan, payload.c_str());
    Lora_send(addh, addl, chan, payload);
  };

  if (parameter == "distance" || slaveName.startsWith("TM")) {
    const long rng   = arr[srcIdx].range;
    const long sense = arr[srcIdx].distance;

    if (rng <= 0) {
      Serial.println("[Eval] Range not set yet (0). Skipping actions.");
      sendNoActionStatus("range=0");
      Serial.printf("[Eval] slave=%s flags: Trigger=%d Stop=%d\n",
                    slaveName.c_str(), (int)Trigger_flag, (int)Stop_flag);
      return;
    }

    long triggerAbs = (long)stopPercent    * rng / 100L;  // MAX
    long stopAbs = (long)triggerPercent * rng / 100L;  // MIN

    Serial.printf("[Eval] Range=%ld cm, Max=%d%%->%ld cm, Min=%d%%->%ld cm, Distance=%ld cm\n",
                  rng, stopPercent, stopAbs, triggerPercent, triggerAbs, sense);

    if (sense > stopAbs && sense < triggerAbs) {
      if (Trigger_flag || Stop_flag) {
        Serial.println("[Flags] Clearing Trigger_flag & Stop_flag (mid band)");
      }
      Trigger_flag = false;
      Stop_flag    = false;
      Serial.println("[Eval] Between MIN% and MAX% -> NO ACTION");
      sendNoActionStatus("mid-band");
    } else if (sense <= stopAbs) {
      if (!Stop_flag) Serial.println("[Flags] Stop_flag RAISED, Trigger_flag CLEARED");
      Stop_flag    = true;
      Trigger_flag = false;
      if (Stop_flag) {
        Serial.println("[Eval] >= MAX% -> INVERTED actions (gated by Stop_flag)");
        runActions(A1, A2, A3, /*invert=*/true);
      }
    } else if (sense >= triggerAbs){ // sense >= triggerAbs
      if (!Trigger_flag) Serial.println("[Flags] Trigger_flag RAISED, Stop_flag CLEARED");
      Trigger_flag = true;
      Stop_flag    = false;
      if (Trigger_flag) {
        Serial.println("[Eval] <= MIN% -> NORMAL actions (gated by Trigger_flag)");
        runActions(A1, A2, A3, /*invert=*/false);
      }
    }
  } else {
    int sNo = pickSenseSwitchFromActions(A1, A2, A3);
    sNo = constrain(sNo, 1, 8);
    int sense = arr[srcIdx].sw_no[sNo - 1] ? 1 : 0;

    if (sense == triggerPercent) {
      if (!Trigger_flag) Serial.println("[Flags] Trigger_flag RAISED (switch), Stop_flag CLEARED");
      Trigger_flag = true; 
      Stop_flag    = false;
      Serial.println("[Eval] Switch matches trigger -> NORMAL actions (gated)");
      runActions(A1, A2, A3, /*invert=*/false);
    } else {
      if (Trigger_flag || Stop_flag) Serial.println("[Flags] Clearing flags (switch mismatch)");
      Trigger_flag = false; 
      Stop_flag    = false;
      Serial.println("[Eval] Switch not matching -> NO ACTION");
      sendNoActionStatus("switch-mismatch");
    }
  }

  Serial.printf("[Eval] slave=%s flags: Trigger=%d Stop=%d\n",
                slaveName.c_str(), (int)Trigger_flag, (int)Stop_flag);
}

static inline bool isPrimary1Empty() {
  return gPrimary1.slave.length() == 0 &&
         gPrimary1.parameter.length() == 0 &&
         gPrimary1.A1.length() == 0 &&
         gPrimary1.A2.length() == 0 &&
         gPrimary1.A3.length() == 0 &&
         gPrimary1.data1 == 0 &&
         gPrimary1.data2 == 0;
}
static inline bool isPrimary2Empty() {
  return gPrimary2.slave.length() == 0 &&
         gPrimary2.parameter.length() == 0 &&
         gPrimary2.A1.length() == 0 &&
         gPrimary2.A2.length() == 0 &&
         gPrimary2.A3.length() == 0 &&
         gPrimary2.data1 == 0 &&
         gPrimary2.data2 == 0;
}

/*********************  LoRa send/recv  *********************/
void Lora_send(byte addh, byte addl, byte chan, String datas) {
  ResponseStatus rs = e220ttl.sendFixedMessage(addh, addl, chan, datas.c_str());
  Serial.printf("[LoRa][Send] ah=%u al=%u ch=%u payload=%s -> %s\n", addh, addl, chan, datas.c_str(), rs.getResponseDescription().c_str());
}

void Lora_receive() {
  if (e220ttl.available() <= 1) { delay(50); return; }

  Serial.println("Message received!");
#ifdef ENABLE_RSSI
  ResponseContainer rc = e220ttl.receiveMessageRSSI();
#else
  ResponseContainer rc = e220ttl.receiveMessage();
#endif
  if (rc.status.code != 1) {
    Serial.println(rc.status.getResponseDescription());
    delay(50);
    return;
  }

  Serial.println("data dump to memory");
  Serial.println(rc.status.getResponseDescription());
  Serial.println(rc.data);

  // Parse incoming JSON
  DynamicJsonDocument loc(1024);
  if (deserializeJson(loc, rc.data)) {
    Serial.println("[LoRa] JSON parse error");
    delay(50);
    return;
  }

  String DeviceId_lora = loc["deviceid"].isNull() ? "#" : loc["deviceid"].as<String>();
  String sensor        = loc["sensor"].isNull()   ? "#" : loc["sensor"].as<String>();
  int Tank_height      = loc["height"].isNull()   ? 0   : loc["height"].as<int>();    // 'range'
  int Tank_cap         = loc["capacity"].isNull() ? 0   : loc["capacity"].as<int>();
  int datas            = loc["data"].isNull()     ? 0   : loc["data"].as<int>();

  // Update arr[] for TM sensors (distance + optional range)
  int idx = tank_index_from_name(sensor);
  if (idx >= 0) {
    arr[idx].device_name = sensor;
    if (DeviceId_lora != "#") arr[idx].device_id = DeviceId_lora;
    arr[idx].mode    = 3;
    arr[idx].channel = channel;
    if (datas > 0)       arr[idx].distance = datas;
    if (Tank_height > 0) arr[idx].range    = Tank_height;
    debug_print_arr_entry(idx, "Lora_receive: after TM update");
    save_slave_entry_nvs(idx);
  }

  // LEDs based on range + distance (for TM1/TM2)
  if (idx >= 0) updateTankLEDs(sensor, arr[idx].distance, arr[idx].range);

  // Publish percentage (0..100). If no range yet, publish 0.
  if (idx >= 0) {
    int rng = arr[idx].range;
    int dist= arr[idx].distance;
    int percent_full = (rng > 0) ? (int)((long)(rng - dist) * 100L / rng) : 0;
    percent_full = constrain(percent_full, 0, 100);
    Publish_Updates(percent_full, sensor);
  }

  // Run Primary1 / Primary2 only if incoming sensor matches those slots
  if (!isPrimary1Empty() && sensor == gPrimary1.slave) {
    Serial.println("[LoRa] Incoming matches Primary1 -> evaluate (flag-gated)");
    evalPrimaryBlock(
      gPrimary1.slave,
      gPrimary1.parameter.length() ? gPrimary1.parameter : (sensor.startsWith("TM") ? "distance" : "switch"),
      gPrimary1.data1,  // max%
      gPrimary1.data2,  // min%
      gPrimary1.A1, gPrimary1.A2, gPrimary1.A3
    );
  }
  if (!isPrimary2Empty() && sensor == gPrimary2.slave) {
    Serial.println("[LoRa] Incoming matches Primary2 -> evaluate (flag-gated)");
    evalPrimaryBlock(
      gPrimary2.slave,
      gPrimary2.parameter.length() ? gPrimary2.parameter : (sensor.startsWith("TM") ? "distance" : "switch"),
      gPrimary2.data1,  // max%
      gPrimary2.data2,  // min%
      gPrimary2.A1, gPrimary2.A2, gPrimary2.A3
    );
  }

  delay(50);
}

/*********************  Setup  *********************/
void setup() {
  Serial.begin(115200);
  Serial.println();

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);

  e220ttl.begin();
  delay(800);

  {
    ResponseStructContainer c = e220ttl.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    Serial.println(c.status.getResponseDescription());
    Serial.println(c.status.code);
    printParameters(configuration);
    c.close();

    ResponseStructContainer cMi = e220ttl.getModuleInformation();
    ModuleInformation mi = *(ModuleInformation*)cMi.data;
    Serial.println(cMi.status.getResponseDescription());
    Serial.println(cMi.status.code);
    printModuleInformation(mi);
    cMi.close();
  }

  Serial.println("Connecting for LORA");

  for (int i = 0; i < pinLength; i++) pinMode(pins[i], OUTPUT);
  for (int i = 0; i < pinLength; i++) pinMode(input[i], INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  /*** First-boot block using prefDatas ***/
  prefDatas.begin("datas", false);
  flag = prefDatas.getBool("flag", flag);

  if (flag == 0) {
    Serial.println("First-time Device Configuration");
    prefDatas.putString("thingid", ThingId);
    prefDatas.putString("deviceid", DeviceId);
    prefDatas.putString("CA", rootCACertificate);
    prefDatas.putString("Device_cert", clientCertificate);
    prefDatas.putString("Private", clientPrivateKey);
    // prefDatas.putString("Pubic", clientPublicKey);
    prefDatas.putString("Token", Access_token);
    prefDatas.putString("Secret", Secret);
    prefDatas.putString("Server", Mqtt_server);
    prefDatas.putString("OTA_host", OTA_host);

    prefDatas.putString("ssid", ssid);
    prefDatas.putString("password", password);
    prefDatas.putBool("state", 0);
    prefDatas.putInt("mode", 1);
    Serial.println("put 1 in the NVS for modes");
    prefDatas.putBool("flag", 1);
    // prefDatas.putInt("tank", 0);

    for (int i = 0; i < pinLength; i++) {
      prefDatas.putBool(("S" + String(i + 1)).c_str(), power_status[i]);
    }

    {
      ResponseStructContainer c = e220ttl.getConfiguration();
      Configuration configuration = *(Configuration*) c.data;

      configuration.ADDL = address_l;
      configuration.ADDH = address_h;
      configuration.CHAN = channel;

      configuration.SPED.uartBaudRate = UART_BPS_9600;
      configuration.SPED.airDataRate  = AIR_DATA_RATE_010_24;
      configuration.SPED.uartParity   = MODE_00_8N1;

      configuration.OPTION.subPacketSetting  = SPS_200_00;
      configuration.OPTION.RSSIAmbientNoise  = RSSI_AMBIENT_NOISE_DISABLED;
      configuration.OPTION.transmissionPower = POWER_22;

      configuration.TRANSMISSION_MODE.enableRSSI        = RSSI_DISABLED;
      configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
      configuration.TRANSMISSION_MODE.enableLBT         = LBT_DISABLED;
      configuration.TRANSMISSION_MODE.WORPeriod         = WOR_2000_011;

      ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
      Serial.println(rs.getResponseDescription());
      Serial.println(rs.code);

      c.close();

      ResponseStructContainer c2 = e220ttl.getConfiguration();
      configuration = *(Configuration*) c2.data;
      Serial.println(c2.status.getResponseDescription());
      Serial.println(c2.status.code);
      printParameters(configuration);
      c2.close();
    }

    Serial.print("ssid="); Serial.println(prefDatas.getString("ssid", ""));
    Serial.print("password="); Serial.println(prefDatas.getString("password", ""));

    // Initialize & save empty slave table using separate prefSlots
    init_slave_table();
    save_slave_table_nvs();
  }
  prefDatas.end();

  Serial.println();
  Serial.println("Device initialing....");

  // Reopen prefDatas to read values
  prefDatas.begin("datas", true);
  ssid             = prefDatas.getString("ssid", "");
  password         = prefDatas.getString("password", "");
  states           = prefDatas.getBool("state", states);
  modes            = prefDatas.getInt("mode", modes);
  flag             = prefDatas.getBool("flag", flag);
  ThingId          = prefDatas.getString("thingid", "");
  DeviceId         = prefDatas.getString("deviceid", "");
  rootCACertificate= prefDatas.getString("CA", "");
  clientCertificate= prefDatas.getString("Device_cert", "");
  clientPrivateKey = prefDatas.getString("Private", "");
  // clientPublicKey  = prefDatas.getString("Pubic", "");
  Access_token     = prefDatas.getString("Token", "");
  Secret           = prefDatas.getString("Secret", "");
  Mqtt_server      = prefDatas.getString("Server", "");
  OTA_host         = prefDatas.getString("OTA_host", "");
  int tank_counter = prefDatas.getInt("tank", 0);
  prefDatas.end();

  for (int i = 0; i < pinLength; i++) {
    // read per-relay states from prefDatas
    prefDatas.begin("datas", true);
    power_status[i] = prefDatas.getBool(("S" + String(i)).c_str(), power_status[i]);
    prefDatas.end();

    digitalWrite(pins[i], power_status[i] ? HIGH : LOW);
    Serial.println("Relay" + String(i + 1) + ": " + String(power_status[i]));
    motor_led(i, power_status[i]);
  }

  test_led();
  leds[2] = CRGB::Green; FastLED.show();
  Serial.print("Distance of Tank= "); Serial.println(Measure());

  // Load slave table from prefSlots
  bool anyLoaded = false;
  for (int i=0;i<20;i++) if (load_slave_entry_nvs(i)) anyLoaded = true;
  if (!anyLoaded) {
    init_slave_table();
    save_slave_table_nvs();
  }

  // Dump arr table after NVS load/init
  debug_dump_arr_table("setup(): after NVS load/init");

  UpdateTopic        = "$aws/things/" + String(ThingId) + "/update";
  OtavalidateTopic   = "$aws/things/" + String(ThingId) + "/ota/validate";
  AliveTopic         = "$aws/things/" + String(ThingId) + "/alive_reply";
  HealthTopic        = "$aws/things/" + String(ThingId) + "/health_reply";
  SlaveTopic         = "$aws/things/" + String(ThingId) + "/slave_response";
  StatusTopic        = "$aws/things/" + String(ThingId) + "/status_response";

  SettingTopic       = "mqtt/device/" +  String(ThingId) + "/setting";
  ConfigTopic        = "mqtt/device/" +  String(ThingId) + "/config";
  ControlTopic       = "mqtt/device/" +  String(ThingId) + "/control";
  isAliveTopic       = "mqtt/device/" +  String(ThingId) + "/alive";
  isHealthTopic      = "mqtt/device/" +  String(ThingId) + "/health";
  OtaintializeTopic  = "mqtt/device/" +  String(ThingId) + "/ota/intialize";
  OtarequestTopic    = "mqtt/device/" +  String(ThingId) + "/ota/request";
  ResetTopic         = "mqtt/device/" +  String(ThingId) + "/reset";
  SlaverequestTopic  = "mqtt/device/" +  String(ThingId) + "/slave_request";
  StatusrequestTopic = "mqtt/device/" +  String(ThingId) + "/status_request";
  DeleteSlaveTopic   = "mqtt/device/" +  String(ThingId) + "/slave/delete";

  Serial.print("ssid="); Serial.println(ssid);
  Serial.print("password="); Serial.println(password);
  Serial.print("Configure status="); Serial.println(states);
  Serial.print("Device mode="); Serial.println(modes);
  Serial.print("SettingTopic="); Serial.println(SettingTopic);
  Serial.print("UpdateTopic="); Serial.println(UpdateTopic);
  Serial.print("Firmware Version="); Serial.println(FirmwareVer);

  reconnect();
  initial = 1;
}

/*********************  E220 debug prints  *********************/
void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX); Serial.print(" ");
  Serial.print(configuration.STARTING_ADDRESS, HEX); Serial.print(" ");
  Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN); Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN); Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN); Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN); Serial.print(" -> ");
  Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN); Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN); Serial.print(" -> ");
  Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN); Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN); Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN); Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN); Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());
  Serial.println("----------------------------------------");
}
void printModuleInformation(struct ModuleInformation moduleInformation) {
  Serial.println("----------------------------------------");
  DEBUG_PRINT(F("HEAD: "));  DEBUG_PRINT(moduleInformation.COMMAND, HEX); DEBUG_PRINT(" ");
  DEBUG_PRINT(moduleInformation.STARTING_ADDRESS, HEX); DEBUG_PRINT(" "); DEBUG_PRINTLN(moduleInformation.LENGHT, DEC);
  Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, HEX);
  Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}

/*********************  IO handling  *********************/
void readInputs() {
  for (int i = 0; i < pinLength-1; i++) {
    inputs[i] = digitalRead(input[i]);
    delay(40);
  }
}
void onWrite(bool status, int pin) {
  delay(100);
  if (status) { digitalWrite(pin, LOW);  Serial.println("Lamp is OFF"); }
  else        { digitalWrite(pin, HIGH); Serial.println("Lamp is ON");  }
}
void storeRelayStates() {
  prefDatas.begin("datas", false);
  for (int i = 0; i < pinLength; i++) {
    prefDatas.putBool(("S" + String(i)).c_str(), power_status[i]);
  }
  prefDatas.end();
  Serial.println("Relay states saved to preferences.");
}
void reseting() {
  prefDatas.begin("datas", false);
  prefDatas.putString("ssid", "R&D");
  prefDatas.putString("password", "RND@1980");
  prefDatas.putInt("mode", 1);
  prefDatas.putBool("state", 0);
  // prefDatas.putInt("tank", 0);
  prefDatas.end();

  for (int i = 0; i <= 20; i++) {
    DynamicJsonDocument doc(1024);
    doc["deviceid"] = DeviceId;
    doc["woking_flag"] = 0;
    doc["reset"] = 1;
    int add_l = address_l + i;
    String payload; serializeJson(doc, payload);
    Lora_send(address_h, add_l, channel, payload.c_str());
    delay(50);
  }
  delay(800);
  ESP.restart();
}
void reseting_slave(String id, String sensor) {
  // no datas write here; only LoRa
  for (int i = 0; i <= 20; i++) {
    if (arr[i].device_id == id) {
      DynamicJsonDocument doc(1024);
      doc["deviceid"] = id;
      doc["woking_flag"] = 0;
      doc["reset"] = 1;
      int add_l = arr[i].addr_low;
      String payload; serializeJson(doc, payload);
      Lora_send(arr[i].addr_high, add_l, arr[i].channel, payload.c_str());
      i = 20;
    }
  }
  delay(800);
}
void handleInputChanges() {
  for (int i = 0; i < pinLength-1; i++) {
    if (inputs[i] != inputs_last[i] ) {
      Serial.print("Input "); Serial.print(i + 1);
      Serial.print(" received: "); Serial.println(inputs[i]);

      if (inputs[0] != inputs_last[0] && input_counter == 0 ) {
        input_counter = input_counter + 1;
        input_time = millis();
        Serial.println(" Reset started");
      }
      if (inputs[0] != inputs_last[0] && millis() - input_time < 10000) {
        input_counter = input_counter + 1;
        Serial.print(" Reset count="); Serial.println(input_counter);
      }
      if (millis() - input_time > 10000 && input_counter > 0) {
        input_counter = 0;
        input_time = 0;
        Serial.println(" Reset count closed");
      }
      if (input_counter >= 15) {
        Serial.println(" Reset detected");
        reseting();
        input_counter = 0;
      }

      onWrite(inputs[i], pins[i]);
      power_status[i] = inputs[i];
      storeRelayStates();
      inputs_last[i] = inputs[i];
      Publish_Update(power_status[i], i);
    }
  }
}

/*********************  Version compare  *********************/
bool isVersionNewer(String current, String ota) {
  int currentMajor, currentMinor, currentPatch;
  int otaMajor, otaMinor, otaPatch;
  sscanf(current.c_str(), "%d.%d.%d", &currentMajor, &currentMinor, &currentPatch);
  sscanf(ota.c_str(), "%d.%d.%d", &otaMajor, &otaMinor, &otaPatch);
  if (otaMajor > currentMajor) return true;
  if (otaMajor == currentMajor && otaMinor > currentMinor) return true;
  if (otaMajor == currentMajor && otaMinor == currentMinor && otaPatch > currentPatch) return true;
  return false;
}

/*********************  MQTT callback  *********************/
void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on "); Serial.print(topic); Serial.print(": ");

  if (strcmp(topic, ControlTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
    String relay    = root["switch_no"].isNull() ? "#" : root["switch_no"].as<String>();
    String power    = root["status"].isNull()    ? "#" : root["status"].as<String>();

    Serial.println(deviceid);
    Serial.println(relay);
    Serial.println(power);

    if (deviceid == DeviceId && relay != "#" && power != "#") {
      for (int i = 0; i < pinLength; i++) {
        if (relay == relays[i]) {
          if (power == "on") {
            onWrite(LOW, pins[i]); power_status[i] = 1; motor_led(i, LOW);
          } else if (power == "off") {
            onWrite(HIGH, pins[i]); power_status[i] = 0; motor_led(i, HIGH);
          } else {
            Serial.println("error in status");
          }
          Publish_Update(power_status[i], i);
        }
      }
      storeRelayStates();
    }
    else if (deviceid != "#" && deviceid != DeviceId) {
      int idx = -1;
      for (int i=0;i<20;i++) if (arr[i].device_id == deviceid){ idx=i; break; }
      if (idx >= 0) {
        String forward; serializeJson(root, forward);
        Lora_send(arr[idx].addr_high, arr[idx].addr_low, arr[idx].channel, forward);
        Serial.println("Forwarded CONTROL to slave via LoRa.");
      } else {
        Serial.println("Wrong deviceid (no such slave in table).");
      }
    }
    else {
      Serial.println("Wrong deviceid or payload");
    }
  }
  else if (strcmp(topic, OtarequestTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid      = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
    String ota_version   = root["ota_version"].isNull() ? "#" : root["ota_version"].as<String>();
    String transactionId = root["transactionId"].isNull() ? "#" : root["transactionId"].as<String>();

    Serial.println(deviceid);
    Serial.println(transactionId);
    if (deviceid == DeviceId) {
      String hashInput = transactionId + ThingId;
      SHA256 sha256; sha256.update(hashInput.c_str(), hashInput.length());
      uint8_t hash[SHA256_HASH_SIZE]; sha256.finalize(hash, SHA256_HASH_SIZE);
      String hashedString;
      for (int i = 0; i < SHA256_HASH_SIZE; i++) { char hex[3]; sprintf(hex, "%02x", hash[i]); hashedString += hex; }
      Publish_OTA_Valid(transactionId, hashedString, FirmwareVer, "Ready for OTA");
    } else {
      Serial.println("Wrong deviceid ");
      Publish_OTA_Valid(transactionId, "#", FirmwareVer, "DeviceID Wrong");
    }
  }
  else if (strcmp(topic, OtaintializeTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid    = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
    String ota_version = root["ota_version"].isNull() ? "#" : root["ota_version"].as<String>();
    OTA_host           = root["url"].isNull() ? "#" : root["url"].as<String>();

    prefDatas.begin("datas", false);
    prefDatas.putBool("state", 0);
    prefDatas.putString("OTA_host", OTA_host);
    prefDatas.end();

    Serial.println(deviceid);
    if (deviceid == DeviceId) {
      if (isVersionNewer(FirmwareVer, ota_version)) {
        Publish_OTA_Valids(ThingId, OTA_host, FirmwareVer, "OTA updating");
        firmwareUpdate();
      } else {
        Publish_OTA_Valids(ThingId, OTA_host, FirmwareVer, "Version Upto date");
        Serial.println("Current version is up to date.");
      }
    } else {
      Publish_OTA_Valids(ThingId, OTA_host, FirmwareVer, "DeviceID Wrong");
    }
  }
  else if (strcmp(topic, SettingTopic.c_str()) == 0) {
    // Primary1/Primary2 values:
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid      = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
    String sensor_number = root["sensor_no"].isNull() ? "#" : root["sensor_no"].as<String>();
    String slot          = root["slot"].isNull() ? "#" : root["slot"].as<String>();
    int maximum          = root["maximum"].isNull() ? 0   : root["maximum"].as<int>();   // PERCENT of range
    int minimum          = root["minimum"].isNull() ? 0   : root["minimum"].as<int>();   // PERCENT of range
    String A1            = root["A1"].isNull() ? "" : root["A1"].as<String>();
    String A2            = root["A2"].isNull() ? "" : root["A2"].as<String>();
    String A3            = root["A3"].isNull() ? "" : root["A3"].as<String>();

    // optional addressing/slot info for slave table
    uint8_t ch = root["channel"].isNull()    ? channel     : (uint8_t)root["channel"].as<int>();
    uint8_t ah = root["address_h"].isNull()  ? address_h   : (uint8_t)root["address_h"].as<int>();
    uint8_t al = root["address_l"].isNull()  ? address_l   : (uint8_t)root["address_l"].as<int>();
    int tank_len = root["height"].isNull()   ? 0           : root["height"].as<int>();      // may set range
    String swNo  = root["switch_no"].isNull()? ""          : root["switch_no"].as<String>(); // for MC/BM

    Serial.println("Device ID: " + deviceid);
    Serial.println("sensor_no: " + sensor_number);
    Serial.println("slot : " + slot);
    Serial.print("max%: "); Serial.println(maximum);
    Serial.print("min%: "); Serial.println(minimum);

    if (deviceid == DeviceId) {
      // Update primary struct (max/min are in PERCENT of range)
      if (slot == "Primary2" || slot == "P2") {
        gPrimary2.slave     = sensor_number;
        gPrimary2.A1        = A1;
        gPrimary2.A2        = A2;
        gPrimary2.A3        = A3;
        gPrimary2.parameter = sensor_number.startsWith("TM") ? "distance" : "switch";
        gPrimary2.data1     = maximum;   // %
        gPrimary2.data2     = minimum;   // %
        debug_print_primary2("SettingTopic: updated P2");
      } else {
        gPrimary1.slave     = sensor_number;
        gPrimary1.A1        = A1;
        gPrimary1.A2        = A2;
        gPrimary1.A3        = A3;
        gPrimary1.parameter = sensor_number.startsWith("TM") ? "distance" : "switch";
        gPrimary1.data1     = maximum;   // %
        gPrimary1.data2     = minimum;   // %
        debug_print_primary1("SettingTopic: updated P1");
      }

      // Update the slave addressing info as requested
      int idx = index_from_code(sensor_number);
      if (idx >= 0) {
        bool sw0 = (swNo.length() > 0);
        set_slave(idx, sensor_number, arr[idx].device_id, 1, ch, ah, al,
                  (sensor_number.startsWith("TM") ? tank_len : arr[idx].range),
                  arr[idx].distance, arr[idx].Ph, arr[idx].TDS, arr[idx].turbulity, sw0);
        debug_print_arr_entry(idx, "SettingTopic: after set_slave");
      }
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, ConfigTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid       = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
    String ssid_e         = root["ssid"].isNull() ? "#" : root["ssid"].as<String>();
    String password_e     = root["password"].isNull() ? "#" : root["password"].as<String>();
    String access_token_e = root["access_token"].isNull() ? "#" : root["access_token"].as<String>();
    String secret_e       = root["secret"].isNull() ? "#" : root["secret"].as<String>();
    String ota_host_e     = root["ota_host"].isNull() ? "#" : root["ota_host"].as<String>();
    String mo             = root["mode"].isNull() ? "#" : root["mode"].as<String>();

    Serial.println(deviceid);
    Serial.println("SSID: " + ssid_e);
    Serial.println("Password: " + password_e);
    Serial.println("access_token: " + access_token_e);
    Serial.println("secret: " + secret_e);
    Serial.println("ota_host: " + ota_host_e);

    if (deviceid == DeviceId) {
      prefDatas.begin("datas", false);
      if (ssid_e != "#" && password_e != "#") {
        prefDatas.putString("ssid", ssid_e);
        prefDatas.putString("password", password_e);
      }
      if (mo != "#") {
        if (mo == "1")      prefDatas.putInt("mode", 1);
        else if (mo == "2") prefDatas.putInt("mode", 2);
        else if (mo == "3") prefDatas.putInt("mode", 3);
        else                prefDatas.putInt("mode", 1);
        prefDatas.putBool("state", 0);
      }
      if (access_token_e != "#") prefDatas.putString("Access_token", access_token_e);
      if (secret_e      != "#") prefDatas.putString("Secret", secret_e);
      if (ota_host_e    != "#") prefDatas.putString("OTA_host", ota_host_e);
      prefDatas.end();
      delay(800);
      ESP.restart();
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, ResetTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid  = root["deviceid"].isNull() ? "default_deviceid" : root["deviceid"].as<String>();
    String slaveid   = root["slaveid"].isNull() ? "#" : root["slaveid"].as<String>();
    String slavename = root["sensor_no"].isNull() ? "#" : root["sensor_no"].as<String>();
    Serial.println(deviceid);
    if (deviceid == DeviceId && slaveid == DeviceId) {
      reseting();
    } else if (deviceid == DeviceId && slaveid != "#" && slavename != "#") {
      reseting_slave(slaveid, slavename);
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, isAliveTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid = root["deviceid"].isNull() ? "default_deviceid" : root["deviceid"].as<String>();
    Serial.println(deviceid);
    if (deviceid == DeviceId) {
      Publish_Alive();
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, isHealthTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid = root["deviceid"].isNull() ? "default_deviceid" : root["deviceid"].as<String>();
    Serial.println(deviceid);
    if (deviceid == DeviceId) {
      Publish_Health();
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, SlaverequestTopic.c_str()) == 0) {
    // accept and store 'range' for the slot
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid = root["deviceid"].isNull() ? "default_deviceid" : root["deviceid"].as<String>();
    String sensor_no= root["sensor_no"].isNull() ? "#" : root["sensor_no"].as<String>();
    String slaveid  = root["slaveid"].isNull()   ? "#" : root["slaveid"].as<String>();
    int rangeVal    = root["range"].isNull()     ? 0   : root["range"].as<int>();
    Serial.println(deviceid);
    if (deviceid == DeviceId && sensor_no != "#" && slaveid != "#") {
      Publish_Slaveresponse(sensor_no, slaveid, rangeVal);
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, StatusrequestTopic.c_str()) == 0) {
    DynamicJsonDocument root(1024);
    deserializeJson(root, payload);
    String deviceid = root["deviceid"].isNull() ? "default_deviceid" : root["deviceid"].as<String>();
    String sensor_no= root["sensor_no"].isNull() ? "#" : root["sensor_no"].as<String>();
    Serial.println(deviceid);
    if (deviceid == DeviceId && sensor_no != "#") {
      Publish_Statusresponse(sensor_no);
    } else {
      Serial.println("Wrong deviceid ");
    }
  }
  else if (strcmp(topic, DeleteSlaveTopic.c_str()) == 0) {
    DynamicJsonDocument root(512);
    deserializeJson(root, payload);
    String deviceid = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
    String sensor_no= root["sensor_no"].isNull() ? "#" : root["sensor_no"].as<String>();
    String slaveid  = root["slaveid"].isNull()   ? "#" : root["slaveid"].as<String>();

    DynamicJsonDocument rep(256);
    rep["deviceid"] = DeviceId;

    if (deviceid != DeviceId) {
      rep["reply"] = "bad_request";
    } else {
      int idx = -1;
      if (sensor_no != "#") {
        idx = index_from_code(sensor_no);
        rep["sensor_no"] = sensor_no;
      } else if (slaveid != "#") {
        for (int i = 0; i < 20; i++) if (arr[i].device_id == slaveid) { idx = i; break; }
        rep["slaveid"] = slaveid;
      } else {
        rep["reply"] = "bad_request";
      }

      if (idx >= 0) {
        arr[idx] = salve();
        save_slave_entry_nvs(idx);
        rep["reply"] = "deleted";
        Serial.printf("[ARR] Deleted slot idx=%d\n", idx);
      } else if (!rep.containsKey("reply")) {
        rep["reply"] = "not_found";
      }
    }
    String payloadOut; serializeJson(rep, payloadOut);
    client.publish(SlaveTopic.c_str(), payloadOut.c_str());
  }
}

/*********************  Publish helpers  *********************/
void Publish_Statusresponse(String sen) {
  // Helper: should this entry be included based on 'sen'?
  auto includeByGroup = [&](const String& devName)->bool {
    if (sen.length() == 2) {             // "TM", "MC", "VC", "WC"
      return devName.startsWith(sen);
    }
    if (sen.equalsIgnoreCase("ALL")) {   // include all
      return true;
    }
    // Fallback: include all if unknown / not provided
    return true;
  };

  // Helper: compact 8 switch booleans into "01010101"
  auto swBits = [&](const salve& s)->String {
    String bits; bits.reserve(8);
    for (int b = 0; b < 8; b++) bits += (s.sw_no[b] ? '1' : '0');
    return bits;
  };

  // Build the flat status string
  String status; status.reserve(2048); // adjust if needed
  bool firstSeg = true;

  for (int i = 0; i < 20; i++) {
    const salve &s = arr[i];

    // consider it "non-empty" if any key field is present
    bool hasContent = (s.device_name.length() > 0) ||
                      (s.device_id.length()   > 0) ||
                      (s.address != 0) ||
                      (s.range   != 0) ||
                      (s.distance!= 0);
    if (!hasContent) continue;
    if (!includeByGroup(s.device_name)) continue;

    if (!firstSeg) status += '#';
    firstSeg = false;

    // One segment: name,id,mode,ch,ah,al,addr,range,distance,Ph,TDS,turb,swbits
    status += s.device_name;            status += '#';
    status += s.device_id;              status += '#';
    status += String(s.mode);           status += '#';
    status += String(s.channel);        status += '#';
    status += String(s.addr_high);      status += '#';
    status += String(s.addr_low);       status += '#';
    status += String(s.address);        status += '#';   // decimal; use String(s.address, HEX) if you prefer hex
    status += String(s.range);          status += '#';
    status += String(s.distance);       status += '#';
    status += String(s.Ph);             status += '#';
    status += String(s.TDS);            status += '#';
    status += String(s.turbulity);      status += '#';
    status += swBits(s);
  }

  // Create minimal payload: deviceid + status
  DynamicJsonDocument doc(512 + status.length());
  doc["deviceid"] = DeviceId;
  doc["status"]   = status;   // may be empty if no entries matched

  String payload; serializeJson(doc, payload);
  Serial.println(StatusTopic.c_str());
  Serial.println(payload);
  client.setBufferSize(2048);
  if (client.publish(StatusTopic.c_str(), payload.c_str())) {
    Serial.println("StatusTopic sent successfully.");
  } else {
    Serial.println("Failed to send StatusTopic.");
  }
}

// Publish_Slaveresponse — fixed address blocks per group.
// TM: 2..5  (arr 0..3)
// MC: 6..9  (arr 4..7)
// VC: 10..13 (arr 8..11)
// WC: 14..17 (arr 12..15)
// ANY (unknown prefix): 18..21 (arr 16..19)
void Publish_Slaveresponse(String sen, String id, int rangeVal) {
  DynamicJsonDocument doc(512);
  doc["deviceid"] = DeviceId;

  // Helper: map "TM2" -> (arrIdx, address_l)
  auto mapCode = [&](const String& code, int& arrIdxOut, uint8_t& addLOut)->bool {
    String pref; int n0;  // n0 is zero-based (1->0, 2->1, 3->2, 4->3)
    if (!parse_sensor_code(code, pref, n0)) return false;
    if (n0 < 0 || n0 > 3) return false;

    int baseArr  = -1;
    int baseAddr = -1;

    if      (pref == "TM") { baseArr =  0; baseAddr =  2; }   // 2..5
    else if (pref == "MC") { baseArr =  4; baseAddr =  6; }   // 6..9
    else if (pref == "VC") { baseArr =  8; baseAddr = 10; }   // 10..13
    else if (pref == "WC") { baseArr = 12; baseAddr = 14; }   // 14..17
    else { // ANYTHING ELSE
      baseArr  = 16;                                           // arr[16..19]
      baseAddr = 18;                                           // 18..21
    }

    arrIdxOut = baseArr + n0;
    addLOut   = (uint8_t)(baseAddr + n0);
    return true;
  };

  int arrIdx = -1;
  uint8_t add_l = 0;

  if (!mapCode(sen, arrIdx, add_l)) {
    Serial.printf("Publish_Slaveresponse: bad sensor code \"%s\"\n", sen.c_str());
    doc["reply"] = "bad_request";
  } else {
    // Fill reply
    doc["sensor_no"] = sen;
    doc["channel"]   = channel;
    doc["address_l"] = add_l;           // Fixed mapping per group
    doc["address_h"] = address_h;
    doc["slaveid"]   = id;
    if (rangeVal > 0) doc["range"] = rangeVal;   // Mostly meaningful for TM*

    // Update arr[] and persist this slot
    int newRange = (rangeVal > 0) ? rangeVal : arr[arrIdx].range;
    set_slave(
      arrIdx, sen, id,
      /*mode=*/1,            // LoRa
      channel,
      address_h,
      add_l,
      /*range*/     newRange,
      /*distance*/  arr[arrIdx].distance,
      /*Ph*/        arr[arrIdx].Ph,
      /*TDS*/       arr[arrIdx].TDS,
      /*turb*/      arr[arrIdx].turbulity,
      /*sw0*/       arr[arrIdx].sw_no[0]
    );
    debug_print_arr_entry(arrIdx, "Publish_Slaveresponse: after set_slave");

    doc["reply"] = "ok";
  }

  // Publish
  String payload; serializeJson(doc, payload);
  Serial.println(SlaveTopic.c_str());
  Serial.println(payload);

  // If payloads get larger elsewhere, ensure buffer: client.setBufferSize(1024);
  if (client.publish(SlaveTopic.c_str(), payload.c_str())) {
    Serial.println("SlaveTopic sent successfully:");
    Serial.println(payload);
  } else {
    Serial.println("Failed to send SlaveTopic.");
  }
}


void Publish_Health() {
  String payload = "{";
  payload += "\"deviceid\":\"" + DeviceId + "\",";
  payload += "\"heap\":\"" + String(ESP.getFreeHeap()) + "\",";
  payload += "\"rssi\":\"" + String(WiFi.RSSI()) + "\",";
  payload += "\"internet_speed\":\"" + String(ESP.getFlashChipSize() / (1024 * 1024)) + "\",";
  payload += "\"chip_model\":\"" + String(ESP.getChipModel()) + "\",";
  payload += "\"chip_revision\":\"" + String(ESP.getChipRevision()) + "\",";
  payload += "\"chip_core\":\"" + String(ESP.getChipCores()) + "\",";
  payload += "\"chip_frequency\":\"" + String(ESP.getCpuFreqMHz()) + "\"";
  payload += "}";
  char attributes[800];

  Serial.println(HealthTopic.c_str());
  payload.toCharArray(attributes, sizeof(attributes));
  Serial.println(attributes);
  if (client.publish(HealthTopic.c_str(), attributes)) {
    Serial.println("HealthTopic sent successfully:"); Serial.println(attributes);
  } else {
    Serial.println("Failed to send HealthTopic.");
  }
}
void Publish_Alive() {
  String payload = "{";
  payload += "\"deviceid\":\"" + DeviceId + "\",";
  payload += "\"thingId\":\"" + ThingId + "\",";
  payload += "\"ssid\":\"" + ssid + "\",";
  payload += "\"password\":\"" + password + "\",";
  payload += "\"ipaddress\":\"" + WiFi.localIP().toString() + "\",";
  payload += "\"macaddress\":\"" + macs + "\",";
  payload += "\"firmware_version\":\"" + FirmwareVer + "\"";
  payload += "}";
  char attributes[800];

  Serial.println(AliveTopic.c_str());
  payload.toCharArray(attributes, sizeof(attributes));
  Serial.println(attributes);
  if (client.publish(AliveTopic.c_str(), attributes)) {
    Serial.println("AliveTopic sent successfully:"); Serial.println(attributes);
  } else {
    Serial.println("Failed to send AliveTopic.");
  }
}
void Publish_Update(bool state, int number) {
  DynamicJsonDocument doc(1024);
  doc["deviceid"]  = DeviceId;
  doc["device"]    = "base";
  const char* label = (number >= 0 && number < pinLength) ? relays[number].c_str() : "UNK";
  doc["switch_no"] = label;
  doc["status"]    = state;

  String payload; serializeJson(doc, payload);
  Serial.println(UpdateTopic.c_str());
  Serial.println(payload);
  if (client.publish(UpdateTopic.c_str(), payload.c_str())) {
    Serial.println("UpdateTopic sent successfully:"); Serial.println(payload);
  } else {
    Serial.println("Failed to send UpdateTopic.");
  }
}
void Publish_Updates(int percent, String number) {
  DynamicJsonDocument doc(1024);
  doc["deviceid"]  = DeviceId;
  doc["device"]    = "slave";
  doc["sensor_no"] = number;
  doc["value"]     = percent;  // 0..100%

  String payload; serializeJson(doc, payload);
  Serial.println(UpdateTopic.c_str());
  Serial.println(payload);
  if (client.publish(UpdateTopic.c_str(), payload.c_str())) {
    Serial.println("UpdateTopic sent successfully:"); Serial.println(payload);
  } else {
    Serial.println("Failed to send UpdateTopic.");
  }
}
void Publish_OTA_Valid(String transaction, String secret, String versions, String reply) {
  client.loop();
  String payload = "{";
  payload += "\"deviceid\":\"" + DeviceId + "\",";
  payload += "\"transactionId\":\"" + transaction + "\",";
  payload += "\"secrets\":\"" + secret + "\",";
  payload += "\"firmware_version\":\"" + versions + "\",";
  payload += "\"reply\":\"" + reply + "\"";
  payload += "}";
  char attributes[1000];
  payload.toCharArray(attributes, sizeof(attributes));

  if (client.publish(OtavalidateTopic.c_str(), attributes)) {
    Serial.println("OtavalidateTopic sent successfully:"); Serial.println(attributes);
  } else {
    Serial.println("Failed to send OtavalidateTopic.");
  }
}
void Publish_OTA_Valids(String thingId, String OTA, String versions, String reply) {
  client.loop();
  String payload = "{";
  payload += "\"deviceid\":\"" + DeviceId + "\",";
  payload += "\"thingId\":\"" + ThingId + "\",";
  payload += "\"url\":\"" + OTA + "\",";
  payload += "\"firmware_version\":\"" + versions + "\",";
  payload += "\"reply\":\"" + reply + "\"";
  payload += "}";
  char attributes[1000];
  payload.toCharArray(attributes, sizeof(attributes));

  if (client.publish(OtavalidateTopic.c_str(), attributes)) {
    Serial.println("OtavalidateTopic sent successfully:"); Serial.println(attributes);
  } else {
    Serial.println("Failed to send OtavalidateTopic.");
  }
}

/*********************  AWS connect / WiFi  *********************/
void connectAWS() {
  net.setCACert(rootCACertificate.c_str());
  net.setCertificate(clientCertificate.c_str());
  net.setPrivateKey(clientPrivateKey.c_str());
  client.setServer(Mqtt_server.c_str(), 8883);
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IoT Core...");
  while (!client.connected()) {
    if (client.connect(ThingId.c_str())) {
      Serial.println("Connected to AWS IoT Core");
      client.subscribe(ConfigTopic.c_str());
      client.subscribe(SettingTopic.c_str());
      client.subscribe(ControlTopic.c_str());
      client.subscribe(OtaintializeTopic.c_str());
      client.subscribe(OtarequestTopic.c_str());
      client.subscribe(isAliveTopic.c_str());
      client.subscribe(isHealthTopic.c_str());
      client.subscribe(ResetTopic.c_str());
      client.subscribe(SlaverequestTopic.c_str());
      client.subscribe(StatusrequestTopic.c_str());
      client.subscribe(DeleteSlaveTopic.c_str());
    } else {
      Serial.print("AWS IoT Core connection failed, status: ");
      Serial.println(client.state());
      delay(3000);
      net_aws = net_aws + 1;
    }
    if (net_aws >= 2) { net_aws = 0; break; }
  }
}
void reconnect() {
  if (modes == 1){
    wifiSetup();
  } 
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connecting to ThingsBoard node ...");
    Serial.print("Attempting MQTT connection...");
    connectAWS();
    prefDatas.begin("datas", false);
    randomSeed(micros());
    prefDatas.putBool("state", 1);
    states = prefDatas.getBool("state", states);
    prefDatas.end();
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    Serial.println("Wifi channel is"); Serial.println(WiFi.channel());
    Serial.println("Ready");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("Netmask: ");   Serial.println(WiFi.subnetMask());
    Serial.print("Gateway: ");   Serial.println(WiFi.gatewayIP());
    macs = WiFi.macAddress();
    Serial.println(macs);
    Serial.println("Wifi mode on");
    Serial.print("Configure status="); Serial.println(states);
    Serial.print("Device mode=");      Serial.println(modes);
    Serial.print("Client mode=");      Serial.println(client.connected());
  } else {
    Serial.println("Wifi mode off");
    if (states == 0) {
      prefDatas.begin("datas", false);
      prefDatas.putInt("mode", 2);
      modes = prefDatas.getInt("mode", modes);
      prefDatas.end();
      Serial.print("Bluetooth mode on");
      blue_set();
    } else {
      prefDatas.begin("datas", false);
      prefDatas.putInt("mode", 1);
      modes = prefDatas.getInt("mode", modes);
      prefDatas.end();
      Serial.print("No Bluetooth mode opt");
    }
  }
}
void wifiSetup() {
  WiFi.mode(WIFI_AP_STA);
  Serial.printf("[WIFI] Connecting to %s ", ssid.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.println("Connecting for WIFI");
  while (WiFi.status() != WL_CONNECTED ) {
    Serial.print(".");
    delay(100);
    net_w = net_w + 10;
    if (net_w >= (1000 - (750 * states))) { net_w = 0; break; }
  }
}

/*********************  Main loop  *********************/
void loop() {
  readInputs();
  handleInputChanges();

  // All evaluation & LEDs happen when LoRa data arrives:
  Lora_receive();

  if (modes == 1) {
    client.loop();
    delay(30);

    if (!client.connected() && millis() - reconnect_time > 20000) {
      Serial.println("wifi disconnected");
      Serial.print("Mqtt connection state "); Serial.println(client.state());
      reconnect_time = millis();
      reconnect();
    }
    if (initial == 1) {
      delay(600);
      Publish_Alive();
      delay(600);
      Publish_Health();
      initial = 0;
    }
  }
  else if (modes == 2) {
    if (valor != "") {
      Serial.println(" BLE Data received");
      DynamicJsonDocument root(1024);
      deserializeJson(root, valor);
      String id = root["deviceid"].isNull() ? "#" : root["deviceid"].as<String>();
      int command = root["mode"].isNull() ? 2 : root["mode"].as<int>();
      Serial.print("Device id ="); Serial.println(id);
      Serial.print("BLE command ="); Serial.println(command);
      if ( command == 1 && DeviceId == id) {
        ssid     = root["ssid"].isNull() ? "#" : root["ssid"].as<String>();
        password = root["password"].isNull() ? "#" : root["password"].as<String>();
        Serial.println(" Shift to wifi connect");
        prefDatas.begin("datas", false);
        if (ssid != "#")     prefDatas.putString("ssid", ssid);
        if (password != "#") prefDatas.putString("password", password);
        prefDatas.putInt("mode", 1);
        prefDatas.putBool("state", 0);
        prefDatas.end();
        delay(200);
        ESP.restart();
      }
      else if (command == 2 && DeviceId == id) {
        ssid     = root["ssid"].isNull() ? "#" : root["ssid"].as<String>();
        password = root["password"].isNull() ? "#" : root["password"].as<String>();
        Serial.println(" Shift to BLE receive");
        prefDatas.begin("datas", false);
        if (ssid != "#")     prefDatas.putString("ssid", ssid);
        if (password != "#") prefDatas.putString("password", password);
        prefDatas.putInt("mode", command);
        prefDatas.putBool("state", 1);
        prefDatas.end();
        modes = command;
        delay(100);
      }
      else if (command == 3 && DeviceId == id) {
        modes = root["mode"].isNull() ? 1 : root["mode"].as<int>();
        Serial.println(" Shift to Espnow connect");
        prefDatas.begin("datas", false);
        prefDatas.putInt("mode", command);
        prefDatas.putBool("state", 1);
        prefDatas.end();
        delay(100);
      }
      else {
        Serial.println("Wrong data received");
      }
      net_b = 0;
      valor = "";
    }
    else {
      delay(100);
      net_b = net_b + 10;
      if (net_b >= 1000 ) {
        prefDatas.begin("datas", false);
        net_b = 0;
        prefDatas.putInt("mode", 1);
        prefDatas.end();
      }
    }
  }
  else if (modes == 3) {
    // reserved
  }
}

/********** Debug-only helpers (definitions) **********/
void debug_print_arr_entry(int idx, const char* where) {
  if (idx < 0 || idx >= 20) return;
  Serial.printf("[ARR][%s] idx=%d name=%s id=%s mode=%d ch=%u ah=%u al=%u addr=0x%04X range=%d dist=%d pH=%d TDS=%d Turb=%d SW[0..7]={",
                where, idx,
                arr[idx].device_name.c_str(), arr[idx].device_id.c_str(),
                arr[idx].mode, arr[idx].channel,
                arr[idx].addr_high, arr[idx].addr_low, arr[idx].address,
                arr[idx].range, arr[idx].distance, arr[idx].Ph, arr[idx].TDS, arr[idx].turbulity);
  for (int b=0;b<8;b++) { Serial.print(arr[idx].sw_no[b] ? "1" : "0"); if (b<7) Serial.print(","); }
  Serial.println("}");
}
void debug_dump_arr_table(const char* where) {
  Serial.printf("========== ARR DUMP (%s) ==========\n", where);
  for (int i=0;i<20;i++) {
    if (arr[i].device_name.length() > 0 || arr[i].device_id.length() > 0 || arr[i].address != 0 || arr[i].range != 0) {
      debug_print_arr_entry(i, "dump");
    }
  }
  Serial.println("===================================");
}
void debug_print_primary1(const char* tag) {
  Serial.printf("[P1][%s] slave=%s param=%s max=%d min=%d A1=%s A2=%s A3=%s\n",
                tag, gPrimary1.slave.c_str(), gPrimary1.parameter.c_str(),
                gPrimary1.data1, gPrimary1.data2,
                gPrimary1.A1.c_str(), gPrimary1.A2.c_str(), gPrimary1.A3.c_str());
}
void debug_print_primary2(const char* tag) {
  Serial.printf("[P2][%s] slave=%s param=%s max=%d min=%d A1=%s A2=%s A3=%s\n",
                tag, gPrimary2.slave.c_str(), gPrimary2.parameter.c_str(),
                gPrimary2.data1, gPrimary2.data2,
                gPrimary2.A1.c_str(), gPrimary2.A2.c_str(), gPrimary2.A3.c_str());
}
void debug_print_parsed_action(const char* raw, const String& devCode, int sNo, bool on, bool ok) {
  Serial.printf("[ParseAction] raw=\"%s\" -> dev=%s S%d state=%s valid=%d\n",
                raw, devCode.c_str(), sNo, on ? "ON" : "OFF", ok);
}
