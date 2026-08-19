/*
  ESP32 <-> LEGO SPIKE Prime Smart Delivery Bridge

  What it does
  1) Scans for the official SPIKE Prime App 3 BLE service (FD02)
  2) Performs InfoRequest handshake
  3) Uploads an embedded SPIKE Python program to slot 0
  4) Enables device notifications
  5) Starts a Wi-Fi AP and browser dashboard
  6) Lets you start/stop the SPIKE program and view battery/distance telemetry

  Target environment
  - ESP32 / ESP32-S3 with BLE + Wi-Fi
  - Arduino IDE 2.x
  - NimBLE-Arduino 2.x

  IMPORTANT
  - This is an educational reference implementation derived from LEGO's published
    SPIKE Prime protocol. It has not been physically hardware-tested in this report.
  - Close the SPIKE App before running this bridge so the Hub is available to ESP32.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#include <vector>

static const char* SERVICE_UUID = "0000fd02-0000-1000-8000-00805f9b34fb";
static const char* RX_UUID      = "0000fd02-0001-1000-8000-00805f9b34fb"; // Hub receives
static const char* TX_UUID      = "0000fd02-0002-1000-8000-00805f9b34fb"; // Hub transmits

static const uint8_t PROGRAM_SLOT = 0;
static const uint16_t NOTIFY_INTERVAL_MS = 500;

// Wi-Fi AP created by the ESP32
static const char* AP_SSID = "SPIKE-ESP32";
static const char* AP_PASS = "spikeprime";

WebServer server(80);
NimBLEClient* spikeClient = nullptr;
NimBLERemoteCharacteristic* spikeRx = nullptr;
NimBLERemoteCharacteristic* spikeTx = nullptr;

volatile bool spikeConnected = false;
volatile int lastDistanceMm = -1;
volatile int batteryPercent = -1;
volatile uint8_t lastResponseType = 0xFF;
volatile uint8_t lastResponseStatus = 0xFF;
volatile bool responseReady = false;
uint16_t maxPacketSize = 20;
uint16_t maxChunkSize = 128;

std::vector<uint8_t> lowIncoming;
std::vector<uint8_t> highIncoming;
bool highPriorityActive = false;

// Same program as spike_delivery.py, embedded so the ESP32 can provision the Hub.
static const char SPIKE_PROGRAM[] = R"PY(import runloop
import motor_pair
import distance_sensor
from hub import port, light_matrix

PAIR = motor_pair.PAIR_1
STOP_DISTANCE_MM = 180

async def main():
    motor_pair.pair(PAIR, port.A, port.B)
    light_matrix.show_image(light_matrix.IMAGE_HAPPY)
    while True:
        d = distance_sensor.distance(port.C)
        if d == -1 or d > STOP_DISTANCE_MM:
            motor_pair.move(PAIR, 0, velocity=350)
            await runloop.sleep_ms(50)
        else:
            motor_pair.stop(PAIR)
            light_matrix.show_image(light_matrix.IMAGE_NO)
            await runloop.sleep_ms(200)
            await motor_pair.move_for_degrees(PAIR, -180, 0, velocity=220)
            await motor_pair.move_for_degrees(PAIR, 330, 100, velocity=260)
            light_matrix.show_image(light_matrix.IMAGE_HAPPY)
            await runloop.sleep_ms(100)

runloop.run(main())
)PY";

// ---------- SPIKE COBS variant ----------
static std::vector<uint8_t> spikeCobsEncode(const uint8_t* data, size_t len) {
  const uint8_t MAX_BLOCK = 84;
  const uint8_t OFFSET = 2;
  std::vector<uint8_t> out;
  size_t codeIndex = 0;
  uint16_t block = 0;

  auto beginBlock = [&]() {
    codeIndex = out.size();
    out.push_back(0xFF);
    block = 1;
  };

  beginBlock();
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    if (b > 0x02) {
      out.push_back(b);
      block++;
    }

    if (b <= 0x02 || block > MAX_BLOCK) {
      if (b <= 0x02) {
        uint16_t delimiterBase = b * MAX_BLOCK;
        uint16_t blockOffset = block + OFFSET;
        out[codeIndex] = (uint8_t)(delimiterBase + blockOffset);
      }
      beginBlock();
    }
  }
  out[codeIndex] = (uint8_t)(block + OFFSET);
  return out;
}

static bool spikeCobsDecode(const std::vector<uint8_t>& data, std::vector<uint8_t>& out) {
  const int MAX_BLOCK = 84;
  const int OFFSET = 2;
  if (data.empty()) return false;
  out.clear();

  auto unescape = [&](uint8_t code, int& value, int& block) -> bool {
    if (code == 0xFF) {
      value = -1;
      block = MAX_BLOCK + 1;
      return true;
    }
    if (code <= 0x02) return false;
    int n = code - OFFSET;
    value = n / MAX_BLOCK;
    block = n % MAX_BLOCK;
    if (block == 0) {
      block = MAX_BLOCK;
      value -= 1;
    }
    return value >= 0 && value <= 2;
  };

  int value, block;
  if (!unescape(data[0], value, block)) return false;

  for (size_t i = 1; i < data.size(); ++i) {
    block--;
    if (block > 0) {
      out.push_back(data[i]);
      continue;
    }
    if (value >= 0) out.push_back((uint8_t)value);
    if (!unescape(data[i], value, block)) return false;
  }
  return true;
}

static std::vector<uint8_t> packFrame(const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame = spikeCobsEncode(payload.data(), payload.size());
  for (auto& b : frame) b ^= 0x03;
  frame.push_back(0x02); // end delimiter
  return frame;
}

static bool unpackFrame(std::vector<uint8_t> frame, std::vector<uint8_t>& payload) {
  if (frame.empty()) return false;
  if (frame.back() == 0x02) frame.pop_back();
  if (!frame.empty() && frame.front() == 0x01) frame.erase(frame.begin());
  if (frame.empty()) return false;
  for (auto& b : frame) b ^= 0x03;
  return spikeCobsDecode(frame, payload);
}

// zlib/binascii-compatible CRC32, with LEGO's 4-byte alignment rule.
static uint32_t crc32Aligned(const uint8_t* data, size_t len, uint32_t seed = 0) {
  uint32_t crc = seed ^ 0xFFFFFFFFu;
  size_t aligned = (len + 3u) & ~3u;
  for (size_t i = 0; i < aligned; ++i) {
    uint8_t b = (i < len) ? data[i] : 0x00;
    crc ^= b;
    for (int k = 0; k < 8; ++k) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void writeLE16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back((uint8_t)(x & 0xFF));
  v.push_back((uint8_t)((x >> 8) & 0xFF));
}
static void writeLE32(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back((uint8_t)(x & 0xFF));
  v.push_back((uint8_t)((x >> 8) & 0xFF));
  v.push_back((uint8_t)((x >> 16) & 0xFF));
  v.push_back((uint8_t)((x >> 24) & 0xFF));
}
static uint16_t readLE16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static int16_t readLE16s(const uint8_t* p) {
  return (int16_t)readLE16(p);
}

static bool sendPayload(const std::vector<uint8_t>& payload) {
  if (!spikeConnected || !spikeRx) return false;
  auto frame = packFrame(payload);
  size_t packetSize = maxPacketSize ? maxPacketSize : frame.size();
  for (size_t i = 0; i < frame.size(); i += packetSize) {
    size_t n = min(packetSize, frame.size() - i);
    if (!spikeRx->writeValue(frame.data() + i, n, false)) return false;
    delay(1);
  }
  return true;
}

static bool requestAndWait(const std::vector<uint8_t>& payload, uint8_t expectedResponse, uint32_t timeoutMs = 1500) {
  responseReady = false;
  lastResponseType = 0xFF;
  lastResponseStatus = 0xFF;
  if (!sendPayload(payload)) return false;

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (responseReady && lastResponseType == expectedResponse) {
      return lastResponseStatus == 0x00;
    }
    delay(5);
  }
  return false;
}

static void parseDeviceNotification(const std::vector<uint8_t>& p) {
  if (p.size() < 3 || p[0] != 0x3C) return;
  uint16_t payloadSize = readLE16(&p[1]);
  size_t end = min((size_t)3 + payloadSize, p.size());
  size_t i = 3;

  while (i < end) {
    uint8_t type = p[i];
    switch (type) {
      case 0x00:
        if (i + 2 > end) return;
        batteryPercent = p[i + 1];
        i += 2;
        break;
      case 0x01:
        if (i + 21 > end) return;
        i += 21;
        break;
      case 0x02:
        if (i + 26 > end) return;
        i += 26;
        break;
      case 0x0A:
        if (i + 12 > end) return;
        i += 12;
        break;
      case 0x0B:
        if (i + 4 > end) return;
        i += 4;
        break;
      case 0x0C:
        if (i + 9 > end) return;
        i += 9;
        break;
      case 0x0D:
        if (i + 4 > end) return;
        lastDistanceMm = readLE16s(&p[i + 2]);
        i += 4;
        break;
      case 0x0E:
        if (i + 11 > end) return;
        i += 11;
        break;
      default:
        return;
    }
  }
}

static void processPayload(const std::vector<uint8_t>& p) {
  if (p.empty()) return;
  uint8_t type = p[0];

  if (type == 0x01 && p.size() >= 17) {
    maxPacketSize = readLE16(&p[9]);
    maxChunkSize = readLE16(&p[13]);
    if (maxPacketSize < 8 || maxPacketSize > 1024) maxPacketSize = 20;
    if (maxChunkSize < 16 || maxChunkSize > 1024) maxChunkSize = 128;
    lastResponseType = type;
    lastResponseStatus = 0x00;
    responseReady = true;
    return;
  }

  if (type == 0x3C) {
    parseDeviceNotification(p);
    return;
  }

  if ((type == 0x0D || type == 0x11 || type == 0x1F || type == 0x29 || type == 0x47) && p.size() >= 2) {
    lastResponseType = type;
    lastResponseStatus = p[1];
    responseReady = true;
  }
}

static void processFrameBuffer(std::vector<uint8_t>& frame) {
  if (frame.empty()) return;
  std::vector<uint8_t> payload;
  if (unpackFrame(frame, payload)) processPayload(payload);
  frame.clear();
}

static void notifyCallback(NimBLERemoteCharacteristic*, uint8_t* data, size_t length, bool) {
  for (size_t i = 0; i < length; ++i) {
    uint8_t b = data[i];

    if (highPriorityActive) {
      if (b == 0x01) {
        highIncoming.clear();
        highIncoming.push_back(0x01);
      } else if (b == 0x02) {
        highIncoming.push_back(0x02);
        processFrameBuffer(highIncoming);
        highPriorityActive = false;
      } else {
        highIncoming.push_back(b);
      }
      continue;
    }

    if (b == 0x01) {
      highPriorityActive = true;
      highIncoming.clear();
      highIncoming.push_back(0x01);
    } else if (b == 0x02) {
      if (!lowIncoming.empty()) {
        lowIncoming.push_back(0x02);
        processFrameBuffer(lowIncoming);
      }
    } else {
      lowIncoming.push_back(b);
    }
  }
}

static bool handshake() {
  std::vector<uint8_t> info = {0x00};
  return requestAndWait(info, 0x01, 2000);
}

static bool enableNotifications() {
  std::vector<uint8_t> req = {0x28};
  writeLE16(req, NOTIFY_INTERVAL_MS);
  return requestAndWait(req, 0x29, 1500);
}

static bool clearSlot(uint8_t slot) {
  std::vector<uint8_t> req = {0x46, slot};
  requestAndWait(req, 0x47, 1000);
  return true;
}

static bool uploadProgram() {
  const uint8_t* bytes = (const uint8_t*)SPIKE_PROGRAM;
  size_t programLen = strlen(SPIKE_PROGRAM);
  uint32_t programCrc = crc32Aligned(bytes, programLen, 0);

  clearSlot(PROGRAM_SLOT);

  std::vector<uint8_t> start = {0x0C};
  const char* filename = "program.py";
  for (size_t i = 0; i < strlen(filename); ++i) start.push_back((uint8_t)filename[i]);
  start.push_back(0x00);
  start.push_back(PROGRAM_SLOT);
  writeLE32(start, programCrc);
  if (!requestAndWait(start, 0x0D, 2000)) return false;

  uint32_t runningCrc = 0;
  size_t offset = 0;
  size_t chunk = maxChunkSize;
  if (chunk < 16) chunk = 16;

  while (offset < programLen) {
    size_t n = min(chunk, programLen - offset);
    runningCrc = crc32Aligned(bytes + offset, n, runningCrc);

    std::vector<uint8_t> req = {0x10};
    writeLE32(req, runningCrc);
    writeLE16(req, (uint16_t)n);
    req.insert(req.end(), bytes + offset, bytes + offset + n);

    if (!requestAndWait(req, 0x11, 2000)) return false;
    offset += n;
  }
  return true;
}

static bool programFlow(bool stop) {
  std::vector<uint8_t> req = {0x1E, (uint8_t)(stop ? 0x01 : 0x00), PROGRAM_SLOT};
  return requestAndWait(req, 0x1F, 1500);
}

static bool connectSpike() {
  Serial.println("Scanning for SPIKE Prime...");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  NimBLEScanResults results = scan->getResults(6000, false);

  const NimBLEAdvertisedDevice* target = nullptr;
  NimBLEUUID serviceUuid(SERVICE_UUID);
  for (int i = 0; i < results.getCount(); ++i) {
    const NimBLEAdvertisedDevice* d = results.getDevice(i);
    if (d && d->isAdvertisingService(serviceUuid)) {
      target = d;
      break;
    }
  }
  if (!target) {
    Serial.println("No SPIKE Prime advertising FD02 found.");
    return false;
  }

  spikeClient = NimBLEDevice::createClient();
  if (!spikeClient->connect(target)) {
    Serial.println("BLE connect failed.");
    return false;
  }

  NimBLERemoteService* svc = spikeClient->getService(SERVICE_UUID);
  if (!svc) return false;
  spikeRx = svc->getCharacteristic(RX_UUID);
  spikeTx = svc->getCharacteristic(TX_UUID);
  if (!spikeRx || !spikeTx) return false;

  if (!spikeTx->canNotify() || !spikeTx->subscribe(true, notifyCallback)) {
    Serial.println("TX notify subscribe failed.");
    return false;
  }

  spikeConnected = true;
  Serial.println("SPIKE connected.");
  return true;
}

static String htmlPage() {
  String status = spikeConnected ? "CONNECTED" : "DISCONNECTED";
  String page = R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
  <title>SPIKE Mission Control</title><style>
  body{font-family:Arial,sans-serif;background:#f5f7fb;color:#17202a;margin:0;padding:24px}.card{max-width:620px;margin:auto;background:white;border-radius:24px;padding:24px;box-shadow:0 12px 36px #0001}.badge{display:inline-block;padding:7px 10px;border-radius:999px;background:#e8f3ff;color:#1b64da;font-weight:700}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:20px 0}.metric{background:#f7f8fa;border-radius:18px;padding:18px}.metric b{display:block;font-size:28px;margin-top:6px}button{border:0;border-radius:14px;padding:14px 20px;font-size:16px;font-weight:700;margin-right:8px;cursor:pointer}.go{background:#3182f6;color:white}.stop{background:#191f28;color:white}small{color:#6b7684} @media(max-width:520px){.grid{grid-template-columns:1fr}}
  </style><meta http-equiv="refresh" content="2"></head><body><div class="card"><span class="badge">)HTML";
  page += status;
  page += R"HTML(</span><h1>SPIKE Prime × ESP32</h1><p>Smart Delivery Rover mission control</p><div class="grid"><div class="metric">Distance<b>)HTML";
  page += String(lastDistanceMm);
  page += R"HTML( mm</b></div><div class="metric">Hub battery<b>)HTML";
  page += String(batteryPercent);
  page += R"HTML( %</b></div></div><p><a href="/start"><button class="go">START</button></a><a href="/stop"><button class="stop">STOP</button></a></p><small>Open 192.168.4.1 while connected to the SPIKE-ESP32 Wi-Fi network.</small></div></body></html>)HTML";
  return page;
}

static void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", [](){ server.send(200, "text/html", htmlPage()); });
  server.on("/start", [](){
    bool ok = programFlow(false);
    server.sendHeader("Location", "/");
    server.send(ok ? 302 : 500, "text/plain", ok ? "Starting" : "Start failed");
  });
  server.on("/stop", [](){
    bool ok = programFlow(true);
    server.sendHeader("Location", "/");
    server.send(ok ? 302 : 500, "text/plain", ok ? "Stopping" : "Stop failed");
  });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(800);
  NimBLEDevice::init("ESP32-SPIKE-BRIDGE");

  setupWeb();

  if (!connectSpike()) return;
  if (!handshake()) {
    Serial.println("Handshake failed.");
    return;
  }
  Serial.printf("Handshake OK. maxPacketSize=%u, maxChunkSize=%u\n", maxPacketSize, maxChunkSize);

  Serial.println("Uploading robot program to slot 0...");
  if (!uploadProgram()) {
    Serial.println("Program upload failed. See troubleshooting notes in the project guide.");
    return;
  }
  Serial.println("Program uploaded.");

  if (!enableNotifications()) Serial.println("Device notification request failed.");
  Serial.println("Use the web dashboard START button.");
}

void loop() {
  server.handleClient();

  if (spikeClient && !spikeClient->isConnected()) {
    spikeConnected = false;
  }
  delay(2);
}
