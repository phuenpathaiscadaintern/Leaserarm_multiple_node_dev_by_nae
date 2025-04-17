#include <ArduinoBLE.h>

// UUIDs ตามฝั่ง Slave
#define SERVICE_UUID              "87E01439-99BE-45AA-9410-DB4D3F23EA99"
#define SOUND_THRESHOLD_UUID      "2C4959B0-010A-4F00-B229-F36AD1500CE8"
#define START_CHARACTERISTIC_UUID "E5F6A1B2-C3D4-5678-9012-3456789ABCDE"
#define SOUND_LEVEL_UUID          "EA727F6D-1B2F-4613-BADF-DDF2C01659EC"
#define COUNTER_ALL_UUID          "134D9AED-7952-42D0-B8FC-96E59AADA8AD"
#define COUNTER_ACC_UUID          "2649093C-943C-4F6E-9585-5A445C70DB8F"
#define DEVICE_ID_UUID            "26B4FBEA-53D6-4D84-B5BC-21A211F0B317"
#define BATTERY_UUID              "DA313D4A-BCAD-4F4E-99A1-1004A5412461"
#define DEVICE_STATUS_UUID        "D90A7C02-9B21-4243-8372-3E523FA7978B"

#define MAX_SLAVES 10

struct SlaveNode {
  String name;
  BLEDevice device;
  BLECharacteristic startChar;
  BLECharacteristic thresholdChar;
  BLECharacteristic soundLevelChar;
  BLECharacteristic counterAllChar;
  BLECharacteristic counterAccChar;
  BLECharacteristic deviceIdChar;
  BLECharacteristic batteryChar;
  BLECharacteristic statusChar;
};

SlaveNode connectedSlaves[MAX_SLAVES];
bool thresholdsReceived[MAX_SLAVES] = {false};
int numSlaves = 0;
bool started = false;

bool isSlaveNode(String name) {
  return name.startsWith("ArLaserTS") || name.indexOf("ArLaserTS") >= 0;
}

void scanAndConnectSlaves() {
  BLE.scan();
  Serial.println("🔍 Scanning for slave-like devices...");

  unsigned long startTime = millis();
  while (millis() - startTime < 15000 && numSlaves < MAX_SLAVES) {
    BLEDevice dev = BLE.available();
    if (dev) {
      String devName = dev.localName();
      Serial.print("📡 Found: ");
      Serial.println(devName);

      if (isSlaveNode(devName)) {
        BLE.stopScan();
        delay(300); // ⏱️ ให้เวลาฝั่ง Slave เตรียมตัวก่อน connect

        Serial.println("🔗 Connecting to: " + devName);
        if (dev.connect()) {
          Serial.println("✅ Connected to: " + devName);

          if (!dev.discoverAttributes()) {
            Serial.println("❌ Discover attributes failed.");
            dev.disconnect();
            BLE.scan();
            continue;
          }

          BLEService service = dev.service(SERVICE_UUID);
          if (!service) {
            Serial.println("⚠️ Service not found.");
            dev.disconnect();
            BLE.scan();
            continue;
          }

          BLECharacteristic thresholdChar = service.characteristic(SOUND_THRESHOLD_UUID);
          BLECharacteristic startChar = service.characteristic(START_CHARACTERISTIC_UUID);
          BLECharacteristic soundLevelChar = service.characteristic(SOUND_LEVEL_UUID);
          BLECharacteristic counterAllChar = service.characteristic(COUNTER_ALL_UUID);
          BLECharacteristic counterAccChar = service.characteristic(COUNTER_ACC_UUID);
          BLECharacteristic deviceIdChar = service.characteristic(DEVICE_ID_UUID);
          BLECharacteristic batteryChar = service.characteristic(BATTERY_UUID);
          BLECharacteristic statusChar = service.characteristic(DEVICE_STATUS_UUID);

          if (thresholdChar && startChar && soundLevelChar &&
              counterAllChar && counterAccChar &&
              deviceIdChar && batteryChar && statusChar) {

            connectedSlaves[numSlaves++] = {
              devName, dev, startChar, thresholdChar, soundLevelChar,
              counterAllChar, counterAccChar, deviceIdChar, batteryChar, statusChar
            };
            Serial.println("✅ Added slave: " + devName);
          } else {
            Serial.println("❌ Missing characteristics. Disconnecting.");
            dev.disconnect();
          }
        } else {
          Serial.println("❌ Failed to connect to: " + devName + ". Retrying...");
        }

        BLE.scan();
      }
    }
    delay(200);
  }

  BLE.stopScan();
}

void sendStartSignal() {
  Serial.println("🚦 Sending START signal to all slaves...");
  for (int i = 0; i < numSlaves; i++) {
    connectedSlaves[i].startChar.writeValue(1);
    Serial.print("➡️ START sent to ");
    Serial.println(connectedSlaves[i].name);
  }
  started = true;
}

void pollSlaves() {
  for (int i = 0; i < numSlaves; i++) {
    SlaveNode& slave = connectedSlaves[i];

    if (!slave.device.connected()) {
      Serial.println("🔌 Disconnected: " + slave.name);
      continue;
    }

    int16_t soundLevel = 0;
    uint32_t counterAll = 0, counterAcc = 0;
    byte deviceId = 0, battery = 255;
    int16_t status = -1;

    bool ok1 = slave.soundLevelChar.readValue((byte*)&soundLevel, sizeof(soundLevel));
    bool ok2 = slave.counterAllChar.readValue((byte*)&counterAll, sizeof(counterAll));
    bool ok3 = slave.counterAccChar.readValue((byte*)&counterAcc, sizeof(counterAcc));
    bool ok4 = slave.deviceIdChar.readValue(&deviceId, 1);
    bool ok5 = slave.batteryChar.readValue(&battery, 1);
    bool ok6 = slave.statusChar.readValue((byte*)&status, sizeof(status));

    if (!(ok1 && ok2 && ok3 && ok4 && ok5 && ok6)) {
      Serial.println("⚠️ Failed to read some characteristics from " + slave.name);
      continue;
    }

    String statusStr = "❓ Unknown";
    switch (status) {
      case 0: statusStr = "🎯 Aiming"; break;
      case 1: statusStr = "😴 Standby"; break;
      case 2: statusStr = "🕰️ Idle"; break;
      case 3: statusStr = "💤 Sleep"; break;
      case -1: statusStr = "❗ Error"; break;
    }

    Serial.print("\n📡 ["); Serial.print(slave.name); Serial.println("]");
    Serial.print("   🔊 Sound Level : "); Serial.println(soundLevel);
    Serial.print("   🔢 Counter All : "); Serial.println(counterAll);
    Serial.print("   🔄 Counter Acc : "); Serial.println(counterAcc);
    Serial.print("   🆔 Device ID   : "); Serial.println(deviceId);
    Serial.print("   🔋 Battery     : ");
    if (battery == 0) Serial.println("🔌 Charging");
    else if (battery == 1) Serial.println("✅ Full / Not Charging");
    else Serial.println("❓ Unknown");
    Serial.print("   📶 Status      : "); Serial.println(statusStr);
    Serial.println("-------------------------------");
  }
}

void setup() {
  Serial.begin(115200);
  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
    while (1);
  }

  delay(3000); // ให้ Slave เริ่มโฆษณาทัน
  scanAndConnectSlaves();

  Serial.println("\n📨 พิมพ์แบบนี้เพื่อเซ็ต threshold:");
  Serial.println("SLAVE-001:120 หรือ ArLaserTS001:150");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.indexOf(':') > 0) {
      String name = input.substring(0, input.indexOf(':'));
      int value = input.substring(input.indexOf(':') + 1).toInt();

      for (int i = 0; i < numSlaves; i++) {
        if (connectedSlaves[i].name.startsWith(name)) {
          connectedSlaves[i].thresholdChar.writeValue(value);
          thresholdsReceived[i] = true;
          Serial.println("✅ Threshold set for " + connectedSlaves[i].name);
        }
      }

      bool allSet = true;
      for (int i = 0; i < numSlaves; i++) {
        if (!thresholdsReceived[i]) {
          allSet = false;
          Serial.println("⏳ Waiting for threshold on " + connectedSlaves[i].name);
        }
      }

      if (allSet && !started) {
        sendStartSignal();
      }
    } else {
      Serial.println("❌ Format: SLAVE-###:value หรือ ArLaserTS###:value");
    }
  }

  if (started) {
    pollSlaves();
  }

  delay(500);
}
