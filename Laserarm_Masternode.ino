#include <ArduinoBLE.h>

// UUIDs จากฝั่ง Slave (ใช้ตัวพิมพ์ใหญ่ทั้งหมด)
#define SERVICE_UUID              "87E01439-99BE-45AA-9410-DB4D3F23EA99"
#define SOUND_THRESHOLD_UUID      "5C3C8B61-82A9-4B92-835F-73927E9D9D5E"
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

void handleNotification(BLEDevice central, BLECharacteristic characteristic) {
  String uuid = String(characteristic.uuid());
  uuid.toUpperCase();

  // 🔍 หาชื่อของ Slave ที่เป็นเจ้าของ characteristic นี้
  String slaveName = "Unknown";
  for (int i = 0; i < numSlaves; i++) {
    if (
      characteristic == connectedSlaves[i].soundLevelChar ||
      characteristic == connectedSlaves[i].counterAllChar ||
      characteristic == connectedSlaves[i].counterAccChar ||
      characteristic == connectedSlaves[i].deviceIdChar ||
      characteristic == connectedSlaves[i].batteryChar ||
      characteristic == connectedSlaves[i].statusChar
    ) {
      slaveName = connectedSlaves[i].name;
      break;
    }
  }

  Serial.print("🔔 Notification from ");
  Serial.println(slaveName);

  // ✅ แสดงข้อมูลตาม UUID
  if (uuid == SOUND_LEVEL_UUID) {
    int16_t soundLevel;
    characteristic.readValue((byte*)&soundLevel, sizeof(soundLevel));
    Serial.print("🔊 Sound Level: ");
    Serial.println(soundLevel);

  } else if (uuid == COUNTER_ALL_UUID) {
    uint32_t counter;
    characteristic.readValue((byte*)&counter, sizeof(counter));
    Serial.print("🔢 Counter All: ");
    Serial.println(counter);

  } else if (uuid == COUNTER_ACC_UUID) {
    uint32_t counter;
    characteristic.readValue((byte*)&counter, sizeof(counter));
    Serial.print("🔄 Counter Acc: ");
    Serial.println(counter);

  } else if (uuid == DEVICE_ID_UUID) {
    byte id;
    characteristic.readValue(&id, 1);
    Serial.print("🆔 Device ID: ");
    Serial.println(id);

  } else if (uuid == BATTERY_UUID) {
    byte battery;
    characteristic.readValue(&battery, 1);
    Serial.print("🔋 Battery: ");
    Serial.println(battery == 0 ? "Charging" : battery == 1 ? "Full" : "Unknown");

  } else if (uuid == DEVICE_STATUS_UUID) {
    int16_t status;
    characteristic.readValue((byte*)&status, sizeof(status));
    Serial.print("📶 Status: ");
    switch (status) {
      case -1: Serial.println("❗ Error"); break;
      case 0:  Serial.println("🔄 Starting"); break;
      case 1:  Serial.println("🎯 Aiming"); break;
      case 2:  Serial.println("😴 Standby"); break;
      case 3:  Serial.println("🕰️ Idle"); break;
      case 4:  Serial.println("💤 Sleep"); break;
      default: Serial.println("❓ Unknown");
    }

  } else {
    Serial.println("❓ Unknown characteristic");
  }
}



void subscribeCharacteristics(SlaveNode &slave) {
  slave.soundLevelChar.setEventHandler(BLEUpdated, handleNotification);
  slave.soundLevelChar.subscribe();

  slave.counterAllChar.setEventHandler(BLEUpdated, handleNotification);
  slave.counterAllChar.subscribe();

  slave.counterAccChar.setEventHandler(BLEUpdated, handleNotification);
  slave.counterAccChar.subscribe();

  slave.deviceIdChar.setEventHandler(BLEUpdated, handleNotification);
  slave.deviceIdChar.subscribe();

  slave.batteryChar.setEventHandler(BLEUpdated, handleNotification);
  slave.batteryChar.subscribe();

  slave.statusChar.setEventHandler(BLEUpdated, handleNotification);
  slave.statusChar.subscribe();
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
        delay(300);

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

            subscribeCharacteristics(connectedSlaves[numSlaves - 1]);
            Serial.println("✅ Subscribed & Added slave: " + devName);
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

void setup() {
  Serial.begin(115200);
  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
    while (1);
  }

  delay(3000);
  scanAndConnectSlaves();

  Serial.println("\n📨 พิมพ์แบบนี้เพื่อเซ็ต threshold:");
  Serial.println("SLAVE-001:120 หรือ ArLaserTS001:150");
}

void loop() {
  BLE.poll();

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

  delay(10);
}
