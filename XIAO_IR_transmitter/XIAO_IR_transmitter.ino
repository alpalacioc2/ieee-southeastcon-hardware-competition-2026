// advertises on BLE
// recieves 0x__ codes over BLE from Pi4
// transmits the codes it receives using NEC format
// sends each code 5 times for greater transmission reliability

#include <IRremote.hpp>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// --- IR ---
#define IR_SEND_PIN 5  // D3 on XIAO ESP32C3 = GPIO5

// --- BLE UUIDs (keep these the same on Pi) ---
#define SERVICE_UUID  "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID_CMD "abcdefab-1234-5678-1234-56789abcdef0"

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String v = pCharacteristic->getValue();
    if (v.length() == 0) return;

    for (size_t i = 0; i < v.length(); i++) {
      uint8_t b = (uint8_t)v[i];

      Serial.printf("Got cmd: 0x%02X\n", b);

      // Send each command 5 total times (1 + 4 repeats)
      IrSender.sendNEC(0x00, b, 4);

      delay(120);
    }
  }
};

void setup() {
  Serial.begin(115200);

  // IR init
  IrSender.begin(IR_SEND_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("IR ready");

  // BLE init
  BLEDevice::init("XIAO-IR-BRIDGE");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  BLECharacteristic *cmdChar = service->createCharacteristic(
    CHAR_UUID_CMD,
    BLECharacteristic::PROPERTY_WRITE
  );
  cmdChar->setCallbacks(new CmdCallbacks());

  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("BLE advertising as XIAO-IR-BRIDGE");
}

void loop() {
  delay(1000);
}