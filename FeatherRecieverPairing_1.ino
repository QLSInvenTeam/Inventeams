#include <SPI.h>
#include <RH_RF69.h>

#include <array>
#include <map>
#include <algorithm>

#define RF69_FREQ 915.0

#define VBATPIN A7
bool pairing_done = false;
#define THRESHOLD_VOLTAGE 3.0

#define DISCONNECT_TIMEOUT 2000

#define RED_PIN A1
#define GREEN_PIN A2
#define BLUE_PIN A3

#define UUID_LEN 16


#define MOTOR_PIN 18
#define PAIRING_PIN 13

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#endif

enum _state { pairing, normal };
_state state = pairing;

RH_RF69 rf69(RFM69_CS, RFM69_INT);

typedef std::array<uint8_t, UUID_LEN> uuid;
uint8_t syncwords[4];

std::map<uuid, std::pair<boolean, unsigned long>> devices;

struct packet {
  uint8_t syncwords[4];
  uint8_t deviceId[16];
  bool button_state;
};

packet data;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t blink_interval;
};

namespace Colors {
Color const RED         = {255,   0,   0,   0};
Color const GREEN       = {  0, 255,   0,   0};
Color const BLUE        = {  0,   0, 255,   0};

Color const RED_BLINK   = {255,   0,   0, 500};
Color const GREEN_BLINK = {  0, 255,   0, 500};
Color const BLUE_BLINK  = {  0,   0, 255, 500};

Color const OFF         = {  0,   0,   0,   0};
}

void rgb_color(Color color) {
  static unsigned long previousMillis = 0;
  static bool blinkState = HIGH;

  unsigned long currentMillis = millis();

  if (color.blink_interval != 0) {
    if (currentMillis - previousMillis >= color.blink_interval) {
      previousMillis = currentMillis;
    }
    blinkState = !blinkState;
    if (blinkState == LOW) {
      color = Colors::OFF;
    }
  }

  analogWrite(RED_PIN, color.r);
  analogWrite(GREEN_PIN, color.g);
  analogWrite(BLUE_PIN, color.b);
}


float get_voltage() {
  return analogRead(VBATPIN) * 2 * 3.3 / 1024;
}

void setup()
{
  Serial.begin(115200);

  pinMode(PAIRING_PIN, INPUT_PULLUP);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  //  reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);


  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);
}


void loop() {
  float vbatm = get_voltage();
  //  Serial.print("Voltage: ");
  //  Serial.println(vbatm);



  uint8_t pairing_state = digitalRead(PAIRING_PIN);
  // just hold button down
  if (!pairing_state) {
    if (rf69.available()) {
      uint8_t buf[sizeof(data)];
      uint8_t len = sizeof(buf);
      if (rf69.recv(buf, &len)) {
        if (!len) return;
        memcpy(&data, buf, sizeof(data));
        // success sequence
        for (int i = 0; i < 3; i++) {
          rgb_color(Colors::GREEN);
          delay(500);
          rgb_color(Colors::OFF);
          delay(500);
        }
        state = normal;
        rf69.setSyncWords(data.syncwords, sizeof(data.syncwords));
      }
    }
    rgb_color(Colors::BLUE_BLINK);
  }
  else {
    // normal operation
    rgb_color(
      vbatm < THRESHOLD_VOLTAGE ?
      Colors::RED
      :
      Colors::GREEN
    
    );
    if (rf69.available()) {
      uint8_t buf[sizeof(data)];
      uint8_t len = sizeof(buf);
      if (rf69.recv(buf, &len)) {
        if (!len) return;
        memcpy(&data, buf, sizeof(data));
        uuid deviceId;
        std::copy_n(std::begin(deviceId), UUID_LEN, std::begin(data.deviceId));
        // !! millis overflows in 70 days
        // https://www.norwegiancreations.com/2018/10/arduino-tutorial-avoiding-the-overflow-issue-when-using-millis-and-micros/
        devices[deviceId] = std::make_pair(!(data.button_state), millis());
      }
    }

    unsigned long timestamp = millis();

    // Set device to LOW if signal timed out
    for (auto& device : devices) {
      if (device.second.first && timestamp - device.second.second > DISCONNECT_TIMEOUT) {
        device.second.first = false;
        device.second.second = timestamp;
      }
    }

    bool buzz = false;
    for (auto& device : devices) {
      if (device.second.first) {
        buzz = true;
      }
    }

    digitalWrite(MOTOR_PIN, buzz ? HIGH : LOW);

  }
}
