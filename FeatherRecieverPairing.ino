#include <SPI.h>
#include <RH_RF69.h>

#include <array>
#include <map>
#include <algorithm>

#define RF69_FREQ 915.0

#define VBATPIN A7
float vbatm = 0;
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


RH_RF69 rf69(RFM69_CS, RFM69_INT);

typedef std::array<uint8_t, UUID_LEN> uuid;
uint8_t syncwords[4];

std::map<uuid, std::pair<boolean, unsigned long>> devices;

typedef struct {
  int r;
  int g;
  int b;
} Color;

namespace Colors {
Color RED = {255, 0, 0};
Color GREEN = {0, 255, 0};
}

void rgb_color(Color color) {
  analogWrite(RED_PIN, color.r);
  analogWrite(GREEN_PIN, color.g);
  analogWrite(BLUE_PIN, color.b);
}

float get_voltage() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  return measuredvbat;
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
  vbatm = get_voltage();
  //  Serial.print("Voltage: ");
  //  Serial.println(vbatm);

  rgb_color(
    vbatm < THRESHOLD_VOLTAGE ?
    Colors::RED
    :
    Colors::GREEN
  );

  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      uint8_t pairing_state = digitalRead(BUTTON_PIN);
      if(!pairing_state && !pairing_done) {
        uuid deviceId;
        std::copy_n(std::begin(buf), UUID_LEN, std::begin(deviceId));
        // !! millis overflows in 70 days
        // https://www.norwegiancreations.com/2018/10/arduino-tutorial-avoiding-the-overflow-issue-when-using-millis-and-micros/
        devices[deviceId] = std::make_pair(!buf[UUID_LEN], millis());
      }
      else {
        std::copy(syncwords, buf+UUID_LEN, 4); 
        setSyncWords(syncwords, sizeof(syncwords));
        pairing_done = true;
      }

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

