#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0

#define VBATPIN A7
float vbatm = 0;
uint8_t lastiter = 0;
#define THRESHOLD_VOLTAGE 3.0

#define RED_PIN A1
#define GREEN_PIN A2
#define BLUE_PIN A3

#define MOTOR_PIN 18

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t button_value = 0;

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
  Serial.print("Voltage: ");
  Serial.println(vbatm);

  rgb_color(
    vbatm < THRESHOLD_VOLTAGE ? 
      Colors::RED
      : 
      Colors::GREEN
  );

  if (rf69.available()) {
    uint8_t buf[3];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      if (buf[0] == 1) {
        if (buf[2] == lastiter) {
          digitalWrite(MOTOR_PIN, LOW);
        }
        else {
          digitalWrite(MOTOR_PIN, HIGH);
          lastiter = buf[2];
          Serial.println(lastiter);
        }
      }
      else {
        digitalWrite(MOTOR_PIN, LOW);
      }
    }
    else {
      //    digitalWrite(led_unavailable, HIGH);
    }
    delay(10);
  }
}
