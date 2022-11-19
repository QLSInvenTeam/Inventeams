#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0

#define VBATPIN A7
float vbatm = 0;
uint8_t lastiter = 0;
#define THRESHOLD_VOLTAGE 3.7

#define RED_PIN A1
#define GREEN_PIN A2
#define BLUE_PIN A3

#define BUTTON_PIN 12

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

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
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

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
//  buf[1] = getChipID();
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

  uint8_t button_state = digitalRead(BUTTON_PIN);
  buf[0] = button_state;
  rf69.send(buf, sizeof(buf));
}

//string getChipID() {
//  volatile uint32_t val1, val2, val3, val4;
//  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
//  val1 = *ptr1;
//  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
//  val2 = *ptr;
//  ptr++;
//  val3 = *ptr;
//  ptr++;
//  val4 = *ptr;
//
//  string concatenate = val1 + "" + val2 + "" + val3 + "" + val4;
//  return concatenate;
//}
