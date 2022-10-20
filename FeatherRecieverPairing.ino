#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t motor_pin = 18;
uint8_t led_unavailable = 5;
uint8_t button_value = 0;


boolean pairing = true;
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 1000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED




void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
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

  pinMode(LED, OUTPUT);
}


void loop() {
  if (rf69.available()) {
    uint8_t freq;
    uint8_t len = sizeof(freq);
    if (rf69.recv(&freq, &len)) {
      if (!len) return;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println(freq);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);
      pairing = false;
    }
  }
  if (pairing) {
    Serial.println("Looking for signal");
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(LED, ledState);
  }
}
