#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define VBATPIN A7
float vbatm = 0;
#define THRESHOLD_VOLTAGE 3.0

#define RED_PIN A1
#define GREEN_PIN A2
#define BLUE_PIN A3

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           2
#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t motor_pin = 18;
uint8_t led_unavailable = 5;
uint8_t button_value = 0;

void rgb_color(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

void setup() 
{
  Serial.begin(115200);
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(motor_pin, OUTPUT);

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
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
}


void loop() {
  vbatm = analogRead(VBATPIN);
  vbatm*=2;
  vbatm*=3.3;
  vbatm/=1024;
//  Serial.print("Voltage: ");
//  Serial.println(vbatm);
  
  if (vbatm < THRESHOLD_VOLTAGE) {
    rgb_color(255, 0, 0);
  } else {
    rgb_color(0, 255, 0);
  }
  
 if (rf69.available()) {
    uint8_t button_value;   
    uint8_t len = sizeof(button_value);
    if (rf69.recv(&button_value, &len)) {
      if (!len) return;
      Serial.println(button_value);
      if(button_value == 1) {
        digitalWrite(motor_pin, HIGH);
      }
      else {
        digitalWrite(motor_pin, LOW);
      }
      digitalWrite(led_unavailable, LOW);
  }
  else {
    digitalWrite(led_unavailable, HIGH);
  }
}
}
