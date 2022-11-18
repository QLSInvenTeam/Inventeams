#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define VBATPIN A0
float vbatm = 0;
uint8_t lastiter = 0;
#define THRESHOLD_VOLTAGE 3.7

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
  Serial.print("Voltage: ");
  Serial.println(vbatm);
  
  if (vbatm < THRESHOLD_VOLTAGE) {
    rgb_color(255, 0, 0);
  } else {
    rgb_color(0, 255, 0);
  }
  
 if (rf69.available()) {
    string buf[3];
    uint8_t len = sizeof(buf);
    if (rf69.recv(&buf, &len)) {
      if (!len) return;
      if(buf[0] == "1") {
        if(buf[2] == lastiter) {
           digitalWrite(motor_pin, LOW); 
        }
        else {
          digitalWrite(motor_pin, HIGH); 
          lastiter = buf[2];
          Serial.println(lastiter);
        }
      }
      else {
       digitalWrite(motor_pin, LOW); 
      }
      digitalWrite(led_unavailable, LOW);
//       if(button_value == 1) {
//         digitalWrite(motor_pin, HIGH);
//       }
//       else {
//         digitalWrite(motor_pin, LOW);
//       }
//       digitalWrite(led_unavailable, LOW);
      //Serial.print("Received [");
      //Serial.print(len);
      //Serial.print("]: ");
      //Serial.println(button_value);
      //Serial.print("RSSI: ");
      //Serial.println(rf69.lastRssi(), DEC);
  }
  else {
    digitalWrite(led_unavailable, HIGH);
  }
  delay(10);
}
}
