#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
uint8_t NEW_FREQ=2;

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

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

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 ,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
}

void loop() {
  Serial.print("Broadcasting offset [");
  Serial.print(NEW_FREQ);
  Serial.print("] on ");
  Serial.print(RF69_FREQ);
  Serial.println("MHz ");
  
  rf69.send(&NEW_FREQ, sizeof(NEW_FREQ));
}
