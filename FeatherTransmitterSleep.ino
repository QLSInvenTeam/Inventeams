#include <SPI.h>
#include <RH_RF69.h>
#include <ArduinoLowPower.h>

#define RF69_FREQ 915.0
#define VBATPIN A7
float vbatm = 0;
#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
  

#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
bool sleep = false;
uint8_t sleep_pin = 5;
uint8_t button_pin = 6;
uint8_t button_value = 0;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void toggleSleep() {
  sleep = !sleep;
  Serial.println(sleep);
}
void setup() 
{
  pinMode(button_pin, INPUT);
  Serial.begin(115200);
  
  
  //check if button has been pressed on sleep pin, then toggle sleep value
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(button_pin), toggleSleep, FALLING);
  
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

uint8_t debounce(uint8_t pintodebounce) {
    uint8_t debounced_state = 0;
    debounced_state = (debounced_state << 1) | digitalRead(pintodebounce) | 0xfe00;
    return debounced_state;
}

void loop() {
  vbatm = analogRead(VBATPIN);
  vbatm*=2;
  vbatm*=3.3;
  vbatm/=1024;
  Serial.print("Voltage: ");
  Serial.println(vbatm);
  if(sleep) {
    Serial.println("sleeping");
    LowPower.deepSleep();
  }
  uint8_t button_state = debounce(button_pin);
  if(button_state == 1) {
    button_value = 1;
    rf69.send(&button_value, sizeof(button_value));
  }
  else {
    button_value = 0;
    rf69.send(&button_value, sizeof(button_value));
  }
}

