#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define VBATPIN A0
float vbatm = 0;

#define THRESHOLD_VOLTAGE 3.7

#define RED_PIN A1
#define GREEN_PIN A2
#define BLUE_PIN A3

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
  

#endif


RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t button_pin = 12;
uint8_t button_value = 0;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];


void rgb_color(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

void setup() 
{
  pinMode(button_pin, INPUT);
  Serial.begin(115200);
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

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

uint8_t debounce() {
    uint8_t debounced_state = 0;
    debounced_state = (debounced_state << 1) | digitalRead(button_pin) | 0xfe00;
    return debounced_state;
}

void loop() {
  vbatm = analogRead(VBATPIN);
  vbatm*=2;
  vbatm*=3.3;
  vbatm/=1024;
  Serial.print("Voltage: ");
  Serial.println(vbatm);
  Serial.println("Chip ID: ");
  printChipID();
  if (vbatm < THRESHOLD_VOLTAGE) {
    rgb_color(255, 0, 0);
  } else {
    rgb_color(0, 255, 0);
  }
  uint8_t button_state = debounce();
  if(button_state == 1) {
    button_value = 1;
    rf69.send(&button_value, sizeof(button_value));
  }
  else {
    button_value = 0;
    rf69.send(&button_value, sizeof(button_value));
  }
}

void printChipID() {
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;

  Serial.print("chip id: 0x");
  char buf[33];
  sprintf(buf, "%8x%8x%8x%8x", val1, val2, val3, val4);
  Serial.println(buf);
}
