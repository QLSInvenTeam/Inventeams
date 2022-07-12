#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0
#define VBATPIN A0
float vbatm = 0;
#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
  

#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t button_pin = 6;
boolean button_state = LOW;
uint8_t button_value = 0;
int16_t packetnum = 0;  // packet counter, we increment per xmission
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void setup() 
{
  pinMode(button_pin, INPUT);
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}



void loop() {
  vbatm = analogRead(VBATPIN);
  vbatm*=2;
  vbatm*=3.3;
  vbatm/=1024;
  Serial.print("Voltage: ");
  Serial.println(vbatm);
  button_state = digitalRead(button_pin);
  //delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!
  //char radiopacket[20] = "Hello World #";
  //itoa(packetnum++, radiopacket+13, 10);
  //Serial.print("Sending "); Serial.println(radiopacket);
  //delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!
  if(button_state == HIGH) {
    button_value = 1;
    rf69.send(&button_value, sizeof(button_value));
  }
  else {
    button_value = 0;
    rf69.send(&button_value, sizeof(button_value));
  }
  Serial.print(button_state);
  Serial.print(button_value);
  // Send a message!
  //rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  //rf69.waitPacketSent();
  
  // Now wait for a reply
 // uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
 // uint8_t len = sizeof(buf);
 // if (rf69.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
   // if (rf69.recv(buf, &len)) {
     // Serial.print("Got a reply: ");
     // Serial.println((char*)buf);
     // Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    //} else {
    //  Serial.println("Receive failed");
   // }
 // } else {
    //Serial.println("No reply, is another RFM69 listening?");
  //}
}
