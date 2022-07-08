#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0


#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           2
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint8_t motor_pin = 13;
uint8_t led_unavailable = 5;
uint8_t button_value = 0;
int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
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
 if (rf69.available()) {
    // Should be a message for us now
    int receivertime = millis();
    uint8_t transmittertime;   
    uint8_t len = sizeof(transmittertime);
    if (rf69.recv(&transmittertime, &len)) {
      if (!len) return;
      if(transmittertime!=0) {
        Serial.print("delay: " +  (receivertime) - (int)(transmittertime)) + " milliseconds");
      }
      digitalWrite(led_unavailable, LOW);
     // Serial.print("Received [");
     // Serial.print(len);
     // Serial.print("]: ");
      //Serial.println(button_value);
      //Serial.print("RSSI: ");
      //Serial.println(rf69.lastRssi(), DEC);

      //if (strstr((char *)buf, "Hello World")) {
        // Send a reply!
       // uint8_t data[] = "And hello back to you";
       // rf69.send(data, sizeof(data));
       // rf69.waitPacketSent();
       // Serial.println("Sent a reply");
       // Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
     // }
   // } else {
     // Serial.println("Receive failed");
    // }
  }
  else {
    digitalWrite(led_unavailable, HIGH);
  }
  delay(10);
}
}
