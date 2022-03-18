/* 11/12/21 TRANSMITTER CODE for TRANSMITTER JW SETUP
  //arduino cloud version of the code, aka as close as possible to the original (text data)
  //Other Changes
            - RF24 library
            - address from [6]002 --> [1]002
            - button value, no button state
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

int csn = 8;
int ce = 7;
RF24 radio(ce, csn); // CE, CSN
const byte address[1] = "00002";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int button_pin = 3;
float data[4];
boolean button_state = LOW;
int button_value = 0;
int testPin = 2;
void setup() {
  pinMode(button_pin, INPUT);
  pinMode(testPin, OUTPUT);

  Serial.begin(9600);
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_250KBPS); // data sent 250 KB per second
  radio.setChannel(108);
  radio.stopListening();          //This sets the module as transmitter

  //not sure if this is necessary, but it doesn't seem to but harming anything..
  //probably necessary for deciding which reciever to send data to
  pinMode(csn, HIGH);
  pinMode(ce, HIGH);

}
void loop() {
  button_state = digitalRead(button_pin);
  data[0] = 501;
  //Serial.println(button_value);
  if (button_state == HIGH) {
    button_value = 1;
    data[1] = button_value;
    int time = millis();
    radio.write(&button_value, sizeof(button_value));
    //Sending the message to receiver
    delay(1000);
    //Serial.println("pressed");
    //const char text[] = "Your Button State is HIGH";
    //radio.write(&text, sizeof(text));                  //Sending the message to receiver
    //digitalWrite(testPin, HIGH);
  }
  else {
    button_value = 0;
    data[1] = button_value;
    radio.write(&button_value, sizeof(button_value));
    //Serial.println("not pressed");
    //const char text[] = "Your Button State is LOW";
    //radio.write(&text, sizeof(text));
    //digitalWrite(testPin,LOW);
  }
  //radio.write(&button_value, sizeof(button_value));  //Sending the message to receiver
  Serial.println(button_value);
  delay(10);
}
