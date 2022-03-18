
/*
//11/19/2021 RECIEVER for HAPTIC DEVICE
//Derived from 211115_Inventeams_Reciever_HapticDevice
//Changes: 
      - Haptic device checks less often, then buzzes for a fixed time
*/
#include <SPI.h> // 11-15 addition
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

RF24 radio(7, 8); // CE, CSN
const byte address[1] = "00002"; //radio 'channel'
boolean button_state = 0;
int motor_pin = 2;
int button_value1 = 0; 
int button_value2 = 0; 
int led_unavailable = 5;
float data[4];
float data1;
float data2;
int csn = 9;
int ce = 7;
void setup() {
pinMode(motor_pin, OUTPUT);
pinMode(led_unavailable, OUTPUT); //11-15 addition
Serial.begin(9600);
radio.begin();
radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
radio.setDataRate(RF24_250KBPS); // data sent 250 KB per second
radio.setChannel(108);
radio.startListening();              //This sets the module as receiver

pinMode(csn, HIGH); 
pinMode(ce, HIGH);

//Initialization Sequence//
digitalWrite(led_unavailable, HIGH);
delay(1000);
digitalWrite(led_unavailable, LOW);

digitalWrite(motor_pin, HIGH);
delay(250);
digitalWrite(motor_pin, LOW);
delay(250);
digitalWrite(motor_pin, HIGH);
delay(250);
digitalWrite(motor_pin, LOW);
}

void loop() {
if (radio.available()){              //Looking for the data.
    //radio.read(&button_value, sizeof(button_value));    //Reading the data
    if(data[0]==501) {
       data1 = data[1];
       button_value1 = data1;
    }
    else if(data[0]==601) {
       data2 = data[1];
       button_value2 = data2;
    }
    //Serial.println(button_value);
    Serial.println(button_value);
    if(button_value1 == 1 || button_value2==1){
      digitalWrite(motor_pin, HIGH);
      delay(1000);
      digitalWrite(motor_pin,LOW);
      button_value1 = 0;
      button_value2 = 0;
      Serial.println(button_value);
    }
    else if(button_value1 == 28505 || button_value2 == 28505){
    }
    else {
      digitalWrite(motor_pin, LOW);
    }
    //Serial.println("here");
    digitalWrite(led_unavailable, LOW);
  }
  else {
    //Serial.println("unavailable");
    digitalWrite(led_unavailable, HIGH);
    //button_value = 0;
  }
delay(10);
}
