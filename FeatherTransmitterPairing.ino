#include <FlashStorage.h>
#include <RH_RF69.h>
#include <PinButton.h>
#include <array>

#define RF69_FREQ 915.0

#define VBATPIN A7
#define THRESHOLD_VOLTAGE 3.7

#define UUID_LEN 16

#define RED_PIN A1
#define GREEN_PIN A2
#define BLUE_PIN A3

#define BUTTON_PIN 10
#define PAIRING_PIN 12

#define RAND_DELAY_MIN 10
#define RAND_DELAY_MAX 20

#define KEY_MIN 0
#define KEY_MAX 128

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#endif

PinButton pairing_pin(PAIRING_PIN);

RH_RF69 rf69(RFM69_CS, RFM69_INT);

FlashStorage(syncwords_storage, uint32_t);

const uint8_t defaultwords[] = { 0x2d, 0x64, 0x64, 0x64 };

/*
  normal: normal operation
  pair_listen: listen from other transmitters
  pair_transmit: transmit your syncwords
*/
enum _state { normal, pair_listen, pair_transmit };
_state state;

struct packet {
  uint8_t syncwords[4];
  uint8_t deviceId[16];
  bool button_state;
};

packet data;

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t blink_interval;
};

namespace Colors {
Color const RED         = {255,   0,   0,   0};
Color const GREEN       = {  0, 255,   0,   0};
Color const BLUE        = {  0,   0, 255,   0};

Color const RED_BLINK   = {255,   0,   0, 200};
Color const GREEN_BLINK = {  0, 255,   0, 200};
Color const BLUE_BLINK  = {  0,   0, 255, 200};

Color const OFF         = {  0,   0,   0,   0};
}

typedef std::array<uint8_t, UUID_LEN> uuid;

uuid getChipID() {
  uuid chipId;

  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;
  uint32_t orig[4] = {val1, val2, val3, val4};

  //https://stackoverflow.com/questions/6499183/converting-a-uint32-value-into-a-uint8-array4
  for (int i = 0; i < 4; i++) {
    uint8_t *origp = (uint8_t *)&orig[i];
    for (int j = 0; j < 4; j++) {
      chipId[4 * i + j] = origp[j];
    }
  }
  return chipId;
}

void rgb_color(Color color) {
  static unsigned long previousMillis = 0;
  static bool blinkState = HIGH;

  unsigned long currentMillis = millis();

  if (color.blink_interval) {
    if (currentMillis - previousMillis >= color.blink_interval) {
      previousMillis = currentMillis;
      blinkState = !blinkState;
    }
    if (blinkState == LOW) {
      color = Colors::OFF;
    }
  }

  analogWrite(RED_PIN, color.r);
  analogWrite(GREEN_PIN, color.g);
  analogWrite(BLUE_PIN, color.b);
}

float get_voltage() {
  return analogRead(VBATPIN) * 2 * 3.3 / 1024;
}

boolean debounce() {
  uint8_t debounced_state = 0;
  debounced_state = (debounced_state << 1) | digitalRead(BUTTON_PIN) | 0xfe00;
  return debounced_state;
}

void random_delay() {
  delay(random(RAND_DELAY_MIN, RAND_DELAY_MAX));
}

uint32_t u32from8(uint8_t b[4]) {
  uint32_t u;
  u = b[0];
  u = (u  << 8) + b[1];
  u = (u  << 8) + b[2];
  u = (u  << 8) + b[3];
  return u;
}

void u8from32 (uint8_t b[4], uint32_t u32) {
  b[3] = (uint8_t)u32;
  b[2] = (uint8_t)(u32 >>= 8);
  b[1] = (uint8_t)(u32 >>= 8);
  b[0] = (uint8_t)(u32 >>= 8);
}

uint32_t generate_syncwords() {
  uint8_t newwords[4];
  for (int i = 0; i < 4; i++) {
    newwords[i] = random(KEY_MIN, KEY_MAX);
  }
  return u32from8(newwords);
}

void setup() {
  Serial.begin(115200);

  randomSeed(analogRead(A0));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PAIRING_PIN, INPUT_PULLUP);

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

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

  uuid UUID = getChipID();
  for (int i = 0; i < UUID_LEN; i++) {
    data.deviceId[i] = UUID[i];
  }

  uint32_t oldwords = syncwords_storage.read();
  // Load previous words if they exist
  if (oldwords != 0) {
    storeSyncWords(oldwords, false);
  } else {
    uint32_t newwords = generate_syncwords();
    storeSyncWords(newwords, true);
  }
  enterNormal();
}

void storeSyncWords(uint32_t newwords, boolean store) {
  uint8_t words8[4];
  u8from32(words8, newwords);
  for (int i = 0; i < 4; i++) {
    data.syncwords[i] = words8[i];
  }
  if (store) {
    syncwords_storage.write(newwords);
  }
}

void enterPairTransmit() {
  state = pair_transmit;
  rf69.setSyncWords(defaultwords, sizeof(defaultwords));
}

void enterNormal() {
  state = normal;
  rf69.setSyncWords(data.syncwords, sizeof(data.syncwords));
}

void enterPairListen() {
  state = pair_listen;
  rf69.setSyncWords(defaultwords, sizeof(defaultwords));
}

void loop() {
  float vbatm = get_voltage();
  //  Serial.print("Voltage: ");
  //  Serial.println(vbatm);

  pairing_pin.update();
  if (pairing_pin.isSingleClick()) {
    if (state == pair_transmit || state == pair_listen) {
      enterNormal();
    } else {
      enterPairTransmit();
    }
  }

  if (pairing_pin.isDoubleClick()) {
    enterPairListen();
  }

  if (state == pair_transmit) {
    uint8_t buf[sizeof(data)];
    memcpy(buf, &data, sizeof(data));
    rf69.send(buf, sizeof(buf));
    Serial.println("Packet sent");
    rgb_color(Colors::GREEN_BLINK);
  }

  if (state == pair_listen) {
    Serial.println("Pair_listen");
    if (rf69.available()) {
      Serial.println("Packet recieved");

      uint8_t buf[sizeof(data)];
      uint8_t len = sizeof(buf);
      packet recv;

      if (rf69.recv(buf, &len)) {
        if (!len) return;
        memcpy(&recv, buf, sizeof(data));
        storeSyncWords(u32from8(recv.syncwords), true);
        for (int i = 0; i < 3; i++) {
          rgb_color(Colors::GREEN);
          delay(500);
          rgb_color(Colors::OFF);
          delay(500);
        }
        enterNormal();
      }
    }
    rgb_color(Colors::BLUE_BLINK);
  }

  if (state == normal) {
    data.button_state = debounce();
    //    Serial.println("Button State:");
    //    Serial.println(data.button_state);
    uint8_t buf[sizeof(data)];
    memcpy(buf, &data, sizeof(data));
    rf69.send(buf, sizeof(buf));

    rgb_color(
      vbatm < THRESHOLD_VOLTAGE ?
      Colors::RED
      :
      Colors::GREEN
    );
  }


  random_delay();
}
