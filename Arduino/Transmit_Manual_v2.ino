#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13

// -------------------- CONFIG --------------------
#define TRANSMITTER_ID 1             // Change for each transmitter
#define PACKETS_PER_SECOND 0.75      // How many packets per second to send
#define PACKET_INTERVAL_MS (1000 / PACKETS_PER_SECOND)

// Hardcoded GPS coordinates
#define FIXED_LATITUDE  -37.777040     // Example: Melbourne
#define FIXED_LONGITUDE 145.050560
// -----------------------------------------------

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  blinkLED(2, 200);

  Serial.begin(9600);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    while (1) { blinkLED(1, 500); delay(500); }
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1) { blinkLED(2, 500); delay(500); }
  }
  rf95.setTxPower(23, false);

  blinkLED(5, 100);
  Serial.println("LoRa init done with hardcoded GPS");
}

void loop() {
  static unsigned long lastPacketTime = 0;
  unsigned long now = millis();

  // Send packet at defined interval
  if (now - lastPacketTime >= PACKET_INTERVAL_MS) {
    lastPacketTime = now;

    char msg[64];
    snprintf(msg, sizeof(msg), "T%d:%.6f,%.6f", TRANSMITTER_ID, FIXED_LATITUDE, FIXED_LONGITUDE);

    rf95.send((uint8_t*)msg, strlen(msg));
    rf95.waitPacketSent();

    Serial.print("Sent hardcoded GPS packet: ");
    Serial.println(msg);

    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
}
