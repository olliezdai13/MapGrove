#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Adjust pins to match the wires from the GPS module (GPS TX -> GPS_RX_PIN).
const uint8_t GPS_RX_PIN = 4;  // Arduino pin receiving data from GPS
const uint8_t GPS_TX_PIN = 3;  // Arduino pin sending data to GPS (rarely used)
const unsigned long GPS_BAUD = 9600;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(GPS_BAUD);
  while (!Serial) {
    ;  // Wait for Serial Monitor (for boards with native USB)
  }
  Serial.println(F("[GPS] Debug monitor starting up..."));

  gpsSerial.begin(GPS_BAUD);
  Serial.println(F("[GPS] Listening for NMEA sentences..."));
}

void loop() {
  static unsigned long lastStatus = 0;

  // Relay GPS bytes straight to the Serial Monitor.
  while (gpsSerial.available()) {
    char incoming = gpsSerial.read();
    gps.encode(incoming);
    digitalWrite(LED_BUILTIN, HIGH);  // Light LED when data arrives
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    Serial.print(F("[GPS] Location fix: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
    if (gps.satellites.isValid()) {
      Serial.print(F(" | Sats: "));
      Serial.print(gps.satellites.value());
    }
    if (gps.hdop.isValid()) {
      Serial.print(F(" | HDOP: "));
      Serial.print(gps.hdop.hdop(), 1);
    }
    Serial.println();
  }

  // Periodic status heartbeat so we know the sketch is alive.
  if (millis() - lastStatus >= 2000) {
    lastStatus = millis();
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.print(F("\n[GPS] Uptime (ms): "));
    // Serial.print(millis());
    // Serial.print(F(" | Chars processed: "));
    // Serial.println(gps.charsProcessed());
    if (gps.charsProcessed() < 10) {
      Serial.println(F("[GPS] Waiting for data..."));
    } else if (!gps.location.isValid()) {
      Serial.println(F("[GPS] No location fix yet."));
    }
  }
}
