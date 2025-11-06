#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Adjust pins to match the wires from the GPS module (GPS TX -> GPS_RX_PIN).
const uint8_t GPS_RX_PIN = 4;  // Arduino pin receiving data from GPS
const uint8_t GPS_TX_PIN = 5;  // Arduino pin sending data to GPS (rarely used)
const uint8_t BUTTON_PIN = 2;  // Button input (active LOW with pull-up)
const unsigned long GPS_BAUD = 9600;
const unsigned long LOG_PERIOD_MS = 250UL;

// SD card pins (r -> l)
// CS - D10
// SCK - D13
// MOSI - D11
// MISO - D12
// VCC - voltage
// GND - ground

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

struct GpsSnapshot {
  bool valid;
  double lat;
  double lng;
  uint8_t satellites;
  unsigned long fixMillis;
};

struct ButtonSnapshot {
  bool pressed;
  unsigned long sampleMillis;
};

static GpsSnapshot lastGps = {false, 0.0, 0.0, 0, 0};
static ButtonSnapshot lastButton = {false, 0};
static unsigned long lastLogMillis = 0;

void feedGpsStream();
void updateGpsSnapshot();
void pollButton(const unsigned long now);
void logCurrentState(const unsigned long now);

void setup() {
  Serial.begin(GPS_BAUD);
  while (!Serial) {
    ;  // Wait for Serial Monitor (for boards with native USB)
  }
  Serial.println(F("[GPS] Debug monitor starting up..."));

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lastButton.pressed = (digitalRead(BUTTON_PIN) == LOW);
  lastButton.sampleMillis = millis();
  lastLogMillis = lastButton.sampleMillis;

  gpsSerial.begin(GPS_BAUD);
  Serial.println(F("[GPS] Listening for NMEA sentences..."));
}

void loop() {
  feedGpsStream();
  updateGpsSnapshot();

  const unsigned long now = millis();
  if (now - lastLogMillis >= LOG_PERIOD_MS) {
    lastLogMillis = now;
    pollButton(now);
    logCurrentState(now);
  }
}

void feedGpsStream() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

void updateGpsSnapshot() {
  if (gps.location.isUpdated() && gps.location.isValid()) {
    lastGps.valid = true;
    lastGps.lat = gps.location.lat();
    lastGps.lng = gps.location.lng();
    lastGps.fixMillis = millis();
  }

  if (gps.satellites.isValid()) {
    lastGps.satellites = gps.satellites.value();
  }
}

void pollButton(const unsigned long now) {
  lastButton.pressed = (digitalRead(BUTTON_PIN) == LOW);
  lastButton.sampleMillis = now;
}

void logCurrentState(const unsigned long now) {
  Serial.print(F("[LOG] t="));
  Serial.print(now);
  Serial.print(F("ms BTN="));
  Serial.print(lastButton.pressed ? F("Pressed") : F("Released"));
  Serial.print(F(" GPS="));

  if (!lastGps.valid) {
    Serial.print(F("NoFix"));
  } else {
    Serial.print(lastGps.lat, 6);
    Serial.print(F(","));
    Serial.print(lastGps.lng, 6);
    Serial.print(F(" sats="));
    Serial.print(lastGps.satellites);
    Serial.print(F(" age="));
    Serial.print(now - lastGps.fixMillis);
    Serial.print(F("ms"));
  }

  Serial.println();
}
