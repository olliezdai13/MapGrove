#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// GPS wiring: module TX -> GPS_RX_PIN, module RX -> GPS_TX_PIN
const uint8_t GPS_RX_PIN = 4;
const uint8_t GPS_TX_PIN = 5;
const uint8_t BUTTON_PIN = 2;  // Expects HIGH when pressed
const unsigned long GPS_BAUD = 9600;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

struct GpsSnapshot {
  bool valid;
  double lat;
  double lng;
  uint8_t satellites;
  unsigned long fixMillis;
};

static GpsSnapshot lastGps = {false, 0.0, 0.0, 0, 0};
static bool headerPrinted = false;
static bool lastButtonHigh = false;

void feedGpsStream();
void updateGpsSnapshot();
void maybeLogButtonPress();
void logCurrentState(const unsigned long pressedAtMillis);

void setup() {
  Serial.begin(GPS_BAUD);
#if defined(USBCON)
  while (!Serial) {
    ;  // Wait for Serial Monitor (for boards with native USB)
  }
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lastButtonHigh = (digitalRead(BUTTON_PIN) == HIGH);

  gpsSerial.begin(GPS_BAUD);
}

void loop() {
  feedGpsStream();
  updateGpsSnapshot();
  maybeLogButtonPress();
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

void maybeLogButtonPress() {
  const bool nowHigh = (digitalRead(BUTTON_PIN) == HIGH);
  if (!lastButtonHigh && nowHigh) {  // rising edge
    logCurrentState(millis());
  }
  lastButtonHigh = nowHigh;
}

void logCurrentState(const unsigned long pressedAtMillis) {
  if (!headerPrinted) {
    Serial.println(F("button,gps_lon,gps_lat,gps_lock,timestamp_ms"));
    headerPrinted = true;
  }

  const bool gpsCurrentlyLocked =
      gps.location.isValid() && gps.location.age() <= 5000UL;
  const bool haveFix = lastGps.valid;
  const double logLat = haveFix ? lastGps.lat : 0.0;
  const double logLng = haveFix ? lastGps.lng : 0.0;

  Serial.print(1);
  Serial.print(',');
  Serial.print(logLng, 6);
  Serial.print(',');
  Serial.print(logLat, 6);
  Serial.print(',');
  Serial.print(gpsCurrentlyLocked ? 1 : 0);
  Serial.print(',');
  Serial.println(pressedAtMillis);
}
