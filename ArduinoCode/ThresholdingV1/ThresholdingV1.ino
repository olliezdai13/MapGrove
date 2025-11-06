/*
 * Hardware & Wiring:
 * - SD Card (SPI): MOSI-11, MISO-12, SCK-13, CS-10
 * - MPU-6050 (I2C): SDA-A4, SCL-A5
 * - GPS Module (Software Serial):
 *     GPS TX -> Nano Pin D4 (RX)
 *     GPS RX -> Nano Pin D5 (TX)
 * - LED: Pin D3 -> Resistor (220-330 Ohm) -> LED -> GND
 */

#include <SdFat.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// --- Configuration ---
const uint8_t CHIP_SELECT = 10;
#define LOG_FILE  "datalog.csv"
#define LOG_FILE2 "detections.csv"
const int MPU_ADDR = 0x68;
const int LED_PIN   = 3; // LED for event window
const int BUTTON_PIN = 2; // Button for manual planting confirmation

// GPS Software Serial pins
static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

// Scale factors for MPU-6050 (±2g, ±250 dps defaults)
const float ACCEL_SCALE_FACTOR = 16384.0;
const float GYRO_SCALE_FACTOR  = 131.0;

// --- Global Objects & Variables ---
SdFat sd;
SdFile logFile;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS module

// MPU-6050 raw AND calculated data variables
int16_t rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ;
float accX, accY, accZ, gyroX, gyroY, gyroZ;

// Timing variables
unsigned long lastLogTime = 0;
const unsigned int LOG_INTERVAL_MS = 100; // 10 Hz

// LED window
bool isLedOn = false;
unsigned long ledOnTime = 0;
const unsigned int LED_ON_DURATION_MS = 5000; // 5 seconds
bool buttonPressedDuringLED = false;

// --- Planting FSM: DOWN (drop->spike) then UP (spike->drop) ---
enum PlantState { WAIT_DOWN_LOW, WAIT_DOWN_HIGH, WAIT_UP_HIGH, WAIT_UP_LOW };
PlantState pstate = WAIT_DOWN_LOW;

// "within next 10 samples" windows @10 Hz ≈ 1 s
uint8_t downWindow = 0;   // counts samples after the DOWN drop
uint8_t upWindow   = 0;   // counts samples after the UP spike
const uint8_t MAX_SAMPLES = 10;

// Thresholds in G (tune if needed)
const float DOWN_LOW_TH  = 0.90f; // drop below this = DOWN start
const float DOWN_HIGH_TH = 1.10f; // then spike above this = DOWN confirmed
const float UP_HIGH_TH   = 1.10f; // spike above this = UP start
const float UP_LOW_TH    = 0.90f; // then drop below this = UP confirmed

// Timing (ms)
const unsigned long PLANT_MIN_MS     = 5000; // 15 s (your requirement)
const unsigned long PLANT_MAX_MS     = 10000; // 30 s
const unsigned long PLANT_TIMEOUT_MS = 5000; // safety reset per phase

unsigned long downConfirmedMs = 0; // when DOWN was confirmed
unsigned long phaseStartMs    = 0; // when current phase started (for timeouts)

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // keep for bench debugging

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println(F("--- Data Logger Initializing ---"));

  // GPS
  ss.begin(GPSBaud);
  Serial.println(F("Software Serial for GPS started."));

  // MPU-6050
  Serial.println(F("Initializing MPU-6050..."));
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up
  Wire.endTransmission(true);
  Serial.println(F("MPU-6050 is awake."));

  // SD
  Serial.println(F("Initializing SD card..."));
  if (!sd.begin(CHIP_SELECT, SD_SCK_MHZ(4))) {
    Serial.println(F("******** SD Card initialization FAILED ********"));
    while (true) { digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW); delay(1000); }
  }
  Serial.println(F("SD card initialized successfully."));

  // Prepare datalog.csv
  if (!sd.exists(LOG_FILE)) {
    Serial.println(F("Creating new log file..."));
    if (!logFile.open(LOG_FILE, O_WRITE | O_CREAT)) {
      Serial.println(F("CRITICAL ERROR: Failed to create log file."));
      while (true) {}
    }
    logFile.println(F("Timestamp,AccX_G,AccY_G,AccZ_G,GyroX_DPS,GyroY_DPS,GyroZ_DPS,Lat,Lon,Altitude,Sats,PlantStat"));
    logFile.close();
    Serial.println(F("Log file created and header written."));
  } else {
    Serial.println(F("Log file exists. Appending data."));
  }

  // Prepare detections.csv
  if (!sd.exists(LOG_FILE2)) {
    Serial.println(F("Creating new detections file..."));
    if (!logFile.open(LOG_FILE2, O_WRITE | O_CREAT)) {
      Serial.println(F("CRITICAL ERROR: Failed to create detections file."));
      while (true) {}
    }
    logFile.println(F("Timestamp,Lat,Lon,Altitude,Sats,PlantStat"));
    logFile.close();
    Serial.println(F("Detections file created and header written."));
  } else {
    Serial.println(F("Detections file exists. Appending data."));
  }

  Serial.println(F("--- Initialization Complete ---"));
  Serial.println(F("Waiting for GPS lock... (This can take several minutes)"));

  lastLogTime = millis();
  phaseStartMs = lastLogTime; // initialize per-phase timer
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  // LED timeout (non-blocking)
  if (isLedOn && (millis() - ledOnTime >= LED_ON_DURATION_MS)) {
    digitalWrite(LED_PIN, LOW);
    int finalPlantStat = buttonPressedDuringLED ? 2 : 1;
    writeDetection(finalPlantStat);
    isLedOn = false;
    buttonPressedDuringLED = false;
  }

  // Feed GPS parser
  while (ss.available() > 0) { gps.encode(ss.read()); }

  // Run at 10 Hz
  if (millis() - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = millis();

    // Button
    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
    if (buttonPressed && isLedOn) { buttonPressedDuringLED = true; }

    // Read MPU-6050 (14 bytes)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    rawAccX = (int16_t)((Wire.read() << 8) | Wire.read());
    rawAccY = (int16_t)((Wire.read() << 8) | Wire.read());
    rawAccZ = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // skip temp
    rawGyroX = (int16_t)((Wire.read() << 8) | Wire.read());
    rawGyroY = (int16_t)((Wire.read() << 8) | Wire.read());
    rawGyroZ = (int16_t)((Wire.read() << 8) | Wire.read());

    accX = rawAccX / ACCEL_SCALE_FACTOR;
    accY = rawAccY / ACCEL_SCALE_FACTOR;
    accZ = rawAccZ / ACCEL_SCALE_FACTOR;
    gyroX = rawGyroX / GYRO_SCALE_FACTOR;
    gyroY = rawGyroY / GYRO_SCALE_FACTOR;
    gyroZ = rawGyroZ / GYRO_SCALE_FACTOR;

    // --- Planting detector: DOWN -> timer -> UP -> fire LED window ---
    switch (pstate) {

      // 1) Wait for a drop below DOWN_LOW_TH, then give 10 samples to rise
      case WAIT_DOWN_LOW:
        if (accZ < DOWN_LOW_TH) {
          pstate = WAIT_DOWN_HIGH;          // saw drop
          downWindow = 0;                   // start 10-sample window for the rise
          phaseStartMs = millis();          // start per-phase timeout
          Serial.print(F("Saw drop... z=")); Serial.println(accZ, 2);
        }
        break;

      // 2) Confirm DOWN when a rise above DOWN_HIGH_TH occurs within 10 samples
      case WAIT_DOWN_HIGH:
        downWindow++;
        if (accZ > DOWN_HIGH_TH) {
          downConfirmedMs = millis();       // start planting timer
          pstate = WAIT_UP_HIGH;
          upWindow = 0;                     // we'll open the UP window after the spike
          phaseStartMs = millis();
          Serial.print(F("Saw secondary rise... z=")); Serial.println(accZ, 2);
        }
        else if (downWindow >= MAX_SAMPLES) {
          pstate = WAIT_DOWN_LOW;           // no rise within 10 samples -> reset
          Serial.println(F("No rise within 10 samples -> reset"));
        }
        else if (millis() - phaseStartMs > PLANT_TIMEOUT_MS) {
          pstate = WAIT_DOWN_LOW;           // safety reset
          Serial.println(F("Timeout in WAIT_DOWN_HIGH -> reset"));
        }
        break;

      case WAIT_UP_HIGH:
        // UP is now "rise-only": when we see a spike above UP_HIGH_TH, we evaluate the window.
        if (accZ > UP_HIGH_TH) {
          unsigned long dt = millis() - downConfirmedMs;   // time since DOWN confirmed
          Serial.print(F("UP rise seen, z=")); Serial.println(accZ, 2);
          Serial.print(F("dt(ms)=")); Serial.println((long)dt);

          if (dt >= PLANT_MIN_MS && dt <= PLANT_MAX_MS) {
            if (!isLedOn) {
              digitalWrite(LED_PIN, HIGH);
              isLedOn = true;
              ledOnTime = millis();
              buttonPressedDuringLED = false;
              Serial.println(F("LED ON (valid planting window)"));
            }
          } else {
            Serial.print(F("No LED: dt outside ["));
            Serial.print((long)PLANT_MIN_MS); Serial.print(F(",")); Serial.print((long)PLANT_MAX_MS); Serial.println(F("]"));
          }

          // After handling UP, reset for the next cycle
          pstate = WAIT_DOWN_LOW;
          Serial.println(F("FSM -> WAIT_DOWN_LOW"));
        }
        else if (millis() - downConfirmedMs > PLANT_TIMEOUT_MS) {
          // Overall safety timeout waiting for UP rise
          pstate = WAIT_DOWN_LOW;
          Serial.println(F("Timeout waiting for UP rise -> reset"));
        }
        break;
    }

    // --- Write continuous log row ---
    if (!logFile.open(LOG_FILE, O_WRITE | O_APPEND)) {
      Serial.println(F("ERROR: Failed to open log file."));
      return;
    }

    int plantStat = 0; // 0=idle, 1=auto (LED window), 2=button-confirmed
    if (isLedOn) plantStat = buttonPressedDuringLED ? 2 : 1;

    logFile.print(lastLogTime);
    logFile.print(','); logFile.print(accX, 4);
    logFile.print(','); logFile.print(accY, 4);
    logFile.print(','); logFile.print(accZ, 4);
    logFile.print(','); logFile.print(gyroX, 2);
    logFile.print(','); logFile.print(gyroY, 2);
    logFile.print(','); logFile.print(gyroZ, 2);

    if (gps.location.isValid()) {
      logFile.print(','); logFile.print(gps.location.lat(), 6);
      logFile.print(','); logFile.print(gps.location.lng(), 6);
      logFile.print(','); logFile.print(gps.altitude.meters());
      logFile.print(','); logFile.print(gps.satellites.value());
    } else {
      logFile.print(F(",0.0,0.0,0.0,0"));
    }

    logFile.print(','); logFile.print(plantStat);
    logFile.println();
    logFile.close();
  }
}

// =========================================================================
// Log a single detection event (called when LED turns off)
// =========================================================================
void writeDetection(int finalPlantStat) {
  if (!logFile.open(LOG_FILE2, O_WRITE | O_APPEND)) {
    Serial.println(F("ERROR: Failed to open detections file."));
    return;
  }
  logFile.print(millis());
  logFile.print(',');
  if (gps.location.isValid()) {
    logFile.print(gps.location.lat(), 6); logFile.print(',');
    logFile.print(gps.location.lng(), 6); logFile.print(',');
    logFile.print(gps.altitude.meters()); logFile.print(',');
    logFile.print(gps.satellites.value());
  } else {
    logFile.print(F("0.0,0.0,0.0,0"));
  }
  logFile.print(',');
  logFile.println(finalPlantStat);
  logFile.close();

  Serial.print(F("Detection logged: PlantStat=")); Serial.println(finalPlantStat);
}
