/*
 * Hardware & Wiring:
 * - SD Card (SPI): MOSI-11, MISO-12, SCK-13, CS-10
 * - MPU-6050 (I2C): SDA-A4, SCL-A5
 * - GPS Module (Software Serial):
 * - GPS TX -> Nano Pin D4 (RX)
 * - GPS RX -> Nano Pin D5 (TX)
 * - LED: Pin D3 -> Resistor (220-330 Ohm) -> LED -> GND
 */

#include <SdFat.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// --- Configuration ---
const uint8_t CHIP_SELECT = 10;
#define LOG_FILE "datalog.csv"
const int MPU_ADDR = 0x68;
const int LED_PIN = 3; // LED for high-G alerts

// GPS Software Serial pins
static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600; // GPS modules default to 9600 baud

// Scale factors for MPU-6050
const float ACCEL_SCALE_FACTOR = 16384.0;
const float GYRO_SCALE_FACTOR = 131.0;

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
const unsigned int LOG_INTERVAL_MS = 100; // Log at 10 Hz to give GPS time to update

// LED Alert variables
bool isLedOn = false;
unsigned long ledOnTime = 0;
const unsigned int LED_ON_DURATION_MS = 5000; // Keep LED on for 3 seconds

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  // delete before making final code
  while (!Serial) { ; }
  
  pinMode(LED_PIN, OUTPUT); // Set LED pin as an output
  digitalWrite(LED_PIN, LOW); // Ensure LED is off at start

  Serial.println(F("--- Data Logger Initializing (Step 4 with LED) ---"));

  // --- Initialize Software Serial for GPS ---
  ss.begin(GPSBaud);
  Serial.println(F("Software Serial for GPS started."));

  // --- Initialize MPU-6050 ---
  Serial.println(F("Initializing MPU-6050..."));
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.println(F("MPU-6050 is awake."));

  // --- Initialize SD Card ---
  Serial.println(F("Initializing SD card..."));
  if (!sd.begin(CHIP_SELECT, SD_SCK_MHZ(4))) {
    Serial.println(F("**************************************"));
    Serial.println(F("SD Card initialization FAILED."));
    Serial.println(F("**************************************"));
    while (true){
      digitalWrite(LED_PIN, HIGH);
      delay(1000); // Wait for 1 second
      // Turn LED off
      digitalWrite(LED_PIN, LOW);
      delay(1000); // Wait for 1 second
    }
  }
  Serial.println(F("SD card initialized successfully."));

  // --- Prepare Log File ---
  if (!sd.exists(LOG_FILE)) {
    Serial.println(F("Creating new log file..."));
    if (!logFile.open(LOG_FILE, O_WRITE | O_CREAT)) {
      Serial.println(F("CRITICAL ERROR: Failed to create log file."));
      while (true);
    }
    // New header row including GPS data
    logFile.println("Timestamp,AccX_G,AccY_G,AccZ_G,GyroX_DPS,GyroY_DPS,GyroZ_DPS,Lat,Lon,Altitude,Sats");
    logFile.close();
    Serial.println(F("File created and header written."));
  } else {
    Serial.println(F("Log file exists. Appending data."));
  }

  Serial.println(F("--- Initialization Complete ---"));
  Serial.println(F("Waiting for GPS lock... (This can take several minutes)"));
  lastLogTime = millis();
}

// =========================================================================
// LOOP
// =========================================================================
void loop() {
  // --- Handle LED timeout (non-blocking) ---
  // Check if the LED is currently on and if its 3-second duration has passed.
  if (isLedOn && (millis() - ledOnTime >= LED_ON_DURATION_MS)) {
    digitalWrite(LED_PIN, LOW); // Turn the LED off
    isLedOn = false;            // Reset the flag
  }

  // This part of the loop runs continuously, feeding serial data from the
  // GPS module to the TinyGPS++ object. This must be done as often as possible.
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // This part of the loop runs at our desired logging frequency.
  if (millis() - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = millis();

    // --- 1. Read MPU Sensor Data & Calculate ---
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    rawAccX = Wire.read() << 8 | Wire.read();
    rawAccY = Wire.read() << 8 | Wire.read();
    rawAccZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    rawGyroX = Wire.read() << 8 | Wire.read();
    rawGyroY = Wire.read() << 8 | Wire.read();
    rawGyroZ = Wire.read() << 8 | Wire.read();

    accX = rawAccX / ACCEL_SCALE_FACTOR;
    accY = rawAccY / ACCEL_SCALE_FACTOR;
    accZ = rawAccZ / ACCEL_SCALE_FACTOR;
    gyroX = rawGyroX / GYRO_SCALE_FACTOR;
    gyroY = rawGyroY / GYRO_SCALE_FACTOR;
    gyroZ = rawGyroZ / GYRO_SCALE_FACTOR;

    // --- NEW: Check for LED Alert Trigger ---
    // If z-axis acceleration exceeds 2 G's and the LED isn't already on.
    if (accZ > 1.5 && !isLedOn) {
      digitalWrite(LED_PIN, HIGH); // Turn the LED on
      isLedOn = true;              // Set the flag
      ledOnTime = millis();        // Record the time it was turned on
    }
    
    // --- 2. Open Log File ---
    if (!logFile.open(LOG_FILE, O_WRITE | O_APPEND)) {
      Serial.println(F("ERROR: Failed to open log file."));
      return;
    }

    // --- 3. Write Data to File ---
    logFile.print(lastLogTime);
    logFile.print(','); logFile.print(accX, 4);
    logFile.print(','); logFile.print(accY, 4);
    logFile.print(','); logFile.print(accZ, 4);
    logFile.print(','); logFile.print(gyroX, 2);
    logFile.print(','); logFile.print(gyroY, 2);
    logFile.print(','); logFile.print(gyroZ, 2);

    // Check if we have a valid GPS location fix before logging GPS data
    if (gps.location.isValid()) {
      logFile.print(','); logFile.print(gps.location.lat(), 6); // 6 decimal places for precision
      logFile.print(','); logFile.print(gps.location.lng(), 6);
      logFile.print(','); logFile.print(gps.altitude.meters());
      logFile.print(','); logFile.print(gps.satellites.value());
    } else {
      // If no fix, log placeholder values to keep the CSV columns aligned
      logFile.print(F(",0.0,0.0,0.0,0"));
    }
    logFile.println(); // Finish the line

    // --- 4. Close File ---
    logFile.close();

    // --- 5. Provide Serial Feedback ---
    Serial.print(F("Acc(G):")); Serial.print(accZ, 2);
    Serial.print(F(" | Sats: ")); Serial.print(gps.satellites.value());
    Serial.print(F(" | Lat: ")); Serial.print(gps.location.lat(), 4);
    Serial.print(F(" | Lon: ")); Serial.println(gps.location.lng(), 4);
  }
}

