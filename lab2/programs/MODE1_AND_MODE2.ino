#include "LSM6DS3.h"
#include "Wire.h"
#include <Arduino.h>
#include <U8x8lib.h>
#include <PCF8563.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoBLE.h>

// ================= BLE (same UUID style as LED test) =================
BLEService loggerService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic modeCharacteristic(
  "19B10001-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLEWrite
);

// ================= Hardware =================
const int ledPin = LED_BUILTIN;

// ⚠️ CHANGE THIS if your SD CS is wired to a different pin
const int chipSelect = D2;

PCF8563 pcf;
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(PIN_WIRE_SCL, PIN_WIRE_SDA, U8X8_PIN_NONE);

// IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A);

File dataFile;

// ================= Mode + Sampling =================
enum Mode { MODE1_SAMPLE_ONLY = 1, MODE2_SAMPLE_AND_LOG = 2 };
Mode currentMode = MODE1_SAMPLE_ONLY;
Mode lastMode    = MODE1_SAMPLE_ONLY;

const int FS_HZ = 100;                       // 100 Hz
const unsigned long PERIOD_MS = 1000 / FS_HZ;
unsigned long lastSampleMs = 0;

// ---------------- Temperature helper ----------------
// If this gives compile error, replace with: return myIMU.readTemp();
float readIMUTempC() {
  return myIMU.readTempC();
}

// ---------------- OLED ----------------
void showModeOnOLED() {
  u8x8.clear();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  u8x8.setCursor(0, 0);
  u8x8.print("MODE ");
  u8x8.print((currentMode == MODE1_SAMPLE_ONLY) ? "1" : "2");

  u8x8.setCursor(0, 2);
  u8x8.print("FS=");
  u8x8.print(FS_HZ);
  u8x8.print("Hz");
}

// ---------------- SD logging ----------------
void startNewLogFile() {
  char filename[13];
  for (int i = 0; i < 1000; i++) {
    sprintf(filename, "LOG%03d.CSV", i);
    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      break;
    }
  }

  if (dataFile) {
    dataFile.println("hr,min,sec,ax,ay,az,gx,gy,gz,temp");
    dataFile.flush();
  } else {
    Serial.println("ERROR: Could not open log file!");
  }
}

void stopLogFile() {
  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // OLED
  u8x8.begin();
  u8x8.setFlipMode(1);

  // I2C + RTC
  Wire.begin();
  pcf.init();
  pcf.startClock();

  // IMU
  if (myIMU.begin() != 0) {
    Serial.println("IMU error");
  } else {
    Serial.println("IMU OK");
  }

  // SD
  if (!SD.begin(chipSelect)) {
    Serial.println("SD init failed (check CS pin + wiring + 3.3V)");
    while (1);
  }
  Serial.println("SD OK");

  // BLE
  if (!BLE.begin()) {
    Serial.println("BLE start failed");
    while (1);
  }

  BLE.setLocalName("Lab3_XIAO");
  BLE.setAdvertisedService(loggerService);

  loggerService.addCharacteristic(modeCharacteristic);
  BLE.addService(loggerService);

  modeCharacteristic.writeValue(0);     // default: Mode 1
  BLE.advertise();

  Serial.println("Advertising: write 0=Mode1, 1=Mode2");

  currentMode = MODE1_SAMPLE_ONLY;
  lastMode = currentMode;
  showModeOnOLED();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());

    while (central.connected()) {

      // -------- BLE write controls the mode --------
      if (modeCharacteristic.written()) {
        byte v = modeCharacteristic.value();
        Serial.print("RAW BLE value = ");
        Serial.println(v);

        // safest mapping: only value 1 => Mode2, everything else => Mode1
        currentMode = (v == 1) ? MODE2_SAMPLE_AND_LOG : MODE1_SAMPLE_ONLY;

        if (currentMode != lastMode) {
          if (lastMode == MODE2_SAMPLE_AND_LOG) {
            stopLogFile();
            Serial.println("SD logging STOPPED");
          }

          if (currentMode == MODE2_SAMPLE_AND_LOG) {
            startNewLogFile();
            Serial.println("SD logging STARTED");
          }

          showModeOnOLED();

          Serial.print("MODE changed to: ");
          Serial.println((currentMode == MODE1_SAMPLE_ONLY) ? "MODE1" : "MODE2");

          lastMode = currentMode;
        }
      }

      // -------- fixed sampling at 100 Hz --------
      unsigned long nowMs = millis();
      if (nowMs - lastSampleMs >= PERIOD_MS) {
        lastSampleMs += PERIOD_MS;

        Time t = pcf.getTime();

        float ax = myIMU.readFloatAccelX();
        float ay = myIMU.readFloatAccelY();
        float az = myIMU.readFloatAccelZ();
        float gx = myIMU.readFloatGyroX();
        float gy = myIMU.readFloatGyroY();
        float gz = myIMU.readFloatGyroZ();
        float tempC = readIMUTempC();

        // LED indicator (ON in Mode2)
        digitalWrite(ledPin, (currentMode == MODE2_SAMPLE_AND_LOG) ? HIGH : LOW);

        // Mode 1: sample only (no SD write)
        // Mode 2: sample + log
        if (currentMode == MODE2_SAMPLE_AND_LOG && dataFile) {
          dataFile.print(t.hour);   dataFile.print(",");
          dataFile.print(t.minute); dataFile.print(",");
          dataFile.print(t.second); dataFile.print(",");
          dataFile.print(ax, 6);    dataFile.print(",");
          dataFile.print(ay, 6);    dataFile.print(",");
          dataFile.print(az, 6);    dataFile.print(",");
          dataFile.print(gx, 6);    dataFile.print(",");
          dataFile.print(gy, 6);    dataFile.print(",");
          dataFile.print(gz, 6);    dataFile.print(",");
          dataFile.println(tempC, 2);

          // flush occasionally (reduce SD overhead)
          static int flushCount = 0;
          if (++flushCount >= 50) {   // every ~0.5s
            dataFile.flush();
            flushCount = 0;
          }

          // Serial message once per second (not spam)
          static unsigned long lastMsg = 0;
          if (millis() - lastMsg >= 1000) {
            Serial.println("Mode2: logging to SD...");
            lastMsg = millis();
          }
        }
      }
    }

    Serial.print("Disconnected: ");
    Serial.println(central.address());

    stopLogFile();
    digitalWrite(ledPin, LOW);
  }
}
