#include <Wire.h>
#include "SparkFun_VL53L1X.h"

const uint8_t NUM_ToF_SENSORS = 4;
uint8_t XSHUT_PINS[NUM_ToF_SENSORS] = {2, 3, 4, 5};   // XSHUT pins for each sensor
uint8_t NEW_ADDRS[NUM_ToF_SENSORS] = {0x2A, 0x2B, 0x2C, 0x2D};    // New I2C addresses to assign 
int distances_mm[NUM_ToF_SENSORS];      // For collecting last readings (mm). If a read fails, value = -1.
SFEVL53L1X tof[NUM_ToF_SENSORS];

const uint8_t STATUS_LED_PIN = LED_BUILTIN; 

// Cycle rates (ms)
const int Tof_Sensor_Period = 50;

unsigned long lastReport = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait on native USB boards */ }

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Wire.begin(); 
  
  tofInit()
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void loop() {
  handleInboundCommands();

  unsigned long now = millis();
  if (now - lastReport >= Tof_Sensor_Period) {
    readAllSensors();
    printCsvLine();
    lastReport = now;
  }
}

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void tofInit(){

  // Prepare XSHUT pins
  for (uint8_t i = 0; i < NUM_ToF_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW); // hold all sensors in reset
  }
  delay(10);

    // Bring up sensors one-by-one and assign new addresses
  for (uint8_t i = 0; i < NUM_ToF_SENSORS; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(2); // allow boot

    if (tof[i].begin() != 0) {
      Serial.print("ERR: begin() failed on sensor ");
      Serial.println(i);
      distances_mm[i] = -1;       // mark as invalid
    } else {
      if (!tof[i].setI2CAddress(NEW_ADDRS[i])) {
        Serial.print("ERR: setI2CAddress failed on sensor ");
        Serial.println(i);
      } else {
        tof[i].setI2CAddress(NEW_ADDRS[i]);
        // tof[i].setTimingBudgetInMs(20);
      }
    }
  }

  Serial.println("ToF sensors online");
}

void readAllSensors() {
  for (uint8_t i = 0; i < NUM_ToF_SENSORS; i++) {
    int current_distance = -1;

    if (tof[i].startRanging() == 0) {
      uint16_t guard = 0;
      while (!tof[i].checkForDataReady() && guard < 1000) {
        delayMicroseconds(200);
        guard++;
      }

      if (guard < 1000) {
        int d = tof[i].getDistance();
        current_distance = (d >= 0) ? d : -1;
        tof[i].clearInterrupt();
      }
      tof[i].stopRanging();
    }

    distances_mm[i] = current_distancem;
  }
}

void printCsvLine() {
  // Format: VL53,NUM_ToF_SENSORS,mm1,mm2,mm3,mm4\n
  Serial.print("VL53,");
  Serial.print(NUM_ToF_SENSORS);
  for (uint8_t i = 0; i < NUM_ToF_SENSORS; i++) {
    Serial.print(",");
    Serial.print(distances_mm[i]); 
  }
  Serial.print("\n");
}

void handleInboundCommands() {
  static String line;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        processCommandLine(line);
        line = "";
      }
    } else {
      line += c;
      if (line.length() > 120) { line = "";}
    }
  }
}

void processCommandLine(const String& cmd) {
  if (cmd.startsWith("PING")) {
    Serial.println("PONG");
    return;
  }

  if (cmd.startsWith("CMD")) {
    int idx = cmd.indexOf("LED=");
    if (idx >= 0 && STATUS_LED_PIN != 255) {
      int val = cmd.substring(idx + 4).toInt();
      digitalWrite(STATUS_LED_PIN, val ? HIGH : LOW);
    }
    // add actuator controls here (relays, valves, etc.)
  }
}
