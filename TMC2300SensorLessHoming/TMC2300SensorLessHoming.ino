/*
 * TMC2300 ESP32-S3 Sensorless Homing with StallGuard4
 * 
 * Hardware Connections:
 * ESP32-S3 Pin 43 (TX) -> TMC2300 PDN_UART
 * ESP32-S3 Pin 44 (RX) -> TMC2300 PDN_UART (via 1k resistor)
 * ESP32-S3 Pin 14 -> TMC2300 STEP
 * ESP32-S3 Pin 21 -> TMC2300 DIR
 * ESP32-S3 Pin 12 -> TMC2300 DIAG (stall detection output)
 * ESP32-S3 Pin 13 -> TMC2300 EN (Active HIGH)
 * 
 * Commands via Serial Monitor:
 * h - Start homing sequence
 * + - Increase SGTHRS (more sensitive)
 * - - Decrease SGTHRS (less sensitive)
 * s - Show current settings and SG_RESULT
 * m - Manual move test (500 steps)
 * r - Reset position counter
 */

#define TX_PIN 43
#define RX_PIN 44
#define STEP_PIN 14
#define DIR_PIN 21
#define DIAG_PIN 12
#define EN_PIN 13

#define TMC_ADDR 0

// Register addresses
#define REG_GCONF      0x00
#define REG_GSTAT      0x01
#define REG_IFCNT      0x02
#define REG_IOIN       0x06
#define REG_IHOLD_IRUN 0x10
#define REG_TPOWERDOWN 0x11
#define REG_TSTEP      0x12
#define REG_TCOOLTHRS  0x14
#define REG_SGTHRS     0x40
#define REG_SG_RESULT  0x41
#define REG_COOLCONF   0x42
#define REG_CHOPCONF   0x6C
#define REG_DRV_STATUS 0x6F
#define REG_PWMCONF    0x70

// Configuration values
uint8_t SGTHRS_VALUE = 20;      // StallGuard threshold (0-255, higher = more sensitive)
uint16_t HOMING_SPEED = 2500;     // Homing speed in Hz (steps per second)
uint8_t IRUN_VALUE = 31;        // Run current (8-31, recommend 16-25 for homing)
uint8_t IHOLD_VALUE = 10;       // Hold current
uint32_t TCOOLTHRS_VALUE = 0xFFFFFFFF; // Always enable CoolStep/StallGuard

// Motor state
long motorPosition = 0;
bool motorEnabled = false;

HardwareSerial TMCSerial(1);

// CRC calculation
uint8_t calcCRC(uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for(uint8_t i = 0; i < len; i++) {
    uint8_t currentByte = data[i];
    for(uint8_t j = 0; j < 8; j++) {
      if((crc >> 7) ^ (currentByte & 0x01)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = (crc << 1);
      }
      currentByte >>= 1;
    }
  }
  return crc;
}

// Write register
void writeRegister(uint8_t address, uint32_t data) {
  uint8_t datagram[8];
  datagram[0] = 0x05;
  datagram[1] = TMC_ADDR;
  datagram[2] = address | 0x80;
  datagram[3] = (data >> 24) & 0xFF;
  datagram[4] = (data >> 16) & 0xFF;
  datagram[5] = (data >> 8) & 0xFF;
  datagram[6] = data & 0xFF;
  datagram[7] = calcCRC(datagram, 7);
  
  while(TMCSerial.available()) TMCSerial.read();
  TMCSerial.write(datagram, 8);
  TMCSerial.flush();
  
  // Discard echo
  unsigned long start = millis();
  int echoCount = 0;
  while(echoCount < 8 && (millis() - start) < 20) {
    if(TMCSerial.available()) {
      TMCSerial.read();
      echoCount++;
    }
  }
  delay(5);
}

// Read register
uint32_t readRegister(uint8_t address, bool silent = false) {
  uint8_t readRequest[4];
  readRequest[0] = 0x05;
  readRequest[1] = TMC_ADDR;
  readRequest[2] = address;
  readRequest[3] = calcCRC(readRequest, 3);
  
  while(TMCSerial.available()) TMCSerial.read();
  TMCSerial.write(readRequest, 4);
  TMCSerial.flush();
  
  // Discard echo
  delay(2);
  for(int i = 0; i < 4; i++) {
    unsigned long start = millis();
    while(!TMCSerial.available() && (millis() - start) < 10);
    if(TMCSerial.available()) TMCSerial.read();
  }
  
  // Read response
  unsigned long start = millis();
  while(TMCSerial.available() < 8 && (millis() - start) < 50) delay(1);
  
  if(TMCSerial.available() >= 8) {
    uint8_t response[8];
    for(int i = 0; i < 8; i++) {
      response[i] = TMCSerial.read();
    }
    
    if(response[0] == 0x05 && response[1] == 0xFF) {
      uint32_t data = ((uint32_t)response[3] << 24) |
                      ((uint32_t)response[4] << 16) |
                      ((uint32_t)response[5] << 8) |
                      response[6];
      return data;
    }
  }
  
  if(!silent) Serial.printf("Read failed for register 0x%02X\n", address);
  return 0xFFFFFFFF;
}

// Motor control functions
void enableMotor() {
  digitalWrite(EN_PIN, HIGH);
  motorEnabled = true;
  delay(10);
  Serial.println("✓ Motor ENABLED");
}

void disableMotor() {
  digitalWrite(EN_PIN, LOW);
  motorEnabled = false;
  Serial.println("✓ Motor DISABLED");
}

void step(int steps, int delayMicros) {
  if(!motorEnabled) {
    Serial.println("Motor not enabled!");
    return;
  }
  
  bool dir = steps > 0;
  digitalWrite(DIR_PIN, dir ? LOW : HIGH);
  delayMicroseconds(10);
  
  steps = abs(steps);
  
  for(int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayMicros);
    
    motorPosition += dir ? 1 : -1;
  }
}

// Initialize TMC2300 for StallGuard homing
void initTMC2300() {
  Serial.println("\n=== Initializing TMC2300 ===");
  
  // Read version
  uint32_t ioin = readRegister(REG_IOIN);
  uint8_t version = (ioin >> 24) & 0xFF;
  Serial.printf("Version: 0x%02X ", version);
  if(version == 0x40) {
    Serial.println("✓ (TMC2300 detected)");
  } else {
    Serial.println("✗ (Unexpected version!)");
  }
  
  // Configure GCONF
  // bit 4: diag_index = 0 (use DIAG for stall)
  // bit 3: shaft = 0 (normal direction)
  writeRegister(REG_GCONF, 0x00000000);
  Serial.println("✓ GCONF configured");
  
  // Set motor currents
  uint32_t ihold_irun = ((uint32_t)IHOLD_VALUE << 0) |
                        ((uint32_t)IRUN_VALUE << 8) |
                        (1 << 16); // IHOLDDELAY = 1
  writeRegister(REG_IHOLD_IRUN, ihold_irun);
  Serial.printf("✓ Motor current: IRUN=%d, IHOLD=%d\n", IRUN_VALUE, IHOLD_VALUE);
  
  // Set power down delay
  writeRegister(REG_TPOWERDOWN, 10);
  Serial.println("✓ TPOWERDOWN = 10");
  
  // Configure CHOPCONF
  // bit 28: intpol = 1 (256 microstep interpolation)
  // bit 27-24: mres = 0100 (16 microsteps)
  // bit 15-16: tbl = 01 (blanking time)
  // bit 0: enabledrv = 1
  uint32_t chopconf = (1UL << 28) | // intpol
                      (4UL << 24) | // mres = 16 microsteps
                      (1UL << 15) | // tbl
                      (1UL << 0);   // enabledrv
  writeRegister(REG_CHOPCONF, chopconf);
  Serial.println("✓ CHOPCONF configured (16 microsteps)");
  
  // Configure PWMCONF for StealthChop
  // Use defaults but ensure pwm_autoscale=1, pwm_autograd=1
  uint32_t pwmconf = 0xC40D1024; // Default from datasheet
  writeRegister(REG_PWMCONF, pwmconf);
  Serial.println("✓ PWMCONF configured (StealthChop)");
  
  // Set TCOOLTHRS - velocity threshold for enabling StallGuard
  writeRegister(REG_TCOOLTHRS, TCOOLTHRS_VALUE);
  Serial.printf("✓ TCOOLTHRS = 0x%08X (StallGuard always enabled)\n", TCOOLTHRS_VALUE);
  
  // Set StallGuard threshold
  writeRegister(REG_SGTHRS, SGTHRS_VALUE);
  Serial.printf("✓ SGTHRS = %d (stall threshold)\n", SGTHRS_VALUE);
  
  // Clear GSTAT flags
  writeRegister(REG_GSTAT, 0x07);
  
  Serial.println("=== Initialization Complete ===\n");
  delay(100);
}

// Show current settings and values
void showStatus() {
  Serial.println("\n=== Current Status ===");
  Serial.printf("Motor Position: %ld steps\n", motorPosition);
  Serial.printf("Motor State: %s\n", motorEnabled ? "ENABLED" : "DISABLED");
  Serial.printf("SGTHRS: %d (Stall threshold)\n", SGTHRS_VALUE);
  Serial.printf("IRUN: %d, IHOLD: %d\n", IRUN_VALUE, IHOLD_VALUE);
  Serial.printf("Homing Speed: %d Hz\n", HOMING_SPEED);
  
  // Read TSTEP
  uint32_t tstep = readRegister(REG_TSTEP, true);
  Serial.printf("TSTEP: 0x%08X ", tstep);
  if(tstep == 0xFFFFF) {
    Serial.println("(standstill)");
  } else {
    Serial.printf("(velocity: ~%lu Hz)\n", 12000000UL / (tstep * 256));
  }
  
  // Read SG_RESULT
  uint32_t sg_result = readRegister(REG_SG_RESULT, true);
  Serial.printf("SG_RESULT: %d ", sg_result & 0x3FF);
  if(sg_result > SGTHRS_VALUE * 2) {
    Serial.println("(LOW LOAD)");
  } else {
    Serial.println("(HIGH LOAD / STALL)");
  }
  
  // Read DIAG pin
  bool diagState = digitalRead(DIAG_PIN);
  Serial.printf("DIAG Pin: %s\n", diagState ? "HIGH (STALL!)" : "LOW");
  
  // Read DRV_STATUS
  uint32_t drv_status = readRegister(REG_DRV_STATUS, true);
  bool stst = (drv_status >> 31) & 0x01;
  uint8_t cs_actual = (drv_status >> 16) & 0x1F;
  Serial.printf("DRV_STATUS: stst=%d, cs_actual=%d\n", stst, cs_actual);
  
  Serial.println("==================\n");
}

// Perform sensorless homing
bool performHoming() {
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║   STARTING SENSORLESS HOMING      ║");
  Serial.println("╚════════════════════════════════════╝");
  
  if(!motorEnabled) {
    enableMotor();
    delay(200); // Allow StealthChop auto-tuning
  }
  
  // Clear any stall flags
  writeRegister(REG_GSTAT, 0x07);
  digitalWrite(DIR_PIN, LOW); // Set homing direction
  delay(10);
  
  Serial.printf("\nSettings:\n");
  Serial.printf("  SGTHRS: %d\n", SGTHRS_VALUE);
  Serial.printf("  Speed: %d Hz\n", HOMING_SPEED);
  Serial.printf("  Step delay: %d µs\n", 1000000 / HOMING_SPEED);
  Serial.println("\nMoving until stall detected...\n");
  
  int stepDelay = (1000000 / HOMING_SPEED) - 5; // Subtract pulse width
  int stepCount = 0;
  int maxSteps = 10000; // Safety limit
  
  unsigned long lastUpdate = 0;
  bool stallDetected = false;
  
  // Move until stall detected
  while(stepCount < maxSteps && !stallDetected) {
    // Check DIAG pin (hardware stall detection)
    if(digitalRead(DIAG_PIN) == HIGH) {
      Serial.println("\n✓ STALL DETECTED via DIAG pin!");
      stallDetected = true;
      break;
    }
    
    // Take a step
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
    
    stepCount++;
    motorPosition++;
    
    // Print status every 50 steps
    if(millis() - lastUpdate > 100) {
      uint32_t sg_result = readRegister(REG_SG_RESULT, true);
      uint32_t tstep = readRegister(REG_TSTEP, true);
      
      Serial.printf("Step: %4d | SG_RESULT: %3d | TSTEP: 0x%05X | DIAG: %d",
                    stepCount, 
                    sg_result & 0x3FF,
                    tstep & 0xFFFFF,
                    digitalRead(DIAG_PIN));
      
      // Show load indication
      int threshold = SGTHRS_VALUE * 2;
      if((sg_result & 0x3FF) <= threshold) {
        Serial.print(" ← HIGH LOAD");
      }
      Serial.println();
      
      lastUpdate = millis();
    }
    
    // Safety check - also check software stall detection
    /*
    if(stepCount % 10 == 0) {
      uint32_t sg_result = readRegister(REG_SG_RESULT, true);
      if((sg_result & 0x3FF) <= SGTHRS_VALUE * 2) {
        // Confirm stall with multiple readings
        delay(5);
        uint32_t sg_confirm = readRegister(REG_SG_RESULT, true);
        if((sg_confirm & 0x3FF) <= SGTHRS_VALUE * 2) {
          Serial.println("\n✓ STALL DETECTED via SG_RESULT!");
          Serial.println((sg_confirm & 0x3FF));
          stallDetected = true;
          break;
        }
      }
    }*/
  }
  
  if(!stallDetected) {
    Serial.println("\n✗ Homing failed - max steps reached!");
    Serial.println("Try: Increase SGTHRS for more sensitivity");
    return false;
  }
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.printf("║  HOMING COMPLETE! (%d steps)     ║\n", stepCount);
  Serial.println("╚════════════════════════════════════╝");
  
  // Back off a bit
  Serial.println("\nBacking off 100 steps...");
  digitalWrite(DIR_PIN, HIGH);
  delay(10);
  step(-100, stepDelay * 2);
  
  // Reset position
  motorPosition = 0;
  Serial.println("✓ Position reset to 0");
  
  delay(100);
  showStatus();
  
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║  TMC2300 Sensorless Homing (StallGuard4)  ║");
  Serial.println("╚════════════════════════════════════════════╝");
  
  // Initialize pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
  
  // Initialize UART
  TMCSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);
  
  // Initialize TMC2300
  initTMC2300();
  
  // Print commands
  Serial.println("\n=== Commands ===");
  Serial.println("h - Start homing");
  Serial.println("+ - Increase SGTHRS (more sensitive)");
  Serial.println("- - Decrease SGTHRS (less sensitive)");
  Serial.println("s - Show current status");
  Serial.println("m - Manual move test (500 steps)");
  Serial.println("r - Reset position to 0");
  Serial.println("e - Toggle motor enable");
  Serial.println("================\n");
  
  showStatus();
}

void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'h':
      case 'H':
        performHoming();
        break;
        
      case '+':
        SGTHRS_VALUE = min(255, SGTHRS_VALUE + 5);
        writeRegister(REG_SGTHRS, SGTHRS_VALUE);
        Serial.printf("\n✓ SGTHRS increased to %d (more sensitive)\n\n", SGTHRS_VALUE);
        showStatus();
        break;
        
      case '-':
        SGTHRS_VALUE = max(0, SGTHRS_VALUE - 5);
        writeRegister(REG_SGTHRS, SGTHRS_VALUE);
        Serial.printf("\n✓ SGTHRS decreased to %d (less sensitive)\n\n", SGTHRS_VALUE);
        showStatus();
        break;
        
      case 's':
      case 'S':
        showStatus();
        break;
        
      case 'm':
      case 'M':
        Serial.println("\nManual move test: 500 steps forward");
        if(!motorEnabled) enableMotor();
        step(500, 1000);
        Serial.printf("Position: %ld\n\n", motorPosition);
        break;
        
      case 'r':
      case 'R':
        motorPosition = 0;
        Serial.println("\n✓ Position reset to 0\n");
        break;
        
      case 'e':
      case 'E':
        if(motorEnabled) {
          disableMotor();
        } else {
          enableMotor();
        }
        Serial.println();
        break;
    }
  }
  
  delay(10);
}