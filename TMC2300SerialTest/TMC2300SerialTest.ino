/*
 * TMC2300 ESP32-S3 UART Communication Test
 * 
 * Hardware Connections:
 * ESP32-S3 Pin 43 (TX) -> TMC2300 PDN_UART
 * ESP32-S3 Pin 44 (RX) -> TMC2300 PDN_UART (via 1k resistor)
 * ESP32-S3 Pin 14 -> TMC2300 STEP
 * ESP32-S3 Pin 21 -> TMC2300 DIR
 * ESP32-S3 Pin 12 -> TMC2300 DIAG
 * 
 * TMC2300 Configuration:
 * MS1_AD0 and MS2_AD1 -> Set to GND or VIO for UART address (default 0)
 * STEPPER -> VIO
 * MODE -> GND (for UART mode)
 */

#define TX_PIN 43
#define RX_PIN 44
#define STEP_PIN 14
#define DIR_PIN 21
#define DIAG_PIN 12

// TMC2300 UART address (0-3 based on MS1_AD0 and MS2_AD1)
#define TMC_ADDR 0

// Register addresses
#define REG_GCONF    0x00
#define REG_GSTAT    0x01
#define REG_IFCNT    0x02
#define REG_IOIN     0x06
#define REG_IHOLD_IRUN 0x10

// Use Serial1 for TMC2300
HardwareSerial TMCSerial(1);

// CRC calculation for TMC UART
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

// Write register to TMC2300
void writeRegister(uint8_t address, uint32_t data) {
  uint8_t datagram[8];
  
  // Sync and reserved nibble
  datagram[0] = 0x05;
  // Slave address
  datagram[1] = TMC_ADDR;
  // Register address (add 0x80 for write)
  datagram[2] = address | 0x80;
  // Data (MSB first)
  datagram[3] = (data >> 24) & 0xFF;
  datagram[4] = (data >> 16) & 0xFF;
  datagram[5] = (data >> 8) & 0xFF;
  datagram[6] = data & 0xFF;
  // CRC
  datagram[7] = calcCRC(datagram, 7);
  
  Serial.println("\n--- Write Register Debug ---");
  Serial.printf("Reg Addr: 0x%02X (with write bit: 0x%02X)\n", address, datagram[2]);
  Serial.printf("Data: 0x%08X\n", data);
  Serial.print("TX -> ");
  for(int i = 0; i < 8; i++) {
    Serial.printf("%02X ", datagram[i]);
  }
  Serial.println();
  
  // Clear RX buffer before sending
  int cleared = 0;
  while(TMCSerial.available()) {
    TMCSerial.read();
    cleared++;
  }
  if(cleared > 0) {
    Serial.printf("Cleared %d old bytes from RX buffer\n", cleared);
  }
  
  // Send datagram
  TMCSerial.write(datagram, 8);
  TMCSerial.flush();
  
  // **CRITICAL: Discard ECHO of our transmitted bytes**
  delay(2); // Short delay for echo
  int echoBytes = 0;
  unsigned long echoStart = millis();
  while(echoBytes < 8 && (millis() - echoStart) < 20) {
    if(TMCSerial.available()) {
      uint8_t echoByte = TMCSerial.read();
      Serial.printf("Echo[%d]: 0x%02X ", echoBytes, echoByte);
      if(echoByte == datagram[echoBytes]) {
        Serial.print("✓");
      } else {
        Serial.printf("✗ (expected 0x%02X)", datagram[echoBytes]);
      }
      if(echoBytes == 7) Serial.println(); else Serial.print(", ");
      echoBytes++;
    }
  }
  
  if(echoBytes == 8) {
    Serial.println("✓ All 8 echo bytes discarded");
  } else {
    Serial.printf("WARNING: Only discarded %d echo bytes (expected 8)\n", echoBytes);
  }
  
  Serial.println("--- Write Complete ---\n");
  
  delay(5); // Allow TMC2300 to process
}

// Read register from TMC2300
uint32_t readRegister(uint8_t address) {
  uint8_t readRequest[4];
  
  // Sync and reserved nibble
  readRequest[0] = 0x05;
  // Slave address
  readRequest[1] = TMC_ADDR;
  // Register address (no 0x80 for read)
  readRequest[2] = address;
  // CRC
  readRequest[3] = calcCRC(readRequest, 3);
  
  Serial.println("\n--- Read Register Debug ---");
  Serial.printf("Reg Addr: 0x%02X\n", address);
  
  // Clear RX buffer
  int cleared = 0;
  while(TMCSerial.available()) {
    TMCSerial.read();
    cleared++;
  }
  if(cleared > 0) {
    Serial.printf("Cleared %d old bytes from RX buffer\n", cleared);
  }
  
  // Send read request
  Serial.print("TX -> ");
  for(int i = 0; i < 4; i++) {
    Serial.printf("%02X ", readRequest[i]);
  }
  Serial.println();
  
  TMCSerial.write(readRequest, 4);
  TMCSerial.flush();
  
  // **CRITICAL: Discard ECHO of our transmitted bytes**
  // When TX and RX are coupled, we receive our own transmission
  delay(2); // Short delay for echo
  int echoBytes = 0;
  unsigned long echoStart = millis();
  while(echoBytes < 4 && (millis() - echoStart) < 10) {
    if(TMCSerial.available()) {
      uint8_t echoByte = TMCSerial.read();
      Serial.printf("Echo[%d]: 0x%02X ", echoBytes, echoByte);
      if(echoByte == readRequest[echoBytes]) {
        Serial.println("✓ (matches TX)");
      } else {
        Serial.printf("✗ (expected 0x%02X)\n", readRequest[echoBytes]);
      }
      echoBytes++;
    }
  }
  
  if(echoBytes < 4) {
    Serial.printf("WARNING: Only received %d echo bytes (expected 4)\n", echoBytes);
  }
  
  // Now wait for actual response (8 bytes from TMC2300)
  Serial.println("Waiting for TMC response...");
  unsigned long responseStart = millis();
  while(TMCSerial.available() < 8 && (millis() - responseStart) < 50) {
    delay(1);
  }
  
  int available = TMCSerial.available();
  Serial.printf("Available bytes: %d\n", available);
  
  if(available >= 8) {
    uint8_t response[12]; // Extra space in case of extra bytes
    int bytesRead = 0;
    
    // Read all available bytes
    while(TMCSerial.available() && bytesRead < 12) {
      response[bytesRead++] = TMCSerial.read();
    }
    
    Serial.print("RX <- ");
    for(int i = 0; i < bytesRead; i++) {
      Serial.printf("%02X ", response[i]);
    }
    Serial.println();
    
    // Look for valid response pattern (0x05 0xFF ...)
    int responseStart = -1;
    for(int i = 0; i <= bytesRead - 8; i++) {
      if(response[i] == 0x05 && response[i+1] == 0xFF) {
        responseStart = i;
        Serial.printf("Found valid response at byte %d\n", i);
        break;
      }
    }
    
    if(responseStart >= 0) {
      // Verify CRC
      uint8_t receivedCRC = response[responseStart + 7];
      uint8_t calculatedCRC = calcCRC(&response[responseStart], 7);
      
      Serial.printf("CRC Check: RX=0x%02X, Calc=0x%02X ", receivedCRC, calculatedCRC);
      if(receivedCRC == calculatedCRC) {
        Serial.println("✓");
      } else {
        Serial.println("✗ CRC MISMATCH!");
      }
      
      // Extract data (bytes 3-6 of response)
      uint32_t data = ((uint32_t)response[responseStart + 3] << 24) |
                      ((uint32_t)response[responseStart + 4] << 16) |
                      ((uint32_t)response[responseStart + 5] << 8) |
                      response[responseStart + 6];
      
      Serial.printf("Data: 0x%08X\n", data);
      Serial.println("--- Read Complete ---\n");
      return data;
    } else {
      Serial.println("✗ No valid response header found (0x05 0xFF)");
      Serial.println("--- Read Failed ---\n");
    }
  } else {
    Serial.printf("✗ Timeout: Only %d bytes received (need 8)\n", available);
    if(available > 0) {
      Serial.print("Received: ");
      while(TMCSerial.available()) {
        Serial.printf("%02X ", TMCSerial.read());
      }
      Serial.println();
    }
    Serial.println("--- Read Failed ---\n");
  }
  
  return 0xFFFFFFFF; // Error value
}

void setup() {
  // Initialize USB Serial for debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nTMC2300 UART Test");
  Serial.println("==================");
  
  // Initialize pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT);
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  // Initialize TMC UART
  // 115200 baud, 8N1 format
  TMCSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);
  
  Serial.println("\n1. Testing UART Communication...");
  
  // Read IOIN register (should always work)
  Serial.println("Reading IOIN register (0x06):");
  uint32_t ioin = readRegister(REG_IOIN);
  if(ioin != 0xFFFFFFFF) {
    Serial.printf("IOIN: 0x%08X\n", ioin);
    Serial.printf("  VERSION: 0x%02X (should be 0x40)\n", (ioin >> 24) & 0xFF);
    Serial.printf("  EN: %d\n", (ioin >> 0) & 0x01);
    Serial.printf("  STEP: %d\n", (ioin >> 8) & 0x01);
    Serial.printf("  DIR: %d\n", (ioin >> 9) & 0x01);
    Serial.println("✓ UART Communication OK!");
  } else {
    Serial.println("✗ UART Communication FAILED!");
    Serial.println("\nTroubleshooting:");
    Serial.println("1. Check wiring (TX->PDN_UART, RX->PDN_UART via 1k)");
    Serial.println("2. Verify STEPPER pin = VIO");
    Serial.println("3. Verify MODE pin = GND");
    Serial.println("4. Check MS1_AD0 and MS2_AD1 for address setting");
  }
  
  delay(500);
  
  // Read IFCNT (interface counter)
  Serial.println("\n2. Reading IFCNT (Interface Counter):");
  uint32_t ifcnt1 = readRegister(REG_IFCNT);
  Serial.printf("IFCNT: %d\n", ifcnt1 & 0xFF);
  
  delay(100);
  
  // Write a test value to IHOLD_IRUN
  Serial.println("\n3. Writing to IHOLD_IRUN register:");
  uint32_t ihold_irun = (16 << 0) |   // IHOLD = 16
                        (31 << 8) |   // IRUN = 31
                        (1 << 16);    // IHOLDDELAY = 1
  writeRegister(REG_IHOLD_IRUN, ihold_irun);
  Serial.printf("Wrote: 0x%08X\n", ihold_irun);
  
  delay(100);
  
  // Read IFCNT again (should increment)
  Serial.println("\n4. Reading IFCNT again:");
  uint32_t ifcnt2 = readRegister(REG_IFCNT);
  Serial.printf("IFCNT: %d\n", ifcnt2 & 0xFF);
  if((ifcnt2 & 0xFF) > (ifcnt1 & 0xFF)) {
    Serial.println("✓ Write successful! (IFCNT incremented)");
  } else {
    Serial.println("✗ Write may have failed (IFCNT not incremented)");
  }
  
  Serial.println("\n5. Test Complete!");
  Serial.println("====================\n");
}

void loop() {
  // Simple test: Read IOIN every 2 seconds and show pin states
  static unsigned long lastRead = 0;
  
  if(millis() - lastRead > 2000) {
    lastRead = millis();
    
    Serial.println("Reading IOIN register:");
    uint32_t ioin = readRegister(REG_IOIN);
    
    if(ioin != 0xFFFFFFFF) {
      Serial.printf("IOIN: 0x%08X\n", ioin);
      Serial.printf("  EN: %d, STEP: %d, DIR: %d, DIAG: %d\n",
                    (ioin >> 0) & 0x01,
                    (ioin >> 8) & 0x01,
                    (ioin >> 9) & 0x01,
                    (ioin >> 4) & 0x01);
    }
    
    // Toggle DIR pin for testing
    static bool dirState = false;
    dirState = !dirState;
    digitalWrite(DIR_PIN, dirState);
    
    Serial.println();
  }
  
  delay(10);
}