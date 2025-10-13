#include <Wire.h>

// Set this to 1 ONLY when you are absolutely sure you want to permanently
// burn the settings into the sensor's OTP memory.
#define BURN_ENABLED 1

// AS5600 I2C Address
const int AS5600_ADDR = 0x36;

// Register Addresses
const int CONF_REG_H = 0x07;
const int CONF_REG_L = 0x08;
const int BURN_REG   = 0xFF;

// Configuration bytes based on the example settings above
// CONF_L (0x08): PWMF=11, OUTS=10, HYST=01, PM=00   -> 0b11100100 -> 0xE4
// CONF_H (0x07):  WD=0, FTH=000, SF=00-> 0b00000000 -> 0x00
const byte config_l = 0xE4;
const byte config_h = 0x00;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  Serial.println("AS5600 Permanent Burner");
  Serial.println("-------------------------");
  Serial.println("This sketch will write a new configuration and, if enabled,");
  Serial.println("permanently burn it to the AS5600's OTP memory.");
  Serial.println();
  
  // --- Step 1: Write the desired configuration ---
  Serial.println("1. Writing new configuration to registers...");
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(CONF_REG_H);
  Wire.write(config_h); // Write high byte
  Wire.write(config_l); // Write low byte
  Wire.endTransmission();
  delay(10);

  // --- Step 2: Verify the configuration was written correctly ---
  Serial.println("2. Verifying configuration...");
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(CONF_REG_H);
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDR, 2);
  byte read_h = Wire.read();
  byte read_l = Wire.read();

  if (read_h == config_h && read_l == config_l) {
    Serial.println("   SUCCESS: Registers written correctly.");
    Serial.print("   CONF_H (0x07): 0x"); Serial.println(read_h, HEX);
    Serial.print("   CONF_L (0x08): 0x"); Serial.println(read_l, HEX);
  } else {
    Serial.println("   ERROR: Verification failed! Halting.");
    while (1); // Stop execution
  }
  Serial.println();

#if BURN_ENABLED
  // --- Step 3: Issue the Burn Command (PERMANENT) ---
  Serial.println("3. BURN_ENABLED is set to 1. Proceeding with PERMANENT BURN.");
  Serial.println("   You have 5 seconds to cancel by resetting the board...");
  delay(5000);
  
  Serial.println("   Issuing BURN command...");
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(BURN_REG);
  Wire.write(0x40); // The magic byte to trigger the burn
  Wire.endTransmission();
  
  Serial.println("   BURN COMMAND SENT. The sensor is now permanently configured.");
  Serial.println("   Please power cycle the sensor to load the new settings.");

#else
  Serial.println("3. BURN_ENABLED is set to 0. Skipping permanent burn.");
  Serial.println("   The configuration is temporary and will be lost on power-off.");
  Serial.println("   Test the PWM output now. If it works as expected,");
  Serial.println("   set BURN_ENABLED to 1 and re-upload to make it permanent.");
#endif

  Serial.println();
  Serial.println("Script finished.");
}

void loop() {
  // The sketch only needs to run once.
}