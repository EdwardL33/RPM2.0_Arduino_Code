// AS5600_measure_pwm.ino
// solder 1k resistor from VCC to GPO??

#include "AS5600.h"

// 1 for angle
// 0 for ticks
#define UNITS 0

int PWMpin = 5;  // connect AS5600 OUT pin here
AS5600 as5600;
void setup()
{
  Serial.begin(115200);
  delay(100);
  // while (!Serial);  // wait for Serial (optional on some boards)
  Serial.println();
  Serial.println("AS5600 PWM Angle Measurement");
  Serial.println();

  pinMode(PWMpin, INPUT);

  Wire.begin();

  as5600.begin(4);  //  set direction pin.

  Serial.println("Configuring AS5600 to PWM mode...");

  // Read current mode
  uint8_t mode = as5600.getOutputMode();
  Serial.print("Current output mode: ");
  Serial.println(mode);

  // Set to PWM mode
  as5600.setOutputMode(AS5600_OUTMODE_PWM);

  // Optional: burn to EEPROM (only 3 times max!)
  // as5600.burnSetting();

  Serial.println("AS5600 set to PWM mode.");
  mode = as5600.getOutputMode();
  Serial.print("Current output mode: ");
  Serial.println(mode);
}

float measureAngle()
{
  // Measure HIGH and LOW pulse durations in microseconds
  unsigned long highTime = pulseIn(PWMpin, HIGH);
  unsigned long lowTime  = pulseIn(PWMpin, LOW);

  // Avoid division by zero
  unsigned long period = highTime + lowTime;
  if (period == 0) return -1.0;

  // Duty cycle (0-1)
  float duty = (float)highTime / period;

  // Map from AS5600 PWM duty cycle range (~2.9% to ~97.1%) to 0–360°
  const float DUTY_MIN = 0.029;   // 2.9%
  const float DUTY_MAX = 0.971;   // 97.1%
  
  // Clamp duty
  if (duty < DUTY_MIN) duty = DUTY_MIN;
  if (duty > DUTY_MAX) duty = DUTY_MAX;

  // Linear mapping to 0-360°
  float angle = (duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 360.0;

  return angle;
}

uint16_t measureTicks()
{
  // Measure HIGH and LOW pulse durations in microseconds
  unsigned long highTime = pulseIn(PWMpin, HIGH);
  unsigned long lowTime  = pulseIn(PWMpin, LOW);

  unsigned long period = highTime + lowTime;
  if (period == 0) return -1;  // no signal detected

  // Duty cycle (0-1)
  float duty = (float)highTime / period;

  // AS5600 PWM duty range: 2.9% - 97.1%
  const float DUTY_MIN = 0.029;
  const float DUTY_MAX = 0.971;

  // Clamp duty
  if (duty < DUTY_MIN) duty = DUTY_MIN;
  if (duty > DUTY_MAX) duty = DUTY_MAX;

  // Map to ticks (0-4095)
  int ticks = round((duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 4095.0);

  return ticks;
}


void loop()
{
  #if UNITS
    float angle = measureAngle();
    if (angle >= 0.0)
      Serial.println(angle, 1);  // print with 1 decimal
    else { 
      Serial.println("PWM signal not detected");
    }
  #else 
    uint16_t ticks = measureTicks();
    if (ticks != 0xFFFF)
      Serial.println(ticks);  // print 12-bit value
    else
      Serial.println("PWM signal not detected");
  #endif


  uint8_t magnetStatus = as5600.detectMagnet();

  Serial.print("Status: ");
  if (magnetStatus) {
    Serial.println("Magnet Detected! :)");
  } else if (magnetStatus & as5600.magnetTooWeak()) {
    Serial.println("Magnet too weak (move it closer).");
  } else if (magnetStatus & as5600.magnetTooStrong()) {
    Serial.println("Magnet too strong (move it away).");
  } else {
    Serial.println("No Magnet Detected. :(");
  }

  delay(1); 
}












// // AS5600_measure_pwm.ino
// // This code reads an AS5600 encoder's PWM output and transmits the value via radio.

// // --- My Code Start ---
// // Include libraries for the nRF24L01 radio
// #include <SPI.h>
// #include "RF24.h"
// #include "AS5600.h"
// // --- My Code End ---


// // 1 for angle (sends a float)
// // 0 for ticks (sends a uint16_t)
// #define UNITS 0

// // --- My Code Start ---
// // Define the CE and CSN pins for the radio.
// #define CE_PIN 7
// #define CSN_PIN 8

// // Instantiate an object for the nRF24L01 transceiver
// RF24 radio(CE_PIN, CSN_PIN);

// AS5600 as5600;

// // Define the address to transmit on. Your receiver must be listening on this same address.
// const byte address[] = "2Node";
// // --- My Code End ---


// int PWMpin = 4;  // connect AS5600 OUT pin here

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial);  // wait for Serial (optional on some boards)
  
//   Wire.begin();
//   as5600.begin(4);  //  set direction pin.

//   Serial.println("Configuring AS5600 to PWM mode...");

//   // Read current mode
//   uint8_t mode = as5600.getOutputMode();
//   // Serial.print("Current output mode: ");
//   // Serial.println(mode);

//   // Set to PWM mode
//   as5600.setOutputMode(AS5600_OUTMODE_PWM);

//   // Serial.println("AS5600 set to PWM mode.");
//   // mode = as5600.getOutputMode();
//   Serial.print("Current output mode: ");
//   Serial.println(mode);




//   pinMode(PWMpin, INPUT);

//   // --- My Code Start ---
//   // Initialize and configure the radio
//   if (!radio.begin()) {
//     Serial.println(F("Radio hardware is not responding!!"));
//     while (1) {} // Halt if radio fails to start
//   }

//   // Set the Power Amplifier level. Options: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
//   radio.setPALevel(RF24_PA_LOW);

//   // Open a writing pipe on the defined address
//   radio.openWritingPipe(address);

//   // Stop listening, which puts the radio in transmit mode
//   radio.stopListening();
//   Serial.println("Radio configured. Starting transmissions...");
//   // --- My Code End ---
// }

// float measureAngle()
// {
//   // Measure HIGH and LOW pulse durations in microseconds
//   unsigned long highTime = pulseIn(PWMpin, HIGH);
//   unsigned long lowTime  = pulseIn(PWMpin, LOW);

//   // Avoid division by zero
//   unsigned long period = highTime + lowTime;
//   if (period == 0) return -1.0;

//   // Duty cycle (0-1)
//   float duty = (float)highTime / period;

//   // Map from AS5600 PWM duty cycle range (~2.9% to ~97.1%) to 0–360°
//   const float DUTY_MIN = 0.029;   // 2.9%
//   const float DUTY_MAX = 0.971;   // 97.1%
  
//   // Clamp duty to valid range
//   if (duty < DUTY_MIN) duty = DUTY_MIN;
//   if (duty > DUTY_MAX) duty = DUTY_MAX;

//   // Linear mapping to 0-360°
//   float angle = (duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 360.0;

//   return angle;
// }

// uint16_t measureTicks()
// {
//   // Measure HIGH and LOW pulse durations in microseconds
//   unsigned long highTime = pulseIn(PWMpin, HIGH);
//   unsigned long lowTime  = pulseIn(PWMpin, LOW);

//   unsigned long period = highTime + lowTime;
//   if (period == 0) return 0xFFFF; // no signal detected, return max value as error

//   // Duty cycle (0-1)
//   float duty = (float)highTime / period;

//   // AS5600 PWM duty range: 2.9% - 97.1%
//   const float DUTY_MIN = 0.029;
//   const float DUTY_MAX = 0.971;

//   // Clamp duty to valid range
//   if (duty < DUTY_MIN) duty = DUTY_MIN;
//   if (duty > DUTY_MAX) duty = DUTY_MAX;

//   // Map to ticks (0-4095)
//   uint16_t ticks = round((duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 4095.0);

//   return ticks;
// }


// void loop()
// {
//   #if UNITS
//     float angle = measureAngle();
//     if (angle >= 0.0) {
//       // --- My Code Start ---
//       // Send the angle value over the radio
//       bool report = radio.write(&angle, sizeof(angle));
      
//       // Print confirmation to Serial Monitor for debugging
//       if (report) {
//         Serial.print("Angle sent: ");
//         Serial.println(angle, 1);
//       } else {
//         Serial.println("Transmission failed.");
//       }
//       // --- My Code End ---
//     }
//   #else 
//     uint16_t ticks = measureTicks();
//     if (ticks != 0xFFFF) {
//       // --- My Code Start ---
//       // Send the ticks value over the radio
//       bool report = radio.write(&ticks, sizeof(ticks));  // 2 bytes of data
      
//       // Print confirmation to Serial Monitor for debugging
//       if (report) {
//         Serial.print("Ticks sent: ");
//         Serial.println(ticks);
//       } else {
//         Serial.print(ticks);
//         Serial.println(" :Transmission failed.");
//       }
//       // --- My Code End ---
//     }
//   #endif
  
//   // --- My Code Start ---
//   // A slightly longer delay can improve reliability and make debugging easier
//   delay(1);
//   // --- My Code End ---
// }