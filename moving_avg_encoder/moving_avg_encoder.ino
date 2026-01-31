// --- AS5600 PWM Measurement with Interrupts & Filtering ---

#include <Wire.h>
#include "AS5600.h"

// --- Configuration ---
const int PWM_PIN = 2; // IMPORTANT: MUST be an interrupt-capable pin (e.g., 2 or 3 on Uno/Nano)
const float ALPHA = 0.1; // Filter smoothness. Smaller = smoother but more lag. 0.1 is a good start.

AS5600 as5600;

// --- Global variables for the Interrupt Service Routine (ISR) ---
// 'volatile' is critical as these are shared between the ISR and the main loop.
volatile unsigned long highTime = 0;
volatile unsigned long lowTime = 0;
volatile unsigned long lastMicros = 0;
volatile bool newDataAvailable = false;

// --- Global variable for the filter ---
float smoothedTicks = 0.0; // Use a float for filter precision

// =========================================================================
// Interrupt Service Routine (ISR)
// This tiny function runs automatically in the background on every PWM signal change.
// =========================================================================
void onPwmChange() {
  unsigned long currentMicros = micros();
  unsigned long duration = currentMicros - lastMicros;
  lastMicros = currentMicros;

  if (digitalRead(PWM_PIN) == LOW) { // Fell from HIGH to LOW, just finished a high pulse
    highTime = duration;
  } else { // Rose from LOW to HIGH, just finished a low pulse
    lowTime = duration;
    newDataAvailable = true; // A full cycle is complete, flag the main loop
  }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nAS5600 High-Performance PWM Measurement");

  // --- Configure AS5600 via I2C (your original setup code) ---
  Wire.begin();
  as5600.begin(4); // Set direction pin
  as5600.setOutputMode(AS5600_OUTMODE_PWM);
  as5600.setPWMFrequency(AS5600_PWM_920); // Using a higher frequency is good for responsiveness
  Serial.println("AS5600 configured for PWM output.");
  
  // --- Configure the interrupt pin ---
  pinMode(PWM_PIN, INPUT_PULLUP); // PULLUP is more stable for digital inputs
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), onPwmChange, CHANGE);
  
  delay(1000); // Give a moment for initial readings
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  // Check if the ISR has delivered a new measurement
  if (newDataAvailable) {
    unsigned long localHighTime, localLowTime;

    // Create a "critical section" to safely copy the volatile data
    noInterrupts(); // Pause interrupts
    localHighTime = highTime;
    localLowTime = lowTime;
    newDataAvailable = false; // Reset the flag for the next measurement
    interrupts(); // Resume interrupts

    unsigned long period = localHighTime + localLowTime;

    if (period > 0) {
      // --- Calculate raw position in ticks ---
      float duty = (float)localHighTime / period;
      const float DUTY_MIN = 0.029;
      const float DUTY_MAX = 0.971;

      // Clamp duty cycle to the valid range
      if (duty < DUTY_MIN) duty = DUTY_MIN;
      if (duty > DUTY_MAX) duty = DUTY_MAX;

      int rawTicks = round((duty - DUTY_MIN) / (DUTY_MAX - DUTY_MIN) * 4095.0);

      // --- Apply the Exponential Moving Average Filter ---
      smoothedTicks = (ALPHA * rawTicks) + ((1.0 - ALPHA) * smoothedTicks);

      int finalPosition = (int)smoothedTicks;
      Serial.println(finalPosition); // Print the clean, filtered value
    }
  }

  // Your other robot code can run here freely. It will never be blocked!
  // The delay is no longer necessary for the measurement to work.
  // delay(2); 
}