/*
 * PIDController.h
 * * A reusable PID controller class
 * * - Manages its own time (dt)
 * - Uses trapezoidal rule for integral
 * - Implements positive-only integral clamping
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h> // Required for millis()

class PIDController {
public:
    /**
     * @brief Constructor
     * @param p Proportional gain (kP)
     * @param i Integral gain (kI)
     * @param d Derivative gain (kD)
     * @param cap The positive limit for the integral (integralCap)
     */
    PIDController(float p, float i, float d, float cap) {
        setGains(p, i, d);
        setIntegralCap(cap);
        reset(); // Initialize all state variables
    }

    /**
     * @brief Computes the PID control output
     * @param setpoint The desired value (your 'desired' variable)
     * @param measurement The current value (your 'current' variable)
     * @return The PID control output (your 'pid' variable)
     */
    float compute(float setpoint, float measurement) {
        unsigned long currTime = millis();

        // On the first run or after a reset, initialize time and state
        if (prevTime == 0) {
            prevTime = currTime;
            lastError = 0;
            integral = 0;
            return 0; // No output on the first loop
        }

        // Calculate time delta (dt) in milliseconds
        unsigned long dt_ms = currTime - prevTime;

        // If no time has passed, return 0 to avoid division by zero
        if (dt_ms == 0) {
            return 0;
        }

        // --- This logic is identical to your original code ---

        // Convert dt to seconds (as a float)
        float dt_sec = dt_ms / 1000.0f;

        // Calculate error
        float error = setpoint - measurement;

        // Calculate integral using the trapezoidal rule
        integral += dt_sec * (error + lastError) / 2.0f;

        // Apply integral clamping (anti-windup)
        if (integral > integralCap) {
            integral = integralCap;
        } else if (integral < -integralCap) { 
            integral = -integralCap;
        }

        // Calculate derivative
        float derivative = (error - lastError) / dt_sec;

        // Calculate the final PID output
        float output = (kP * error) + (kI * integral) + (kD * derivative);

        // --- End of identical logic ---

        // Save state for the next computation
        lastError = error;
        prevTime = currTime;

        return output;
    }

    /**
     * @brief Resets the controller's internal state.
     * Call this when changing modes or disabling/enabling the controller
     * to prevent huge derivative spikes from old time values.
     */
    void reset() {
        integral = 0;
        lastError = 0;
        prevTime = 0; // Will be re-initialized on the next compute() call
    }

    /**
     * @brief Set new PID gains at runtime
     */
    void setGains(float p, float i, float d) {
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * @brief Set a new integral cap at runtime
     */
    void setIntegralCap(float cap) {
        integralCap = cap;
    }

private:
    // Gains
    float kP;
    float kI;
    float kD;

    // State
    float integral;
    float lastError;
    float integralCap;

    // Timekeeping
    unsigned long prevTime;
};

#endif // PID_CONTROLLER_H