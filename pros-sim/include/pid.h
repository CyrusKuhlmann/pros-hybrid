#pragma once

#include <cmath>
#include "api.h"

/**
 * @brief Simple PID controller (replaces lemlib::PID)
 *
 * Supports integral anti-windup and optional sign-flip integral reset.
 */
class PID {
public:
    /**
     * @brief Construct a PID controller
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param windupRange Anti-windup range; integral only accumulates when |error| < windupRange (0 = disabled)
     * @param signFlipReset If true, reset integral when error crosses zero
     */
    PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false)
        : kP(kP), kI(kI), kD(kD), windupRange(windupRange), signFlipReset(signFlipReset) {
    }

    /**
     * @brief Update the controller with a new error value
     * @param error Current error
     * @return Controller output
     */
    float update(float error) {
        // Sign-flip integral reset
        if (signFlipReset && ((error > 0 && prevError < 0) || (error < 0 && prevError > 0))) {
            integral = 0;
        }

        // Accumulate integral with anti-windup
        if (windupRange == 0 || std::abs(error) < windupRange) {
            integral += error;
        }

        // Derivative
        float derivative = error - prevError;
        prevError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    /**
     * @brief Reset the controller state
     */
    void reset() {
        integral = 0;
        prevError = 0;
    }

private:
    const float kP;
    const float kI;
    const float kD;
    const float windupRange;
    const bool signFlipReset;
    float integral = 0;
    float prevError = 0;
};

/**
 * @brief Exit condition based on error staying within a range for a specified time (replaces lemlib::ExitCondition)
 */
class ExitCondition {
public:
    /**
     * @brief Construct an exit condition
     * @param range Error must be below this value
     * @param time Error must stay below range for this many milliseconds
     */
    ExitCondition(float range, int time)
        : range(range), time(time) {
    }

    /**
     * @brief Update with current error and check if exit condition is met
     * @param input Current absolute error value
     * @return true if the error has been within range for the required time
     */
    bool update(float input) {
        if (input < range) {
            if (startTime < 0) startTime = (int)pros::millis();
            if (((int)pros::millis() - startTime) >= time) {
                done = true;
            }
        }
        else {
            startTime = -1;
        }
        return done;
    }

    /**
     * @brief Check if exit condition has been met (without updating)
     */
    bool getExit() const { return done; }

    /**
     * @brief Reset the exit condition state
     */
    void reset() {
        startTime = -1;
        done = false;
    }

private:
    const float range;
    const int time;
    int startTime = -1;
    bool done = false;
};
