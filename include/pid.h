#pragma once

#include <cmath>

/**
 * @brief Simple PID controller with anti-windup and output clamping
 * 
 * Uses float precision for STM32F103 (no FPU).
 */
class PIDController {
public:
    struct Gains {
        float kP = 0.0f;
        float kI = 0.0f;
        float kD = 0.0f;
        
        Gains() = default;
        Gains(float p, float i, float d) : kP(p), kI(i), kD(d) {}
    };

    PIDController() = default;
    
    PIDController(float kP, float kI, float kD, float outputMin = -1e6f, float outputMax = 1e6f)
        : _gains(kP, kI, kD), _outputMin(outputMin), _outputMax(outputMax) {}
    
    PIDController(const Gains& gains, float outputMin = -1e6f, float outputMax = 1e6f)
        : _gains(gains), _outputMin(outputMin), _outputMax(outputMax) {}

    void setGains(float kP, float kI, float kD) {
        _gains.kP = kP;
        _gains.kI = kI;
        _gains.kD = kD;
    }
    
    void setGains(const Gains& gains) {
        _gains = gains;
    }

    void setOutputLimits(float min, float max) {
        _outputMin = min;
        _outputMax = max;
    }

    void setIntegratorLimits(float min, float max) {
        _integralMin = min;
        _integralMax = max;
    }

    void reset() {
        _integral = 0.0f;
        _prevError = 0.0f;
        _firstUpdate = true;
    }

    /**
     * @brief Compute PID output
     * 
     * @param error Current error (setpoint - measurement)
     * @param dt Time step in seconds
     * @return Clamped PID output
     */
    float compute(float error, float dt) {
        if (dt <= 0.0f) return 0.0f;

        // Proportional term
        float pTerm = _gains.kP * error;

        // Integral term with anti-windup
        _integral += error * dt;
        _integral = clamp(_integral, _integralMin, _integralMax);
        float iTerm = _gains.kI * _integral;

        // Derivative term (on error, not measurement to avoid derivative kick on setpoint change)
        float dTerm = 0.0f;
        if (!_firstUpdate) {
            float derivative = (error - _prevError) / dt;
            dTerm = _gains.kD * derivative;
        }
        _prevError = error;
        _firstUpdate = false;

        // Sum and clamp output
        float output = pTerm + iTerm + dTerm;
        return clamp(output, _outputMin, _outputMax);
    }

    float compute(float setpoint, float measurement, float dt) {
        return compute(setpoint - measurement, dt);
    }

    const Gains& getGains() const { return _gains; }
    float getIntegral() const { return _integral; }
    float getPrevError() const { return _prevError; }

private:
    static float clamp(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    Gains _gains;
    
    float _integral = 0.0f;
    float _prevError = 0.0f;
    bool _firstUpdate = true;
    
    float _outputMin = -1e6f;
    float _outputMax = 1e6f;
    float _integralMin = -1000.0f;
    float _integralMax = 1000.0f;
};
