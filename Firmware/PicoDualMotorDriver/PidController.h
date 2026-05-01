#pragma once

#include <Arduino.h>

class PidController {
 public:
  struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
  };

  void setGains(float kp, float ki, float kd) {
    gains_.kp = kp;
    gains_.ki = ki;
    gains_.kd = kd;
  }

  void setGains(const Gains &gains) {
    gains_ = gains;
  }

  Gains gains() const {
    return gains_;
  }

  void setOutputLimits(float minOutput, float maxOutput) {
    if (minOutput > maxOutput) {
      const float swap = minOutput;
      minOutput = maxOutput;
      maxOutput = swap;
    }
    minOutput_ = minOutput;
    maxOutput_ = maxOutput;
    output_ = clamp(output_, minOutput_, maxOutput_);
    integral_ = clamp(integral_, -integralLimit_, integralLimit_);
  }

  void setIntegralLimit(float limit) {
    if (limit < 0.0f) {
      limit = -limit;
    }
    integralLimit_ = limit;
    integral_ = clamp(integral_, -integralLimit_, integralLimit_);
  }

  void reset(float integral = 0.0f) {
    integral_ = clamp(integral, -integralLimit_, integralLimit_);
    previousError_ = 0.0f;
    output_ = 0.0f;
    initialized_ = false;
  }

  float update(float setpoint, float measurement, float dtSeconds) {
    if (dtSeconds <= 0.0f) {
      return output_;
    }

    const float error = setpoint - measurement;
    const float derivative = initialized_ ? (error - previousError_) / dtSeconds : 0.0f;
    const float candidateIntegral = clamp(integral_ + error * dtSeconds, -integralLimit_, integralLimit_);
    const float candidateOutput = gains_.kp * error + gains_.ki * candidateIntegral + gains_.kd * derivative;
    const float clampedOutput = clamp(candidateOutput, minOutput_, maxOutput_);

    if (clampedOutput == candidateOutput ||
        (clampedOutput >= maxOutput_ && error < 0.0f) ||
        (clampedOutput <= minOutput_ && error > 0.0f)) {
      integral_ = candidateIntegral;
    }

    previousError_ = error;
    output_ = clampedOutput;
    initialized_ = true;
    return output_;
  }

  float output() const {
    return output_;
  }

  float integral() const {
    return integral_;
  }

  float previousError() const {
    return previousError_;
  }

 private:
  static float clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
      return minValue;
    }
    if (value > maxValue) {
      return maxValue;
    }
    return value;
  }

  Gains gains_;
  float minOutput_ = -1.0f;
  float maxOutput_ = 1.0f;
  float integralLimit_ = 1.0f;
  float integral_ = 0.0f;
  float previousError_ = 0.0f;
  float output_ = 0.0f;
  bool initialized_ = false;
};
