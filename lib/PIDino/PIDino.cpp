#include "PIDino.hpp"
#include <Arduino.h>

PIDino::PIDino(const float setPoint, const float minValue, const float maxValue, const float Kp, const float Ki, const float Kd)
       :setPoint_(setPoint),
        Kp_(Kp), Ki_(Ki), Kd_(Kd),
        minValue_(minValue), maxValue_(maxValue),
        last_error_(0), integral_(0), derivative_(), lastTime_(0) {
}

float PIDino::compute(const float input) {
    const float dt = (millis() - lastTime_) / 1000.0;

    const float e = setPoint_ - input;

    integral_ += dt / 2 * (e + last_error_);
    integral_ = constrain(integral_, minValue_, maxValue_);

    derivative_ = (e - last_error_) / dt;

    const float output = Kp_ * e + Ki_ * integral_ + Kd_ * derivative_;

    lastTime_ = millis();
    last_error_ = e;

    return output;
}