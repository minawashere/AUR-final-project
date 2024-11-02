
#include "PID.h"
#include "Arduino.h"
#include "float.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define clamp(v, l, h) min(max(v, l), h)

PID::PID(const float kp, const float ki, const float kd, const float max_output): kp(kp), ki(ki), kd(kd),
    max_output(max_output) {
}

double PID::get_output(const double target, const double current) {
    const auto err = target - current;
    const auto dt = (micros() - prev_time) / 1000000.0;

    if(dt > DBL_EPSILON)
    {
        const auto p = kp * err;
        const auto d = kd * (prev_err - err) / dt;
        accum += err * dt;
        accum = clamp(accum, -max_output, max_output);

        const auto i = ki * accum;

        prev_err = err;
        return p + d + i;
    } return accum;
}
