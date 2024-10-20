
#include "PID.h"
#include "Arduino.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define clamp(v, l, h) min(max(v, l), h)

    PID::PID(const float kp,const  float ki, const float kd, const float max_output):
            kp(kp), ki(ki), kd(kd), max_output(max_output) {}

    float PID::get_output(const float target, const float current) {
        const float err = target - current;
        const auto dt = (micros() - prev_time) / 1000000.0;

        const float p = kp * err;
        const float d = kd * (prev_err - err) / dt;
        accum += err * dt;
        accum = clamp(accum, -max_output, max_output);

        const float i = ki * accum;

        prev_err = err;
        return p + d + i;
    }
