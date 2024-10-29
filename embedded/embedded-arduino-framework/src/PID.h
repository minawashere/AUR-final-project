#ifndef PID_H
#define PID_H

#include <Arduino.h>


class PID {
    double kp, ki, kd, max_output;
    double accum = 0;
    double prev_err = 0;
    unsigned long long prev_time = micros();

public:
    PID(const float kp, const float ki, const float kd, const float max_output);

    double get_output(const double target, const double current);
};


#endif //PID_H
