#ifndef PID_H
#define PID_H




class PID {
    float kp, ki, kd, max_output;
    float accum{};
    float prev_err{};
    unsigned long long prev_time{};
public:
    PID(const float kp,const  float ki, const float kd, const float max_output);

    double get_output(double target, double current);
};



#endif //PID_H
