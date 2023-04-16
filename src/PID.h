#include <Arduino.h>
// Header guard
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    float target_value;
    float Kp;
    float Ki;
    float Kd;

    float output_min;
    float output_max;

    unsigned long minimum_time_difference;

    float default_output;

private:
    unsigned long time_since_last_ran;
    unsigned long time_now;
    float total_error;
    float last_error;
    float control_signal;

public:
    void Init(float proportional_constant, float integral_constant, float differential_constant, float min_output, float max_output, float default_output, unsigned long minimum_time_difference = 0, float target_value = 0);
    void Update_target(float target_value);
    double Update_and_Return(float sensed_output);

};

#endif // PID_CONTROLLER_H
