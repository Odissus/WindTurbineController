#include <Arduino.h>
#include <PID.h>

void PIDController::Init(float proportional_constant, float integral_constant, float differential_constant, float min_output, float max_output, float default_output, unsigned long minimum_time_difference, float target_value) {
    this->Kp = proportional_constant;
    this->Ki = integral_constant;
    this->Kd = differential_constant;
    this->output_min = min_output;
    this->output_max = max_output;
    this->minimum_time_difference = minimum_time_difference;
    this->target_value = target_value;
    this->default_output = default_output;
    this->control_signal = default_output;
}

void PIDController::Update_target(float target_value) {
    this->target_value = target_value;
}

double PIDController::Update_and_Return(float sensed_output) {
    this->time_now = millis();
    unsigned long delta_time = this->time_now - this->time_since_last_ran; //delta time interval 
    if (delta_time >= minimum_time_difference) {
        float error = this->target_value - sensed_output;

        this->total_error += error; //accumalates the error - integral term
        if (total_error >= this->output_max) {
            this->total_error = this->output_max;
        } else if (total_error <= this->output_min) {
            this->total_error = this->output_min;
        }

        float delta_error = error - this->last_error; //difference of error for derivative term

        float control_signal = this->Kp * error + (this->Ki * delta_time) * total_error + (this->Kd / delta_time) * delta_error; //PID control compute
        if (control_signal >= this->output_max) {
            control_signal = this->output_max;
        } else if (control_signal <= this->output_min) {
            control_signal = this->output_min;
        }

        this->last_error = error;
        this->time_since_last_ran = this->time_now;
        this->control_signal = control_signal;
        return control_signal;
    }
    // If the controller hasn't ran for long enough, return last value 
    return this->control_signal;
}
