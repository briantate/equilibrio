#include "pid.h"

void pid_clear_values(pid_t *pid){
    pid->error_last = 0;
    pid->PID_i = 0;
}

// float PidControl(uint16_t measured, uint16_t setpoint, float *error_last, float *PID_i)
float pid_control(pid_t *pid, uint16_t measured, uint16_t setpoint)
{
    float PID_p, PID_d, PID_total;
    float error;

    error = setpoint - measured;
    PID_p = pid->kp * error;

    PID_d = pid->kd*((error - pid->error_last)/pid->sampling_period);

    if(-100 < error && error < 100)
    {
      pid->PID_i = pid->PID_i + (pid->ki * error);
    }
    else
    {
      pid->PID_i = 0;
    }

    PID_total = PID_p + pid->PID_i + PID_d;

    PID_total = PID_p + pid->PID_i + PID_d;  
    
    pid->error_last = error;
    return PID_total;
}