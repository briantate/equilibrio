#ifndef _PID_H
#define _PID_H

#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

typedef struct{
    float kp; //4; //8
    float ki; //0.2
    float kd; //3100
    float error_last;
    float PID_i;
} pid_t;


void pid_clear_values(pid_t *pid);
float pid_control(pid_t *pid, uint16_t measured, uint16_t setpoint);


#ifdef __cplusplus
}
#endif

#endif //_PID_H