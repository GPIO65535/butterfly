#ifndef __COM_PID_H__
#define __COM_PID_H__

typedef struct
{
    float kp;
    float ki;
    float kd;
    float actual;
    float target;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output; 
}PID;
void PID_Cal(PID *pid,float dt);
void PID_Cascade(PID *outer,PID *inner,float dt);
#endif