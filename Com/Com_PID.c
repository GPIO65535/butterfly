#include"Com_PID.h"
void PID_Cal(PID *pid,float dt)
{
    pid->error=pid->actual-pid->target;
    pid->integral+=pid->error*dt;
    pid->derivative=(pid->error-pid->last_error)/dt;
    pid->output=pid->kp*pid->error+pid->ki*pid->integral+pid->kd*pid->derivative;
    pid->last_error=pid->error;
}
void PID_Cascade(PID *outer,PID *inner,float dt)
{
    PID_Cal(outer,dt);
    inner->target=outer->output;
    PID_Cal(inner,dt);
}