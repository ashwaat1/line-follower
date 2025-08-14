#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
	/* Controller gains */
	float Kp;
	float Ki;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Controller "memory" */
	float integrator;
    
} PIDController;


float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    float out = 0.0f;

	/*
	* Error signal
	*/
    float error = setpoint - measurement;

	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;

	/*
	* Integral
	*/
    pid->integrator = pid->integrator + error;
    float integral = pid->Ki * pid->integrator;

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } 
    else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }

	/*
	* Compute output and apply limits
	*/
    out = proportional + integral;

    if (out > pid->limMax) {
        out = pid->limMax;
    } 
    else if (out < pid->limMin) {
        out = pid->limMin;
    }

	/* Return controller output */
    return out;
}

#endif