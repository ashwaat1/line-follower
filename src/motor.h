#include "driver/mcpwm.h"

#define AIN1 19
#define BIN1 21
#define AIN2 18
#define BIN2 22

#define PWMA 17
#define PWMB 23

// #define STBY 19

void motor_configure(){
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWMA);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PWMB);

    printf("Configuring Initial Parameters of mcpwm0...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    printf("Configuring Initial Parameters of mcpwm1...\n");
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    

    // pinMode(STBY, GPIO_MODE_OUTPUT);
    // digitalWrite(STBY, 1);

}

void motorL_drive(float dutycycle){

    if (dutycycle >= 0){
        digitalWrite(AIN1, 1);
        digitalWrite(AIN2, 0);
    }
    else{
        dutycycle = -dutycycle;
        digitalWrite(AIN1, 0);
        digitalWrite(AIN2, 1);
    }

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutycycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state}
}

void motorR_drive(float dutycycle){

    if (dutycycle >= 0){
        digitalWrite(BIN1, 1);
        digitalWrite(BIN2, 0);
    }
    else{
        dutycycle = -dutycycle;
        digitalWrite(BIN1, 0);
        digitalWrite(BIN2, 1);
    }

    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, dutycycle);
    mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state}
}

void motorL_brake(){
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);

    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
}

void motorR_brake(){
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);

    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
}

void motorL_turnoff(){
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 0);

    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);    
}

void motorR_turnoff(){
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 0);

    mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
}

void both_brake(){
    motorL_brake();
    motorR_brake();
}