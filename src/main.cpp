#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/pcnt.h>
#include "driver/gptimer.h"
#include <soc/pcnt_struct.h>

#include "motor.h"
#include "ESP32Encoder.h"
#include "PID.h"

#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#include "BluetoothSerial.h" // Include the Bluetooth Serial library
BluetoothSerial SerialBT; // Create a Bluetooth Serial object
#endif

using namespace std;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#define COUNTER_H_LIM h_lim_lat
#define COUNTER_L_LIM l_lim_lat

#define PID_Kp 0.3 // 0.3
#define PID_Ki 0.2 // 0.2
#define PID_MinLimit -100.0
#define PID_MaxLimit 100.0
#define PID_MinIntLimit -9999.0
#define PID_MaxIntLimit 9999.0

#define LOOP_FREQ_MS 5
#define RAMP_INTERVAL 1

#define DRY_RUN_RPM 250//200
#define EXTRA_INCH 180//250

#define TURN_HIGH_RPM 250//250
#define TURN_LOW_RPM 50//50
#define ROTATE_RPM 250//100 ,for 250rpm turn time is 250 and for 300 rpm turn time is 208.33
#define TURN_TIME 230//600,250,208.33
#define UTURN_FACTOR 2//2
#define FACTOR_45 2


//-----------------------------------------------------------------------
// Pin definitions

#define L_MIDDLE 32 //
#define LEFT_MIDDLE 33 //
#define RMIDDLE 35   //
#define RIGHT_MIDDLE 34 // 
#define farRightSensorPin 36 //
#define farLeftSensorPin 13
#define SW_PIN 5
#define LED_PIN 15

uint16_t sensorValues[6];

#define ENC_RA 25
#define ENC_RB 26  
#define ENC_LA 27
#define ENC_LB 14

#define GPIO_Read(PIN) (REG_READ(GPIO_IN1_REG) & (1 << (PIN))) >> (PIN)
#define GPIO1_Read(PIN) (REG_READ(GPIO_IN1_REG) & (1 << (PIN-32))) >> (PIN-32)



PIDController PID_L = {PID_Kp, PID_Ki, PID_MinLimit, PID_MaxLimit, PID_MinIntLimit, PID_MaxIntLimit, 0.0f};
PIDController PID_R = {PID_Kp, PID_Ki, PID_MinLimit, PID_MaxLimit, PID_MinIntLimit, PID_MaxIntLimit, 0.0f};

TaskHandle_t PID_task;
TaskHandle_t motor_control_core0_task;

ESP32Encoder encoder;
ESP32Encoder encoder2;

QueueHandle_t left_queue;
QueueHandle_t right_queue;

volatile int_fast32_t pulseL = 0, pulseR = 0;
volatile int_fast32_t pulse_prevL = 0, pulse_prevR = 0;

volatile int_fast16_t rptmL = 0, rptmR = 0; // rotations per 1000 minutes

volatile float setpoint_L_prev = 0.0;
volatile float setpoint_R_prev = 0.0;

volatile float L_duty = 0.0; 
volatile float R_duty = 0.0;

float ramp_values_L[RAMP_INTERVAL];
uint_fast8_t ramp_index_L = RAMP_INTERVAL - 1;

float ramp_values_R[RAMP_INTERVAL];
uint_fast8_t ramp_index_R = RAMP_INTERVAL - 1;

int nolinecounter = 0;
int left_sensor = 0;
int lstat = 0;

// Extra Added.
char path[100] = " ";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0; // used to reach an specific array element.
unsigned int status = 0; 

//QTRSensors qtr;

static bool IRAM_ATTR enc_timer_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
    
    int16_t c;
	int_fast32_t compensate = 0;

	taskENTER_CRITICAL_ISR(&spinlock);
	pcnt_get_counter_value(encoder.unit, &c);
	if (PCNT.int_st.val & BIT(encoder.unit)) {
        pcnt_get_counter_value(encoder.unit, &c);
		if(PCNT.status_unit[encoder.unit].COUNTER_H_LIM){
			compensate = encoder.r_enc_config.counter_h_lim;
		} else if (PCNT.status_unit[encoder.unit].COUNTER_L_LIM) {
			compensate = encoder.r_enc_config.counter_l_lim;
		}
	}
    pulseL = encoder.count + compensate + c;

	pcnt_get_counter_value(encoder2.unit, &c);
	if (PCNT.int_st.val & BIT(encoder2.unit)) {
        pcnt_get_counter_value(encoder2.unit, &c);
		if(PCNT.status_unit[encoder2.unit].COUNTER_H_LIM){
			compensate = encoder2.r_enc_config.counter_h_lim;
		} else if (PCNT.status_unit[encoder2.unit].COUNTER_L_LIM) {
			compensate = encoder2.r_enc_config.counter_l_lim;
		}
	}
    pulseR = encoder2.count + compensate + c;    
	taskEXIT_CRITICAL_ISR(&spinlock);

    rptmL = 42857 * (pulseL - pulse_prevL) / LOOP_FREQ_MS;
    rptmR = 42857 * (pulseR - pulse_prevR) / LOOP_FREQ_MS;     

    pulse_prevL = pulseL;
    pulse_prevR = pulseR;

    // Notify PID task to execute
    vTaskNotifyGiveIndexedFromISR( PID_task, 0, &xHigherPriorityTaskWoken );  

    return 0;
}

static void encoder_configure(){
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

	encoder.attachFullQuad(ENC_LA, ENC_LB);
	encoder2.attachFullQuad(ENC_RA, ENC_RB);

    encoder.setFilter(1000);
    encoder2.setFilter(1000);

    encoder.clearCount();
    encoder2.clearCount();  

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));


    gptimer_event_callbacks_t cbs = {
        .on_alarm = enc_timer_intr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = LOOP_FREQ_MS * 1000, // period = 20ms @resolution 1MHz
        .reload_count = 0, // counter will reload with 0 on alarm event
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

static void PID_fn(void* parameter){
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int spL, spR;
        if (xQueueReceive(left_queue, &spL, 0) == pdTRUE) {
        }
        if (xQueueReceive(right_queue, &spR, 0) == pdTRUE) {
        }        

        if (spL != setpoint_L_prev){
            // PIDController_Reset(&PID_L);
            for (int i=0 ; i<RAMP_INTERVAL ; i++){
                ramp_values_L[i] = setpoint_L_prev + (((spL - setpoint_L_prev) / (float) RAMP_INTERVAL) * (i+1));
            ramp_index_L = 0;
            }
            setpoint_L_prev = spL;
        }

        if (spR != setpoint_R_prev){
            // PIDController_Reset(&PID_R);
            for (int i=0 ; i<RAMP_INTERVAL ; i++){
                ramp_values_R[i] = setpoint_R_prev + (((spR - setpoint_R_prev) / (float) RAMP_INTERVAL) * (i+1));
            }
            ramp_index_R = 0;
            setpoint_R_prev = spR;
        }

        if (ramp_values_L[ramp_index_L] == 0.0){
            motorL_brake();
            // PID_L.integrator = 0;
            // PID_R.integrator = 0;
            // L_duty = 0;
            // R_duty = 0;
        }
        else{
            L_duty = PIDController_Update(&PID_L , ramp_values_L[ramp_index_L], rptmL / 1000.0);
            motorL_drive(L_duty);

            if (ramp_index_L != (RAMP_INTERVAL - 1) )
                ramp_index_L++;
        }

        if (ramp_values_R[ramp_index_R] == 0.0){
            motorR_brake();
        }
        else{
            R_duty = PIDController_Update(&PID_R , ramp_values_R[ramp_index_R], rptmR / 1000.0);
            motorR_drive(R_duty);
            if (ramp_index_R != (RAMP_INTERVAL - 1) )
                ramp_index_R++;
        }
        // #ifdef DEBUG_SERIAL
        // Serial.printf("L_Cur : %.3f R_Cur : %.3f L_Duty : %.3f R_Duty : %.3f L_RPM : %.3f R_RPM : %.3f L_Int : %.3f R_Int : %.3f\n", ramp_values_L[ramp_index_L], ramp_values_R[ramp_index_R], L_duty, R_duty, (float)rptmL/1000.0, (float)rptmR/1000.0, PID_L.integrator, PID_R.integrator);
        // #endif
    }
}

int P = 0, I = 0, D = 0;
int error = 0;
int mode;

#define St_Kp 22
#define St_Ki 0

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2
# define LEFT_TURN 10
# define RIGHT_TURN 20
# define DIAMOND 30

bool started = false;

void calculateError(){

    int LFSensor[6] ; 
    LFSensor[0]=digitalRead(16);
    LFSensor[1]=digitalRead(LEFT_MIDDLE);
    LFSensor[2]=digitalRead(L_MIDDLE);
    LFSensor[3]=digitalRead(RMIDDLE);
    LFSensor[4]=digitalRead(RIGHT_MIDDLE);
    LFSensor[5]=digitalRead(39);

    int farRightSensor = digitalRead(farRightSensorPin);
    int farLeftSensor = digitalRead(farLeftSensorPin);
    #ifdef DEBUG_SERIAL
    Serial.printf("%d %d %d %d %d \n", LFSensor[0], LFSensor[1], LFSensor[2], LFSensor[3],LFSensor[4]);  
    Serial.printf("%d %d\n",farRightSensor,farLeftSensor);  
    SerialBT.printf("%d %d %d %d %d %d %d %d \n", farLeftSensor,LFSensor[0], LFSensor[1], LFSensor[2], LFSensor[3], LFSensor[4], LFSensor[5], farRightSensor); 
    #endif    
    

    if(( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1))  {mode = STOPPED; error = 0;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = STOPPED; error = 0;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = STOPPED; error = 0;}

    else if ((farRightSensor==1)&&(LFSensor[5]== 1 )&&(LFSensor[4]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[2]== 1 )) {mode = RIGHT_TURN; error = 0;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1))  {mode = RIGHT_TURN; error = 0;}

    else if ((farLeftSensor==1)&&(LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )) {mode = LEFT_TURN; error = 0;}

    else if((farLeftSensor==1)&&( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0)&&(farRightSensor==1))  {mode = DIAMOND; error = 0;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0)&&(farRightSensor==1))  {mode = DIAMOND; error = 0;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = DIAMOND; error = 0;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = DIAMOND; error = 0;}

    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0)&&(farRightSensor==1))  {mode = FOLLOWING_LINE; error = 12;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = FOLLOWING_LINE; error = 11;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = FOLLOWING_LINE; error = 10;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1)&&(farRightSensor==1))  {mode = FOLLOWING_LINE; error = 10;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1))  {mode = FOLLOWING_LINE; error = 8;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1))  {mode = FOLLOWING_LINE; error = 7;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==1))  {mode = FOLLOWING_LINE; error = 6;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = 4;}  
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = 1;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = 1;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = 0;}
    else if((farLeftSensor == 0)&&(LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1)&&(LFSensor[5]== 0 )&&(farRightSensor==0))  {mode = FOLLOWING_LINE; error = 0;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 0)&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -1;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 1)&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -1;}
    else if(( LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -4;} 
    else if(( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -6;}
    else if(( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -7;} 
    else if(( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -8;} 
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -10;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -10;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -11;}
    else if((farLeftSensor==1)&&( LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0)&&(LFSensor[5]==0))  {mode = FOLLOWING_LINE; error = -12;}

    else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )&&(LFSensor[5]==0))  {
        nolinecounter++;
        if (nolinecounter == 3){
            mode = NO_LINE; error = 0;
            nolinecounter = 0;
        }
    }
    else{mode = FOLLOWING_LINE; error = 0;}

    if(farLeftSensor == 1) {
        left_sensor = 1;
    }
    else{left_sensor = 0;}
}


void steeringPID(){
    // calculateError();

    if (mode == NO_LINE){
        int spL = DRY_RUN_RPM;
        int spR = DRY_RUN_RPM;
        xQueueSend(left_queue, &spL, 0);
        xQueueSend(right_queue, &spR, 0);   
    }

    else{
        
        P = error;
        I = I + error;

        int PIDvalue = (St_Kp * P) + (St_Ki * I);

        int spL = DRY_RUN_RPM + PIDvalue;
        int spR = DRY_RUN_RPM - PIDvalue;

        if (spL > 300) {
            spL = 300;
        }
        if (spL < 0) {
            spL = 0;
        }
        if (spR > 300) {
            spR = 300;
        }
        if (spR < 0) {
            spR = 0;
        }
        xQueueSend(left_queue, &spL, 0);
        xQueueSend(right_queue, &spR, 0);   
    }
}

void rotate_left(){
    
    int spR = ROTATE_RPM;   // Slow down left motor
    int spL = -ROTATE_RPM; // Maintain speed on  motor
        xQueueSend(left_queue, &spL, 0);
        xQueueSend(right_queue, &spR, 0);

    vTaskDelay(pdMS_TO_TICKS(TURN_TIME));
    spR = 0 ;   // Slow down left motor
    spL = 0; // Maintain speed on  motor
        xQueueSend(left_queue, &spL, 0);
        xQueueSend(right_queue, &spR, 0);

}

void rotate_right(){
    
    int spL = ROTATE_RPM;   // Slow down left motor
    int spR = -ROTATE_RPM; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

    vTaskDelay(pdMS_TO_TICKS(TURN_TIME));

    spR = 0 ;   // Slow down left motor
    spL = 0; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

}

void rotate_left_45(){
    
    int spL = -ROTATE_RPM;   // Slow down left motor
    int spR = ROTATE_RPM; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

    vTaskDelay(pdMS_TO_TICKS(TURN_TIME/FACTOR_45));

    spR = 0 ;   // Slow down left motor
    spL = 0; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

}

void rotate_right_45(){
    
    int spL = ROTATE_RPM;   // Slow down left motor
    int spR = -ROTATE_RPM; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

    vTaskDelay(pdMS_TO_TICKS(TURN_TIME/FACTOR_45));

    spR = 0 ;   // Slow down left motor
    spL = 0; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

}

 // later Implemented
void bot_180(){
    
    int spR = ROTATE_RPM;   // Slow down left motor
    int spL = -ROTATE_RPM; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

    vTaskDelay(pdMS_TO_TICKS(TURN_TIME * UTURN_FACTOR));

    spR = 0 ;   // Slow down left motor
    spL = 0; // Maintain speed on  motor
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

    
}
void motorStop(){
    

    int spR=0;
    int spL=0;
    xQueueSend(left_queue, &spL, 0);
    xQueueSend(right_queue, &spR, 0);

    started = false;

    
}

int runExtraInch(void){
    #ifdef DEBUG_SERIAL
    SerialBT.println("extra");
    #endif

    int k = 0;
    lstat = 0;

    for (int i=0; i<(EXTRA_INCH / 25); i++){
        calculateError();
        if (mode == DIAMOND) {
            SerialBT.println("DIAMOND");
            k = 30;
        }
        if (mode == STOPPED){
            k = 30;
        }
        if (left_sensor == 1){
            lstat = 1;
        }
        steeringPID();
        delay(25); 
    }

    #ifdef DEBUG_SERIAL
    SerialBT.println("extra done");
    #endif

    delay(EXTRA_INCH % 25);
    return k;
    // delay(EXTRA_INCH);
}

int runExtraInchWithout(void){
    #ifdef DEBUG_SERIAL
    SerialBT.println("extra");
    #endif

    int k = 0;
    lstat = 0;

    for (int i=0; i<(EXTRA_INCH / 25); i++){
        calculateError();
        if (mode == DIAMOND) {
            SerialBT.println("DIAMOND");
            k = 30;
        }
        if (left_sensor == 1){
            lstat = 1;
        }
        delay(25); 
    }

    #ifdef DEBUG_SERIAL
    SerialBT.println("extra done");
    #endif

    delay(EXTRA_INCH % 25);
    return k;
    // delay(EXTRA_INCH);
}

int runHalfInch(void){
    #ifdef DEBUG_SERIAL
    SerialBT.println("extra");
    #endif

    int k = 0;
    lstat = 0;

    for (int i=0; i<(EXTRA_INCH / 50); i++){
        calculateError();
        if (mode == DIAMOND) {
            SerialBT.println("DIAMOND");
            k = 30;
        }
        if (left_sensor == 1){
            lstat = 1;
        }
        delay(25); 
    }

    #ifdef DEBUG_SERIAL
    SerialBT.println("extra half done");
    #endif

    delay(EXTRA_INCH % 25);
    return k;
    // delay(EXTRA_INCH);
}

void mazeEnd(){
    motorStop();
    status = 1;
    started = false;
}

void mazeSolve(){
    int counter = 1;
    
    while (!status){
        calculateError();
        steeringPID();
        
        if (counter != 1){
            counter++;
        }
        else{
            counter = 1;
            switch (mode)
            {
                case NO_LINE:
                    #ifdef DEBUG_SERIAL
                    Serial.println("None");
                    SerialBT.println("None");
                    #endif

                    runExtraInch();
                    motorStop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    bot_180();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;

                case STOPPED: // CONT_LINE
                    #ifdef DEBUG_SERIAL
                    Serial.println("Stopped");
                    SerialBT.println("Stopped");
                    #endif


                    if (runHalfInch() == 30){
                        SerialBT.println("Diamond taken");
                        motorStop();
                        vTaskDelay(pdMS_TO_TICKS(100));
                        calculateError();
                        rotate_right_45();
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    else {
                        if (runHalfInch() == 30){
                            SerialBT.println("Diamond taken");
                            motorStop();
                            vTaskDelay(pdMS_TO_TICKS(100));
                            calculateError();
                            rotate_right_45();
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        else if (mode == STOPPED){
                            motorStop();
                            vTaskDelay(pdMS_TO_TICKS(100));
                            calculateError();
                            mazeEnd();                      
                        }
                        else{
                            motorStop();
                            vTaskDelay(pdMS_TO_TICKS(100));
                            calculateError();                        
                            rotate_right(); // or it is a "T" or "Cross"). In both cases, goes to LEFT
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }

                    }
                    break;

                case LEFT_TURN:
                    #ifdef DEBUG_SERIAL
                    Serial.println("left");
                    SerialBT.println("left");
                    #endif

                    if (runExtraInch() == 30){
                        SerialBT.println("Diamond taken");
                        motorStop();
                        vTaskDelay(pdMS_TO_TICKS(100));
                        calculateError();
                        rotate_left_45();
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    
                    else{
                        calculateError();
                        if (mode == NO_LINE)  {
                            motorStop();
                            vTaskDelay(pdMS_TO_TICKS(100));
                            rotate_left(); 
                            vTaskDelay(pdMS_TO_TICKS(100)); 
                        }
                        else if (mode == LEFT_TURN){
                            mazeEnd();
                        }
                    }
                    break;

                case RIGHT_TURN:
                    #ifdef DEBUG_SERIAL
                    Serial.println("right");
                    SerialBT.println("right");
                    #endif
                    
                    runExtraInch();
                    calculateError();
                    if (mode == FOLLOWING_LINE && (error > 2 || error < -2)){
                        runExtraInch();
                    }
                    else{
                        motorStop();                        
                        vTaskDelay(pdMS_TO_TICKS(100));
                        rotate_right();
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    break;     
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(25));

    }
}

void motor_control_core0(void* parameter){
    
    motor_configure();
    encoder_configure();

    while(1){
        vTaskDelay(pdMS_TO_TICKS(portMAX_DELAY));
    }
}


void setup(void)
{
    #ifdef DEBUG_SERIAL
    SerialBT.begin("ESP32_BT_Device"); // Start Bluetooth with a custom device name
    Serial.println("Bluetooth device is ready to pair.");
    #endif

    // motor control task with` priority 2 pinned to core 1
    xTaskCreatePinnedToCore(motor_control_core0, "motor_control", 10000, NULL, 2, &motor_control_core0_task, 0);

    // PID task with higher priority than main tasks
    xTaskCreatePinnedToCore(PID_fn, "PID_fn", 5000, NULL, 3, &PID_task, 0);

    left_queue = xQueueCreate(20, sizeof(int));
    right_queue = xQueueCreate(20, sizeof(int));

	pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SW_PIN, INPUT_PULLUP);
    pinMode(L_MIDDLE,INPUT);
    pinMode(LEFT_MIDDLE,INPUT);
    pinMode(RMIDDLE,INPUT);
    pinMode(RIGHT_MIDDLE,INPUT);
    pinMode(farLeftSensorPin,INPUT);
    pinMode(farRightSensorPin,INPUT);
    pinMode(16,INPUT);
    pinMode(39,INPUT);
    
    #ifdef DEBUG_SERIAL
    Serial.begin(9600);    
    #endif
    
    digitalWrite(LED_PIN, 0);
    digitalWrite(LED_BUILTIN, 1);
    
    digitalWrite(LED_BUILTIN, 0); // turn off Arduino's LED to indicate we are through with calibration

    delay(100);

    digitalWrite(LED_PIN, 1);


    while (digitalRead(SW_PIN)) { 
        #ifdef DEBUG_SERIAL
        // Serial.printf("%d %d %d %d %d %d, posn = %d\n", sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5], position);    
        delay(50);
        #endif
    }

    started = true;
}

int endFound = 0;

void loop(){

    #ifdef DEBUG_SERIAL
    // Serial.printf("L_Cur : %.3f R_Cur : %.3f Error : %d %d %d %d\n", ramp_values_L[ramp_index_L], ramp_values_R[ramp_index_R], error, analogRead(LEFT_MIDDLE), analogRead(MIDDLE), analogRead(RIGHT_MIDDLE));
    // Serial.printf("L_Cur : %.3f R_Cur : %.3f L_Duty : %.3f R_Duty : %.3f L_RPM : %.3f R_RPM : %.3f L_Int : %.3f R_Int : %.3f\n", ramp_values_L[ramp_index_L], ramp_values_R[ramp_index_R], L_duty, R_duty, (float)rptmL/1000.0, (float)rptmR/1000.0, PID_L.integrator, PID_R.integrator);
    // Serial.printf("%d %d %d %d %d %d\n", (analogRead(33) < 3700), (analogRead(32) < 100), (analogRead(35) < 100), (analogRead(34) < 100), (analogRead(39) <100), (analogRead(36) <100));
    // Serial.printf("%d %d %d %d %d\n", analogRead(farLeftSensorPin), analogRead(LEFT_MIDDLE), analogRead(MIDDLE), analogRead(RIGHT_MIDDLE), analogRead(farRightSensorPin));
    // Serial.println(digitalRead(farLeftSensorPin));
    #endif

    digitalWrite(LED_BUILTIN, 0); 
    digitalWrite(LED_PIN, 0);
    mazeSolve(); // First pass to solve the maze

    digitalWrite(LED_PIN, 1);
    digitalWrite(LED_BUILTIN, 1);
    
    SerialBT.println(path);

    #ifdef DEBUG_SERIAL
    // Serial.printf("%s\n", path);
    #endif
    
    while (digitalRead(SW_PIN)) {}

    status = 0;
}