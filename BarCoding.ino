#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "pid.h"
//#include <math.h>

#define LED_PIN                 13          // Pin to activate the orange LED of the LED, and toggle it.
#define SAMPLING_TIME           BIT_SIZE/(1000*OFFSET_SPEED)   // ms
#define OFFSET_SPEED            90          // mm/s
#define BIT_SIZE                18.         // mm

#define CPR                     358.3       // counts per revolution
#define WHEEL_DIAMETER          32.         // mm
#define WHEEL_DISTANCE          85.         // mm, the distance between both wheels
#define ANGLE_PER_COUNT         2*PI/CPR    // rad
#define TRAVEL_PER_COUNT        WHEEL_DIAMETER*PI/CPR // mm/count

#define FOLLOW_LINE_UPDATE      100         // ms
#define JOIN_LINE_UPDATE        100         // ms
#define SPEED_UPDATE            20          // ms
#define ROTATION_SPEED_UPDATE   9           // ms
#define CALIBRATION_TIME        3000        //ms  

#define STATE_INITIALISE        'I'
#define STATE_READ_CODE         'C'
#define STATE_FOLLOW_LINE       'L'
#define STATE_DEBUG             'D'
#define STATE_FINISHED          'F' 

static LineSensor_c lineSensors;
static Motors_c motors;
static PID_c line_PID;
static PID_c speed_PID_l;
static PID_c speed_PID_r;

static double rotation_velocity_r=0;
static double rotation_velocity_l=0;

static char state=STATE_INITIALISE;

static int16_t speed_target_l=OFFSET_SPEED_FAST;
static int16_t speed_target_r=OFFSET_SPEED_FAST;

 //flu = Follow Line Update, sb = Starting Behaviour, pidu= PID Update, su = Speed Update, rsu = Rotation Speed Update:
static uint32_t flu_ts=0, sb_ts=0, pidu_ts=0, su_ts=0, rsu_ts=0;

/*
 * The calibration function makes the robot advance for a given time, during which it samples
 * the ground and stores the extreme values that will thereafter be considered as pure
 * white and pure black.
 */
static void calibrate(){
    const uint32_t initial_ts = millis();
    motors.advance(20);
    while(millis() - initial_ts < CALIBRATION_TIME){
        lineSensors.measure();
        for(uint8_t i=0; i<NB_LS_PINS; i++){
            if(lineSensors.ls_raw_data[i] > lineSensors.ls_max_values[i]) lineSensors.ls_max_values[i]=lineSensors.ls_raw_data[i];
            if(lineSensors.ls_raw_data[i] < lineSensors.ls_min_values[i]) lineSensors.ls_min_values[i]=lineSensors.ls_raw_data[i];
        }
        delay(30);
    }
    motors.halt();

    // Calculate the scaling factors S:
    for(uint8_t i=0; i<NB_LS_PINS; i++){
        lineSensors.scaling_factors[i]=1./(lineSensors.ls_max_values[i]-lineSensors.ls_min_values[i]);    
    }
}

static void lineFollowingBehaviour(){
    lineSensors.measure();  // Conducts a read of the line sensors
    double feedback_signal_line=line_PID.update(0,lineSensors.getPositionError());

    speed_target_l=OFFSET_SPEED - feedback_signal_line;
    speed_target_r=OFFSET_SPEED + feedback_signal_line;
    Serial.println(feedback_signal_line);
}

static void read_rotation_speeds(){
    static uint32_t current_ts_us, loop_duration_us, previous_ts_us = micros();
    static int32_t current_count_r, current_count_l, previous_count_r = count_r, previous_count_l = count_l;
    double rotation_speed_r=0, rotation_speed_l=0;
    
    current_ts_us = micros();
    current_count_r=count_r;
    current_count_l=count_l;

    loop_duration_us=current_ts_us-previous_ts_us;
    if(current_count_r!=previous_count_r && loop_duration_us!=0){
        rotation_speed_r=(current_count_r-previous_count_r)/((float)loop_duration_us)*1000000*TRAVEL_PER_COUNT;
    }
    if(current_count_l!=previous_count_l && loop_duration_us!=0){
        rotation_speed_l=(current_count_l-previous_count_l)/((float)loop_duration_us)*1000000*TRAVEL_PER_COUNT;
    }

    float new_value_weight = 0.3;  // value between 0:1
    // Update low pass filter
    rotation_velocity_r = (rotation_velocity_r * (1 - new_value_weight)) + (rotation_speed_r * new_value_weight);
    rotation_velocity_l = (rotation_velocity_l * (1 - new_value_weight)) + (rotation_speed_l * new_value_weight);

    previous_ts_us=current_ts_us;
    previous_count_r=current_count_r;
    previous_count_l=current_count_l;
}

void setup(){
    Serial.begin( 9600 ); // Start a serial connection
    delay(1500); // Wait for stable connection
    pinMode(LED_PIN, OUTPUT);
    
    setupEncoder0();
    setupEncoder1();
    lineSensors.initialise();
    motors.initialise();

    // PID intialisations: (Kp,Ki,Kd)
    line_PID.initialise(60, 0.15, 0.01);
    speed_PID_l.initialise(0.5, 0.7, 0.001);
    speed_PID_r.initialise(0.5, 0.7, 0.001);
    
    calibrate();
    
    state=STATE_FOLLOW_LINE;
    line_PID.reset();
    speed_PID_l.reset();
    speed_PID_r.reset();

    Serial.println("***RESET***");
}


void loop(){
    static uint32_t current_ts_ms;
    current_ts_ms = millis();

    if(current_ts_ms - rsu_ts > ROTATION_SPEED_UPDATE) {
        read_rotation_speeds();
        rsu_ts=millis();
    }

    if(current_ts_ms - su_ts > SPEED_UPDATE and state != STATE_FINISHED) {
        double update_signal_r=speed_PID_r.update(speed_target_r,rotation_velocity_r);
        double update_signal_l=speed_PID_l.update(speed_target_l,rotation_velocity_l);
        motors.setRightMotorPower((int16_t)update_signal_r);
        motors.setLeftMotorPower((int16_t)update_signal_l);
        su_ts=millis();
    }

    switch(state){
        case STATE_FOLLOW_LINE:
            if(current_ts_ms - flu_ts > FOLLOW_LINE_UPDATE){
                lineFollowingBehaviour();
                flu_ts = millis();
            }
            break;

        case STATE_DEBUG:
            break;
            
        default:
            break;
    }
}
