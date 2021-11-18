#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "pid.h"

#define CPR                     358.3       // counts per revolution
#define WHEEL_DIAMETER          32.         // mm
#define WHEEL_DISTANCE          85.         // mm, the distance between both wheels
#define ANGLE_PER_COUNT         2*PI/CPR    // rad
#define TRAVEL_PER_COUNT        WHEEL_DIAMETER*PI/CPR // mm/count


#define FOLLOW_LINE_UPDATE      100         // ms
#define JOIN_LINE_UPDATE        100         // ms
#define SPEED_UPDATE            20          // ms
#define ROTATION_SPEED_UPDATE   9           // ms
#define CALIBRATION_TIME        2000        // ms   
#define EDGE_UPDATE             10          // ms

#define NUMBER_MEASUREMENTS     50

static bool measurements[NUMBER_MEASUREMENTS]={WHITE}; 
static uint32_t timings[2*NUMBER_MEASUREMENTS]={0}; 
static double sampling_position[NUMBER_MEASUREMENTS]={0.};
static uint8_t index=0;
static bool current_color=WHITE;

static LineSensor_c lineSensors;
static Motors_c motors;
static PID_c line_PID;
static PID_c speed_PID_l;
static PID_c speed_PID_r;

static double rotation_velocity_r=OFFSET_SPEED; // mm/s
static double rotation_velocity_l=OFFSET_SPEED; // mm/s

static int16_t speed_target_l=OFFSET_SPEED;
static int16_t speed_target_r=OFFSET_SPEED;

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
}

static void read_rotation_speeds(){
    static uint32_t current_ts_us, loop_duration_us, previous_ts_us = micros();
    static int32_t current_count_r, current_count_l, previous_count_r = count_r, previous_count_l = count_l;
    double rotation_speed_r=OFFSET_SPEED*0.8, rotation_speed_l=OFFSET_SPEED*0.8;
    
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

static compute_sampling_positions(){
    for(uint8_t i=0 ; i<index ; i++){
        sampling_position[i]=(double)(timings[2*i+1]-timings[2*i])/(timings[2*(i+1)]-timings[2*i]);
    }
}

static print_timings(){
    Serial.println("Timings are: ");
    for(uint8_t i=0; i < 2*index; i++){
        Serial.println(timings[i]);
    }
}

static print_sampling_positions(){
    for(uint8_t i=0 ; i<index ; i++){
        Serial.print("Sampling position of bit ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(sampling_position[i]);
    }
}

void setup(){
    Serial.begin(9600); // Start a serial connection
    delay(1500); // Wait for stable connection
    pinMode(LED_PIN, OUTPUT);
    
    setupEncoder0();
    setupEncoder1();
    setupBarCodeReader();
    lineSensors.initialise();
    motors.initialise();

    // PID intialisations: (Kp,Ki,Kd)
    line_PID.initialise(60, 0.1, 0.005);
    speed_PID_l.initialise(0.5, 0.5, 0.001 ); //0.5, 0.7, 0.001
    speed_PID_r.initialise(0.5, 0.5, 0.001);
    
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

    if(current_ts_ms - rsu_ts > ROTATION_SPEED_UPDATE and !read_bit) {
        read_rotation_speeds();
        rsu_ts=millis();
    }

    if(current_ts_ms - su_ts > SPEED_UPDATE and state != STATE_FAILED and !read_bit) {
        double update_signal_r=speed_PID_r.update(speed_target_r,rotation_velocity_r);
        double update_signal_l=speed_PID_l.update(speed_target_l,rotation_velocity_l);
//        Serial.print(speed_target_r);
//        Serial.print(" ");
//        Serial.print(rotation_velocity_r);
//        Serial.print(" ");
//        Serial.println(update_signal_r);
        motors.setRightMotorPower((int16_t)update_signal_r);
        motors.setLeftMotorPower((int16_t)update_signal_l);
        su_ts=millis();
    }

    if(current_ts_ms - su_ts > EDGE_UPDATE and state != STATE_FAILED and !read_bit) {
        bool color = lineSensors.numerical_measure();
        if(color!= current_color){
            timings[2*index] = millis();
            current_color=color;
        }      
    }
    
    switch(state){
        case STATE_FOLLOW_LINE:
            if(current_ts_ms - flu_ts > FOLLOW_LINE_UPDATE){
                lineFollowingBehaviour();
                flu_ts = millis();
            }
            if(!lineSensors.on_line()){
                state=STATE_READ_CODE;
                TCNT3=OCR3A/2;
                timings[index]=millis();
                speed_target_r=OFFSET_SPEED;
                speed_target_l=OFFSET_SPEED;
            }
            break;

        case STATE_READ_CODE:
            if(read_bit){ // only reads when the interrupt routine is called
                timings[2*index+1] = millis();
                boolean current_bit = lineSensors.numerical_measure();
                if(index<NUMBER_MEASUREMENTS){
                    if(index >0 && measurements[index-1]==current_bit){
                        state=STATE_FAILED;
                        speed_target_r=0;
                        speed_target_l=0;
                        motors.halt();
                        compute_sampling_positions();
                    }
                    else{
                        measurements[index]=current_bit;
                        index++;   
                    }
                }
                digitalWrite(13, current_bit);                
                read_bit = false;         
            }
            break;

        case STATE_FAILED:
            Serial.print("Last index =");
            Serial.println(index);
            print_sampling_positions();
            print_timings();
            delay(1000);

        case STATE_DEBUG:
            Serial.println(millis());
            delay(1000);
            break;
            
        default:
            break;
    }
    
}
