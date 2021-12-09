//This is the kinematics branch
#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "pid.h"
#include "kinematics.h"

#define STRAIGHT_PID_UPDATE     100         // ms
#define SPEED_UPDATE            20          // ms

#define KINEMATCS_UPDATE        30          // ms
#define SPEED_READING_UPDATE    9           // ms
#define CALIBRATION_TIME        800        // ms   
#define EDGE_UPDATE             10          // ms

#define SPEED_SWITCH            3000        // ms
#define SPEED1                  60
#define SPEED2                  140

#define NUMBER_MEASUREMENTS     50

static bool measurements[NUMBER_MEASUREMENTS]={WHITE}; 
static double sampling_point[NUMBER_MEASUREMENTS] = {0.};
static uint8_t index=0;
static bool current_color=WHITE;

static double sampling_distance[NUMBER_MEASUREMENTS] = {0.};
static double edges_distance[NUMBER_MEASUREMENTS] = {0.};

static LineSensor_c lineSensors;
static Motors_c motors;
static PID_c straight_PID; // used for the line-following AND for barcode reading in a straight line
static PID_c speed_PID_l;
static PID_c speed_PID_r;
static Kinematics_c kinematics;

static double rotation_velocity_r = OFFSET_SPEED; // mm/s
static double rotation_velocity_l = OFFSET_SPEED; // mm/s

static int16_t speed_target_l = OFFSET_SPEED;
static int16_t speed_target_r = OFFSET_SPEED;

//spu = Straight PID Update, su = Speed Update, sru = Speed Rotation Update, eu = Edge Update, ku = Kinematics Update :
static uint32_t spu_ts=0, su_ts=0, sru_ts=0, eu_ts=0, ku_ts=0, pidu_ts=0;

/*
 * The calibration function makes the robot advance for a given time, during which it samples
 * the ground and stores the extreme values that will thereafter be considered as pure
 * white and pure black.
 */
static void calibrate(){
    const uint32_t initial_ts = millis();
    motors.advance(OFFSET_SPEED/5); // The division by 5 is an approximation of the relation between speed (mm/s) and pwm power
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
    double feedback_signal_line=straight_PID.update(0,lineSensors.getPositionError());

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
    if(loop_duration_us!=0){
        rotation_speed_r=(current_count_r-previous_count_r)/((float)loop_duration_us)*1000000*TRAVEL_PER_COUNT;
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

static compute_sampling_point(){
    for(uint8_t i = 0 ; i < index ; i++){
        sampling_point[i]=(float)(sampling_distance[i]-edges_distance[i])/(edges_distance[i+1]-edges_distance[i]);
    }
}

static print_sampling_distance(){
    Serial.println(" ");
    Serial.println("Sampling distances are: ");
    for(uint8_t i=0; i <= index; i++){
        Serial.print("Sampling distance of bit ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(sampling_distance[i]);
    }
}

static print_edges_distance(){
    Serial.println(" ");
    Serial.println("Edges distances are: ");
    for(uint8_t i=0; i <= index; i++){
        Serial.print("Distance of the edge of bit ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(edges_distance[i]);
    }   
}

static print_sampling_point(){
    Serial.println(" ");
    for(uint8_t i=0 ; i<index ; i++){
        Serial.print("Sampling position of bit ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(sampling_point[i]);
    }
}

static print_raw_sampling_point(){
    for(uint8_t i=0 ; i<index ; i++){
        delay(100);
        Serial.println(sampling_point[i]);
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
    straight_PID.initialise(60, 0.1, 0.005);
    speed_PID_l.initialise(0.5, 0.5, 0.001); //older: 0.5, 0.7, 0.001 
    speed_PID_r.initialise(0.5, 0.5, 0.001);  //old: 0.7, 0.9, 0.00
    
    calibrate();

    
    state=STATE_FOLLOW_LINE;
    straight_PID.reset();
    speed_PID_l.reset();
    speed_PID_r.reset();

    Serial.println("***RESET***");
}


void loop(){
    static float reading_time;
    static uint32_t begin_reading_ts;
    static uint32_t current_ts_ms;
    current_ts_ms = millis();
    
    if(current_ts_ms - sru_ts > SPEED_READING_UPDATE) {
        read_rotation_speeds();
        sru_ts=millis();
    }

    if(current_ts_ms - su_ts > SPEED_UPDATE and state != STATE_FAILED) {
        double update_signal_r=speed_PID_r.update(speed_target_r,rotation_velocity_r);
        double update_signal_l=speed_PID_l.update(speed_target_l,rotation_velocity_l);
        motors.setRightMotorPower((int16_t)update_signal_r);
        motors.setLeftMotorPower((int16_t)update_signal_l);
        su_ts=millis();
    }
    
    switch(state){
        case STATE_FOLLOW_LINE:
            if(current_ts_ms - spu_ts > STRAIGHT_PID_UPDATE){
                lineFollowingBehaviour();
                spu_ts = millis();
            }
            if(!lineSensors.on_line()){
                kinematics.reset();              
                speed_target_r=OFFSET_SPEED;
                speed_target_l=OFFSET_SPEED;
                begin_reading_ts=millis();
                state=STATE_READ_CODE;
            }
            break;
            
        case STATE_READ_CODE:
            if(current_ts_ms - begin_reading_ts > 2000){
                motors.halt();
                speed_target_l=0;
                speed_target_r=0;
                reading_time=millis()-begin_reading_ts;
                state=STATE_FAILED; // 144 mm at 60mm/s  and he thinks he did 127 (2 seconds) --> Delta_v = 8.5 mm/s
                                    // 288 mm at 140mm/s and he thinks he did 269 (2 seconds) --> Delta_v = 9.5 mm/s
                                    // 352 mm at 220mm/s and he thinks he did 332 (1.5 seconds) --> Delta_v = 13.3 mm/s
                                    // Error in speed constant
            }

            if(current_ts_ms - ku_ts > KINEMATCS_UPDATE ) {
                kinematics.update();
                ku_ts=millis();
            }
            
            if(current_ts_ms - spu_ts > STRAIGHT_PID_UPDATE){ // 10 Hz
                double feedback_signal_line=straight_PID.update(0,kinematics.theta);

                speed_target_l=OFFSET_SPEED - feedback_signal_line;
                speed_target_r=OFFSET_SPEED + feedback_signal_line;
                
                spu_ts = millis();
            }
               
            break;

        case STATE_FAILED:
            Serial.println("---------------------------------------");
            Serial.print("Reading time = ");
            Serial.println(reading_time);
             Serial.print("Kinematics XIabs = ");
            Serial.println(kinematics.XIabs);

            delay(1000);
            break;

        case STATE_DEBUG:
            break;
            
        default:
            break;
    }
}
