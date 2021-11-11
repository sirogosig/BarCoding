// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define NB_LS_PINS          3

#define LS_LEFT_PIN         18
#define LS_CENTRE_PIN       20
#define LS_RIGHT_PIN        21

#define RIGHT               true
#define LEFT                false

#define EMIT_PIN            11
#define TIME_OUT            4000    // After 4ms, report a time-out

#define LED_PIN             13          // Pin to activate the orange LED of the LED, and toggle it.
#define SAMPLING_TIME       BIT_SIZE/OFFSET_SPEED   // s
#define OFFSET_SPEED        90          // mm/s
#define BIT_SIZE            18.         // mm

#define STATE_INITIALISE        'I'
#define STATE_READ_CODE         'C'
#define STATE_FOLLOW_LINE       'L'
#define STATE_DEBUG             'D'
#define STATE_FINISHED          'F'

volatile boolean read_bit = false;
volatile boolean DEBUG_LED_STATE = false;
static char state=STATE_INITIALISE;


//The following could possibly be moved as a privte class variable:
static uint8_t ls_pin[NB_LS_PINS] = {LS_LEFT_PIN, LS_CENTRE_PIN, LS_RIGHT_PIN};


// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the 
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {
    if(state==STATE_READ_CODE){
        // Invert LED state
        DEBUG_LED_STATE = !DEBUG_LED_STATE;

        // Enable/disable LED
        digitalWrite(13, DEBUG_LED_STATE);
        TCNT3=OCR3A/2;
    
        read_bit=true;
    }
}


void setupBarCodeReader(){
    // disable global interrupts
    cli();          

    // Reset timer3 to a blank condition.
    // TCCR = Timer/Counter Control Register
    TCCR3A = 0;     // set entire TCCR3A register to 0
    TCCR3B = 0;     // set entire TCCR3B register to 0

    // First, turn on CTC mode.  Timer3 will count up
    // and create an interrupt on a match to a value.
    // See table 14.4 in manual, it is mode 4.
    TCCR3B = TCCR3B | (1 << WGM32);

    // For a cpu clock precaler of 256:
    // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
    // Table 14.5 in manual. 
    TCCR3B = TCCR3B | (1 << CS32);


    // set compare match register to desired timer count.
    // CPU Clock  = 16000000 (16mhz).
    // Prescaler  = 256
    // Timer freq = 16000000/256 = 62500
    // We can think of this as timer3 counting up to 62500 in 1 second.
    // compare match value = 62500 / 5 (we desire 5hz).
    OCR3A = 62500*SAMPLING_TIME;

    // enable timer compare interrupt:
    TIMSK3 = TIMSK3 | (1 << OCIE3A);

    // enable global interrupts:
    sei(); 
}



// Class to operate the linesensor(s).
class LineSensor_c {
    private:
        double ls_values_percentage[NB_LS_PINS];
        double ls_conditioned_data[NB_LS_PINS]; //Sensor time values

        void reset(){
            // Charge capacitor by setting input pin
            // temporarily to output and HIGH
            pinMode( LS_LEFT_PIN, OUTPUT );
            pinMode( LS_CENTRE_PIN, OUTPUT );
            pinMode( LS_RIGHT_PIN, OUTPUT );

            digitalWrite( LS_LEFT_PIN, HIGH );
            digitalWrite( LS_CENTRE_PIN, HIGH );
            digitalWrite( LS_RIGHT_PIN, HIGH );

            // Tiny delay for capacitor to charge.
            delayMicroseconds(10);

            //  Turn input pin back to an input
            pinMode( LS_LEFT_PIN, INPUT );
            pinMode( LS_RIGHT_PIN, INPUT );
            pinMode( LS_CENTRE_PIN, INPUT );
        }

        /*
         * Calculates the percentage values of the 3 central line sensors
         */
        void calc_values_percentage(){
            double total_conditioned_data = 0.;
            for(uint8_t i=0; i<NB_LS_PINS; i++){
                total_conditioned_data += ls_conditioned_data[i];
            }
            for(uint8_t i=0; i<NB_LS_PINS; i++){
                if(total_conditioned_data<0.01) ls_values_percentage[i]=0;
                else ls_values_percentage[i]=ls_conditioned_data[i] / total_conditioned_data;
            }    
        }

        /*
         * Calculates the conditioned date from raw data, thus taking into account calibration
         */
        void calc_conditioned_data(){
            for(uint8_t i=0;i<NB_LS_PINS;i++){
                if(ls_raw_data[i]<=ls_min_values[i]) ls_conditioned_data[i]=0;
                else{
                    ls_conditioned_data[i]=(ls_raw_data[i]-ls_min_values[i])*scaling_factors[i];    
                }
            }    
        }

    public:
        uint32_t ls_raw_data[NB_LS_PINS]; //Sensor time values
        uint32_t ls_max_values[NB_LS_PINS]; //max values for calibration
        uint32_t ls_min_values[NB_LS_PINS]; //min values for calibration
        double scaling_factors[NB_LS_PINS];
        bool turn_direction;

        // Constructor, must exist.
        LineSensor_c() {
            for(uint8_t i=0;i<NB_LS_PINS;i++){
                ls_values_percentage[i]=0.;
                ls_raw_data[i]=0;
                ls_conditioned_data[i]=0;
                ls_max_values[i]=0;
                ls_min_values[i]=2000;
                scaling_factors[i]=0.; 
                turn_direction=LEFT;  
            }
        } 

        void print_LS_raw_data(){
            Serial.print("Left line sensor raw value: " );
            Serial.println( ls_raw_data[0] );

            Serial.print("Center line sensor raw value: " );
            Serial.println(ls_raw_data[1] );

            Serial.print("Right line sensor raw value: " );
            Serial.println( ls_raw_data[2] );
        }

        void print_LS_conditioned_data(){
            Serial.print("Left line sensor conditioned value: " );
            Serial.println(ls_conditioned_data[0],6);

            Serial.print("Center line sensor conditioned value: " );
            Serial.println(ls_conditioned_data[1],6);

            Serial.print("Right line sensor conditioned value: " );
            Serial.println(ls_conditioned_data[2],6);
        }

        void print_LS_percentage_data(){
            Serial.print("Left line sensor percentage value: " );
            Serial.println(ls_values_percentage[0],6);

            Serial.print("Center line sensor percentage value: " );
            Serial.println(ls_values_percentage[1],6);

            Serial.print("Right line sensor percentage value: " );
            Serial.println(ls_values_percentage[2],6);
        }

        void print_LS_min_max_data(){
            Serial.println("Min values : ");
            Serial.println(ls_min_values[0]);
            Serial.println(ls_min_values[1]);
            Serial.println(ls_min_values[2]);
            
            Serial.println("Max values : ");
            Serial.println(ls_max_values[0]);
            Serial.println(ls_max_values[1]);
            Serial.println(ls_max_values[2]);   
        }

    
        // Initialises the pins and state of linesensorss
        void initialise(){
            pinMode(EMIT_PIN, OUTPUT );
            pinMode(LS_LEFT_PIN, INPUT );
            pinMode(LS_CENTRE_PIN, INPUT );
            pinMode(LS_RIGHT_PIN, INPUT );

            digitalWrite(EMIT_PIN, HIGH); //enables the IR LEDs
        }

        double getPositionError(){
            return (ls_values_percentage[NB_LS_PINS-1]-ls_values_percentage[0]);
        }

        /*
         * Detects if the frontmost sensor is on a black line
         */
        bool on_line(){
            measure();
            if(ls_conditioned_data[1]<0.7) return false;
            else return true;   
        }

        /*
         * Returns true if any of the 5 sensors detects a black line
         */
        bool line_detected(){
            measure();
            for(uint8_t i=0; i<NB_LS_PINS; i++){
                if(ls_conditioned_data[i]>0.2) return true;
            }
            return false; 
        }

        /*
         * Measures the raw, conditioned and percentage values for all the sensors
         */
        void measure(){
            uint8_t done=0; // The number of sensors that already have recorded a value
            // Places to store microsecond count:
            uint32_t ls_temp_times[NB_LS_PINS] = {0}; //initialised at 0 (if ≠ than 0, then it means a value has already been read)
            uint32_t start_time; // t_1
            
            reset();
            start_time = micros();// Store current microsecond count

            // Stay in a loop whilst the capacitor
            // is still registering as "HIGH".
            while(done<NB_LS_PINS && (micros()-start_time)<TIME_OUT){
                // Read all three sensors. Happens very quickly.
                // The while() above will repeat this process, until 
                // all the sensors have been read or timed out
                for(uint8_t i = 0; i < NB_LS_PINS; i++ ) { 
                    if( digitalRead(ls_pin[i] ) == LOW && ls_temp_times[i]==0) {
                        // Store the time elapsed for this sensor
                        ls_temp_times[i]=micros()-start_time;
                        done++;
                    }
                }
            }
            for(uint8_t i=0; i<NB_LS_PINS;i++){
                ls_raw_data[i] = ls_temp_times[i];
            }


            if(scaling_factors[NB_LS_PINS-1]!=0) {
                calc_conditioned_data(); //Calculate the conditioned data only if calibration has happenned
            }
            calc_values_percentage();  
            print_LS_raw_data();  
        }

        boolean numerical_measure(){
            bool done=false;
            uint32_t start_time; // t_1
            reset();
            start_time = micros();// Store current microsecond count
            while(!done && (micros()-start_time)<TIME_OUT){
                // Read all three sensors. Happens very quickly.
                // The while() above will repeat this process, until 
                // all the sensors have been read or timed out
                for(uint8_t i = 0; i < NB_LS_PINS; i++ ) { 
                    if( digitalRead(ls_pin[i] ) == LOW && ls_temp_times[i]==0) {
                        // Store the time elapsed for this sensor
                        ls_temp_times[i]=micros()-start_time;
                        done++;
                    }
                }
            }
            
            
        }
        
};


#endif
