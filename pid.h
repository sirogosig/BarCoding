// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H


// Class to contain generic PID algorithm:
class PID_c {
    private:
        float Kp=0.;
        float Ki=0.;
        float Kd=0.;
        double total_error=0.;
        uint32_t previous_ts=0;
        
    public:
        double error=0;
        double previous_error=0;

        PID_c() {
            Kp=0;
            Ki=0;
            Kd=0;
            error=0;
            total_error=0;
            previous_error=0;
            previous_ts=0;
        } 
        
        void reset(){
            total_error=0.;
            previous_ts=millis();
            previous_error = 0;
        }
        
        void initialise(float Kp_, float Ki_, float Kd_){
            Kp=Kp_;
            Ki=Ki_;
            Kd=Kd_;
        }

        double update(double demand, double measurement){
            uint32_t current_ts=millis();
            double elapsed_time=0., feedback_signal=0.; // Times in seconds
            if((current_ts-previous_ts)<=0) elapsed_time=1./1000;
            else elapsed_time = (current_ts-previous_ts)*1./1000;
            error = demand - measurement;
            total_error += error * elapsed_time;
            
            if(Ki * total_error > 200.) total_error = 200./Ki;    //Anti wind-up
            if(Ki * total_error < -200.) total_error = -200./Ki;  //Anti wind-up
            
            feedback_signal= (Kp*error+Ki*total_error+Kd*(error-previous_error)/elapsed_time);
            previous_error=error;
            previous_ts=current_ts;
            return feedback_signal;
        }

        void print_errors(){
            Serial.print("Error = ");
            Serial.println(error,5);
            Serial.print("Total error = ");
            Serial.println(total_error,5);
        }
};



#endif
