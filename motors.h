// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN           10
#define L_DIR_PIN           16
#define R_PWM_PIN           9
#define R_DIR_PIN           15
#define PWM_LIMIT           60
#define FWD                 LOW    // forward
#define BWD                 HIGH   // backward


// Class to operate the motors.
// Class keyword, followed by the assigned data type "Motor_c".
// This is what we will later use to declare instances of our class datatype.
class Motors_c {
    // This keyword means that all code below this point is accesible from outside the class.
    public:  
        Motors_c() {
        } 
      
        void setLeftMotorPower(float pwm) {
            if(pwm>=-PWM_LIMIT && pwm<=PWM_LIMIT){
                if(pwm<0){
                    digitalWrite(L_DIR_PIN, BWD);
                    analogWrite(L_PWM_PIN, -pwm);
                }
                else{
                    digitalWrite(L_DIR_PIN, FWD);
                    analogWrite(L_PWM_PIN, pwm);
                }
            }
            else if(pwm>PWM_LIMIT){
                digitalWrite(L_DIR_PIN, FWD);
                analogWrite(L_PWM_PIN, PWM_LIMIT);
            }
            else if(pwm<-PWM_LIMIT){
                digitalWrite(L_DIR_PIN, BWD);
                analogWrite(L_PWM_PIN, PWM_LIMIT);
            }
            else Serial.println("Invalide pwm value");
        }

        void setRightMotorPower(float pwm) {
            if(pwm>=-PWM_LIMIT && pwm<=PWM_LIMIT){
                if(pwm<0){
                    digitalWrite(R_DIR_PIN, BWD);
                    analogWrite(R_PWM_PIN, -pwm);
                }
                else{
                    digitalWrite(R_DIR_PIN, FWD);
                    analogWrite(R_PWM_PIN, pwm);
                }
            }
            else if(pwm>PWM_LIMIT){
                digitalWrite(R_DIR_PIN, FWD);
                analogWrite(R_PWM_PIN, PWM_LIMIT);
            }
            else if(pwm<-PWM_LIMIT){
                digitalWrite(R_DIR_PIN, BWD);
                analogWrite(R_PWM_PIN, PWM_LIMIT);
            }
            else Serial.println("Invalide pwm value");
        }

        void halt(){
             setLeftMotorPower(0);
             setRightMotorPower(0);
        }

        void advance(float pwm){
            setRightMotorPower(pwm);
            setLeftMotorPower(pwm);     
        }
        
        void rotate(float pwm){
            setRightMotorPower(pwm);
            setLeftMotorPower(-pwm);
        }

        // Use this function to initialise the pins and state of your motor(s).
        void initialise() {
            // Set all the motor pins as outputs.
            pinMode(L_PWM_PIN, OUTPUT);
            pinMode(L_DIR_PIN, OUTPUT);
            pinMode(R_PWM_PIN, OUTPUT);
            pinMode(R_DIR_PIN, OUTPUT);

            // Set initial direction (HIGH/LOW) for the direction pins.
            digitalWrite(L_DIR_PIN, FWD);
            digitalWrite(R_DIR_PIN, FWD);

            // Set initial values for the PWM pins.
            digitalWrite(L_PWM_PIN, LOW);
            digitalWrite(R_PWM_PIN, LOW);

            setLeftMotorPower(0);
            setRightMotorPower(0);
        }
};



#endif
