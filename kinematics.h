// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#define CPR                     358.3       // counts per revolution
#define WHEEL_DIAMETER          32.         // mm
#define WHEEL_DISTANCE          85.         // mm, the distance between both wheels
#define ANGLE_PER_COUNT         2*PI/CPR    // rad
#define TRAVEL_PER_COUNT        WHEEL_DIAMETER*PI/CPR // mm/count
//#define r 16.5
//#define L 43.1


// Class to track robot position.
class Kinematics_c {
    public:
        // r = 16mm
        int32_t count_r_old=0;
        int32_t count_l_old=0; 
        float XIabs=0.;
        float YIabs=0.;
        float theta=0.;


        // Constructor, must exist.
        Kinematics_c() {
            count_r_old=0;
            count_l_old=0;
            XIabs=0.;
            YIabs=0.;
            theta=0.;
        }

        void reset() {
            YIabs=0.;
            XIabs=0.;
            theta=0.;
            count_r_old=count_r;
            count_l_old=count_l;
        }

        
        // Use this function to update
        // your kinematics
        void update(){
            if(count_r_old!=count_r or count_l_old!=count_l){
                double phi_r=((count_r-count_r_old)*ANGLE_PER_COUNT); // Right wheel Delta_rotation estimate
                double phi_l=((count_l-count_l_old)*ANGLE_PER_COUNT); // Left wheel Delta_rotation estimate
                
                double XR_dot=WHEEL_DIAMETER/4*(phi_r+phi_l);
                double theta_dot=WHEEL_DIAMETER/(2*WHEEL_DISTANCE)*(phi_r-phi_l);
                
                XIabs+=cos(theta)*XR_dot;
                YIabs+=sin(theta)*XR_dot;
                theta+=theta_dot;

                // The following is so that theta remains between [0,2∏[
                while(theta>=2*PI) theta-=2*PI;
                while(theta<0) theta+=2*PI;    
                
                count_r_old = count_r;
                count_l_old = count_l;
            }
        }
};



#endif
