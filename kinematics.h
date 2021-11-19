// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#define r 16.5
#define L 43.1


// Class to track robot position.
class Kinematics_c {
  public:

    // r = 16mm
    float ThetaR;
    float XIabs;
    float YIabs;
    float RightWheelDeg;
    float LeftWheelDeg;

    float PrevRightWheelDeg;
    float PrevLeftWheelDeg;



    // Constructor, must exist.
    Kinematics_c() {

    }

    // Use this function to update
    // your kinematics
    void initialise() {
      RightWheelDeg = 0;
      LeftWheelDeg = 0;
      PrevRightWheelDeg = 0;
      PrevLeftWheelDeg = 0;


    }
    void update() {


    }
    

    void getInstantPosition() {
      float XR;
      RightWheelDeg = (float)count_r;
      LeftWheelDeg =  (float)count_l;

      float DifRightWheelDeg = RightWheelDeg - PrevRightWheelDeg;
      float DifLeftWheelDeg = LeftWheelDeg - PrevLeftWheelDeg;

      XR = (r * DifRightWheelDeg * PI / 180) / 2 + (r * DifLeftWheelDeg * PI / 180) / 2; // r = 16mm
      ThetaR = ((r * RightWheelDeg) / (2 * L) - (r * LeftWheelDeg) / (2 * L));
      XIabs += XR * cos(ThetaR * PI / 180);
      YIabs += XR * sin(ThetaR * PI / 180);


      PrevRightWheelDeg = RightWheelDeg;
      PrevLeftWheelDeg = LeftWheelDeg;
    }



    float getThetaR() {
      return ThetaR;
    }
    float getXIabs() {
      return XIabs;
    }
    float getYIabs() {
      return YIabs;
    }


};



#endif
