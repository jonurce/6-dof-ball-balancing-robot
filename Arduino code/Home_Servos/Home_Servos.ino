  
// libraries
#include "PololuMaestro.h"

//teensy
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN 

MicroMaestro maestro(maestroSerial);

//**CONSTANTS**
float abs_0 = 4000;   //ms position of absolute 0 degrees
float abs_90 = 8000;  //ms position of absolute 90 degrees

//**USER DEFINED VALUES**

// ANGLE RANGE FOR EACH SERVO CCW
// Each servo can move from absolute 0 to absolute 90 degrees
// This defines a different range such as 90 to 180 degrees
float range[6][2] = { { -45, 45 }, { 45, -45 },  // a1, a2
                      { -45, 45 }, { 45, -45 },  // b1, b2
                      { -45, 45 }, { 45, -45 }};  // c1, c2

// --------------------------------------------------
// --------------------- TUNE: ----------------------
// ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
// *should be less than 14 degrees since there are 25 teeth on the servo
float offset[6] = { 0.0, 0.0,    // channel #0 and channel #1
                    0.0, 0.0,    // channel #2 and channel #3
                    0.0, 0.0 };  // channel #4 and channel #5
// --------------------------------------------------

// X and Y co-ords of the ball
float X, Y; 

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);

  // moves servos to 0 degree position
  int pos;
  for(int i = 0; i < 6; i++){
    maestro.setSpeed(0, 0); //sets speed to servo i
    maestro.setAcceleration(0, 0); //setsacceleration to servo i
    pos = map(offset[i], range[i][0], range[i][1], abs_0, abs_90);  //finds servo 0 degree position in microseconds
    maestro.setTarget(i, pos); //drives motor to calculated position
  }
}

void loop() {  
}
