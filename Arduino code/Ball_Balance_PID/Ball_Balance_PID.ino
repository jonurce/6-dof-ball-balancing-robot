
// libraries
#include "PololuMaestro.h"
#include "math.h"
#include <Pixy2SPI_SS.h>

//teensy
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  

// --------------------------------------------------
// --------------------- TUNE: ----------------------

// TUNE: ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
// *should be less than 14 degrees since there are 25 teeth on the servo
float offset[6] = { 0.0, 0.0,    // channel #0 and channel #1
                    0.0, 0.0,    // channel #2 and channel #3
                    0.0, 0.0 };  // channel #4 and channel #5

// TUNE: INPUTS FROM CAMERA CALIBRATION
float origin[2] = { 0, 0 };  // X and Y of the center of the platform in the camera frame - should be around (140, 110)
float r_platform = 0;   // distance from the center of the platform to the corner of the platform seen from the pixy2 cam - should be around 120

// TUNE: HOME hx, hy, hz, roll, pitch, yaw
float hx_home = 0;
float hy_home = 0;
float hz_home = 120;
float roll_home = 0;
float pitch_home = 0;
float yaw_home = 0;

// TUNE: MAX AVERAGE ANGLE CHANGE (PITCH AND ROLL) IN ONE STEP - RADIANS
float r_max = 0.1;  

// TUNE: PID CONSTANTS VALUES
float kp = 0;
float kd = 0;
float ki = 0;

// TUNE: MOTOR VELOCITIES AND ACCELERATIONS
int vel_home = 20;
int acc_home = 20;
int vel_control = 80;
int acc_control = 80;

// ------------------- END TUNE -------------------
// ------------------------------------------------


// **OBJECTS**
Pixy2SPI_SS pixy;
MicroMaestro maestro(maestroSerial);  // if using the Maestro Micro controller

//**CONSTANTS**
float abs_0 = 4000;       //ms position of absolute 0 degrees
float abs_90 = 8000;      //ms position of absolute 90 degrees
float toDeg = 180 / PI;   // radians to degrees conversion factor
int x = 0, y = 1, z = 2;  // defines x, y, and z array indexes to be used

// ANGLE RANGE FOR EACH SERVO CCW
// Each servo can move from absolute 0 to absolute 90 degrees
// This defines a different range such as 90 to 180 degrees
float range[6][2] = { { -45, 45 }, { 45, -45 },  // a1, a2
                      { -45, 45 }, { 45, -45 },  // b1, b2
                      { -45, 45 }, { 45, -45 }};  // c1, c2


// ------------------------------------------------
//**INVERSE KINEMATICS CONSTANTS**
// write here...
// ------------------------------------------------

// calculated theta values a1, a2, b1, b2, c1, c2 respectively
float theta[6];     

// **PIXY2 CAMERA**
float ball[2];// X and Y co-ords of the ball in camera frame

// **PID**
float error[2];       // error of the ball
float error_prev[2];  // previous error value used to calculate derivative. Derivative = (error-previous_error)/(change in time)
float deriv[2];       // derivative of the error
float integral[2] = {0, 0}; // Initialize integral (x, y components)
float out[2];         // output values (pitch and roll)
float time_i;         // initial time
float time_f;         // final time
float delta_time;     // change in time (rename from 'time')

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);
  pixy.init();
}

void loop() {
  findBall();
  // if ball position (x and y) is 4004, then the ball is not detected, and platform should be in home position
  if (ball[x] == 4004 && ball[y] == 4004) {
    InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
    moveServos(vel_home, acc_home);
  } else {
    PID();  // calculates the proportional and derivative terms and outputs them to the platform
  }
}

// move all servos to their calculated positions at a certain speed and acceleration value
void moveServos(int spd, int acc) {  
  float pos;
  for (int i = 0; i < 6; i++) {
    maestro.setSpeed(i, spd);                                 //sets input speed to servo i
    maestro.setAcceleration(i, acc);                          //sets input acceleration to servo i
    pos = theta[i] + offset[i];                               //adds offset amount to the calculated angle (degrees)
    pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
    maestro.setTarget(i, pos);                                //drives motor to calculated position
  }
}

// end the program and stops all of the servos
void stop() {  
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, 0);  //stops servo i
  }
  while (1) {}
}

// find the location of the ball using the pixy2 cam
void findBall() { 
  pixy.ccc.getBlocks();

  // If there is 1 ball detected, collect the data
  if (pixy.ccc.numBlocks == 1) {
    ball[x] = pixy.ccc.blocks[0].m_x;  // X location of the ball in camera frame
    ball[y] = pixy.ccc.blocks[0].m_y;  // Y location of the ball in camera frame
  }
  // If there are multiple balls detected, print and set (x,y) to 4004
  else if (pixy.ccc.numBlocks > 1) {
    Serial.println("MULTIPLE BALLS DETECTED");
    ball[x] = 4004;
    ball[y] = 4004;
  }
  // If there is no ball detected, print and set (x,y) to 4004
  else {
    // Serial.println("NO BALL DETECTED");
    ball[x] = 4004;
    ball[y] = 4004;
  }
}

// calculates the Proportional, Derivative an Integral values and moves the servos
void PID() {  
 
  // error = vector pointing from the center of the platform to the center of the ball
  error[x] = ball[x] - origin[x];  // x component of error (origin x-axis and camera x-axis are the same)
  error[y] = -(ball[y] - origin[y]);  // y component of error (origin y-axis and camera y-axis are opposite)

  time_f = millis();
  delta_time = time_f - time_i;
  time_i = millis();

  // x and y components of derivative
  deriv[x] = (error[x] - error_prev[x]) / delta_time; 
  deriv[y] = (error[y] - error_prev[y]) / delta_time; 

  // trapezoidal rule for integral values of x and y
  integral[x] += (error[x] + error_prev[x]) * delta_time / 2.0; 
  integral[y] += (error[y] + error_prev[y]) * delta_time / 2.0; 

  // check if derivative is NaN or INF. If so, set to zero
  if (isnan(deriv[x]) || isinf(deriv[x])) {
    deriv[x] = 0;
  }
  if (isnan(deriv[y]) || isinf(deriv[y])) {
    deriv[y] = 0;
  }

  // set previous error to current error
  error_prev[x] = error[x]; 
  error_prev[y] = error[y];

  // calculates the distance from the center of the platfrom to the ball
  float r_ball = sqrt(pow(error[x], 2) + pow(error[y], 2));
  
  // if ball to far away from center, it's out of the platform -> move to home position
  if (r_ball > r_platform) {
    Serial.println("Detected ball out of platform");
    InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
    moveServos(vel_home, acc_home);
  }
  // if ball is inside the platform -> PID control -> set motor andgles (thetas) to move the motors
  else {  
    out[x] = error[x] * kp + deriv[x] * kd + integral[x] * ki;
    out[y] = error[y] * kp + deriv[y] * kd + integral[y] * ki;

    // average of the out angle (pitch and roll)
    float r_out = sqrt(pow(out[x], 2) + pow(out[y], 2));
    // if r_out is greater than r_max, scale down out values
    if (r_out > r_max) {   
      out[x] = out[x] * (r_max / r_out);
      out[y] = out[y] * (r_max / r_out);
    }

    // compute thetas with IK
    Serial.println("Performing PID control");
    // out[x] proportional to x-axis -> to achieve linear movement in x(+) -> pitch(+) rotation
    // out[y] proportional to y-axis -> to achieve linear movement in y(+) -> roll(-) rotation
    InverseKinematics(hx_home, hy_home, hz_home, roll_home - out[y], pitch_home + out[x], yaw_home);  //hx, hy, hz, roll, pitch, yaw
    
    // move platform
    moveServos(vel_control, acc_control);
  }
}

// compute inverse kinematics
void InverseKinematics(float hx, float hy, float hz, float roll, float pitch, float yaw) { 
  // -------------------------------------------------
  // TODO: CALCULATE MOTOR ANGLES (theta[i]) GIVEN hx, hy, hz, roll, pitch, yaw
  // write here...
  // -------------------------------------------------

  // error handle after motor angles are calculated
  for (int i = 0; i < 6; i++) { 

    // change theta sign (negative up) 
    theta[i] = -theta[i];

    //checks theta values are between -40 and 40 degrees
    if (abs(theta[i]) > 40) {
      Serial.println("ERROR: CURRENT VALUES EXCEED ANGLE RANGE");
      Serial.println("Motor index:");
      Serial.println(i);
      Serial.println("Motor angle value:");
      Serial.println(theta[i]);
      stop();
    }

    //checks theta values are real numbers
    if (isnan(theta[i])) {
      Serial.println("ERROR: CURRENT VALUES CANNOT PHYSICALLY BE EXECUTED");
      Serial.println("Motor index:");
      Serial.println(i);
      Serial.println("Motor angle value:");
      Serial.println(theta[i]);
      stop();
    }

  }
}