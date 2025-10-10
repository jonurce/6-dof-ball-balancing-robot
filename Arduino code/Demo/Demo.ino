
// libraries
#include "PololuMaestro.h"
#include "math.h"

//teensy
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  

// objects
MicroMaestro maestro(maestroSerial);


// --------------------------------------------------
// --------------------- TUNE: ----------------------

// TUNE: ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
// *should be less than 14 degrees since there are 25 teeth on the servo
float offset[6] = { 0.0, 0.0,    // channel #0 and channel #1
                    0.0, 0.0,    // channel #2 and channel #3
                    0.0, 0.0 };  // channel #4 and channel #5

// TUNE: HOME hx, hy, hz, roll, pitch, yaw
float hx_home = 0;
float hy_home = 0;
float hz_home = 120;
float roll_home = 0;
float pitch_home = 0;
float yaw_home = 0;

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

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);
}

void loop() {
  demo(50, 50); // try with different speeds and accelerations, respectively
  delay(1000);
  //circle(); // once demo works, try this one
  stop();
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

// demo
void demo(int speed, int acc){
  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Rotation: Positive Pitch");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home + 0.1, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Rotation: Negative Pitch");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home - 0.1, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Rotation: Positive Roll");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home + 0.1, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Rotation: Negative Roll");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home - 0.1, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Rotation: Positive Yaw");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home + 0.1);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Rotation: Negative Yaw");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home - 0.1);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Translation: Positive X");
  InverseKinematics(hx_home + 20, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Translation: Negative X");
  InverseKinematics(hx_home - 20, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);
  
  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Translation: Positive Y");
  InverseKinematics(hx_home, hy_home + 20, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Translation: Negative Y");
  InverseKinematics(hx_home, hy_home - 20, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Translation: Positive Z");
  InverseKinematics(hx_home, hy_home, hz_home + 10, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Translation: Negative Z");
  InverseKinematics(hx_home, hy_home, hz_home - 10, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);

  Serial.println("Moving to Home Position");
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
  delay(1000);
}

// circle
void circle(int speed, int acc){ 
  Serial.println("Testing circle motion in X-Y plane");
  for (float i = 0; i <= 2*PI; i = i + 0.05){
    InverseKinematics(hx_home + 20*cos(i), hy_home + 20*sin(i), hz_home, roll_home, pitch_home, yaw_home); // hx, hy, hz, roll, pitch yaw
    moveServos(speed, acc);
  }
  delay(1000);
  InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  // hx, hy, hz, roll, pitch yaw
  moveServos(speed, acc);
}