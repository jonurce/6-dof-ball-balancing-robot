/*               ****NOTES****
  Before using this, go to the Serial Settings
  tab in the Maestro Control Center and apply these settings:

   Serial mode: UART, fixed baud rate
   Baud rate: 9600
   CRC disabled

  Be sure to click "Apply Settings" after making any changes.

  Array Indexes .> a1 = 0 / a2 = 1 / b1 = 2 / b2 = 3 / c1 = 4 / c2 = 5

  range for hz (105, 140)

*/

// libraries
#include "PololuMaestro.h"
#include "math.h"
#include <Pixy2SPI_SS.h>
#include <ArduinoEigen.h>

#define maestroSerial SERIAL_PORT_HARDWARE_OPEN  //for teensy

// --------------------------------------------------
// ------------------- START TUNE -------------------

// TUNE: ANGLE OFFSET FOR EACH MOTOR [NEGATIVE UP / POSITIVE DOWN] - DEGREES
// *should be less than 14 degrees since there are 25 teeth on the servo
float offset[6] = { -0.5, 7,    // channel #0 and channel #1
                    -2, 4,    // channel #2 and channel #3
                    -2.5, 5.5 };  // channel #4 and channel #5

// TUNE: INPUTS FROM CAMERA CALIBRATION
float origin[2] = { 144, 108 };  // X and Y co-ords of the  (in the camera frame)
float r_platform = 115;   // (121) distance from the center of the platform to the corner of the platform seen from the pixy2 cam


// TUNE: HOME hx, hy, hz, roll, pitch, yaw
float hx_home = 0;
float hy_home = 0;
float hz_home = 120;// 118.19374266158451;
float roll_home = 0;
float pitch_home = 0;
float yaw_home = 0;

// TUNE: MAX AVERAGE ANGLE CHANGE (PITCH AND ROLL) IN ONE STEP - DEGREES
float r_max = 15;  

// TUNE: LQR CONSTANTS VALUES
float q_pos = 1.0; // penalize for position error
float q_vel = 1.0; // penalize for velocity error
float r = 0.1; // penalize for angles
const int N = 20;

// TUNE: Low pass filter constant T
float T = 5.0; //0.08 (dt is around 0.02)

// TUNE: MOTOR VELOCITIES AND ACCELERATIONS
int vel_home = 20;
int acc_home = 20;
int vel_control = 60;
int acc_control = 60;

// TUNE: INPUTS FROM MEASUREMENTS (mm)
float l0 = 73.025; // from h0 to a0
float d1 = 36.8893; // from a0 to a01
float lf = 67.775; // from hf to midle af1-cf2
float m = 12.7; // from previous middle point to af1
float p1 = 31.75; // motor small arm
float p2 = 129; // motor long arm

// ------------------- END TUNE -------------------
// ------------------------------------------------


// **OBJECTS**
Pixy2SPI_SS pixy;
MicroMaestro maestro(maestroSerial);  // if using the Maestro Micro controller

//**CONSTANTS**
float abs_0 = 4000;                                     //ms position of absolute 0 degrees
float abs_90 = 8000;                                    //ms position of absolute 90 degrees
float toDeg = 180 / PI;                                 // radians to degrees conversion factor
float toRad = 1 / toDeg;                                // degrees to radians conversion factor
int x = 0, y = 1, z = 2;                                // defines x, y, and z array indexes to be used
String ID[6] = { "a1", "a2", "b1", "b2", "c1", "c2" };  //servo ID's

// ANGLE RANGE FOR EACH SERVO CCW
// each servo can move from absolute 0 to absolute 90 degrees
// this defines a different range such as 90 to 180 degrees
float range[6][2] = { { -45, 45 }, { 45, -45 },  // a1, a2
                      { -45, 45 }, { 45, -45 },  // b1, b2
                      { -45, 45 }, { 45, -45 }};  // c1, c2


//**INVERSE KINEMATICS**
float u = sqrt(pow(l0, 2) + pow(d1, 2)) * sin((2 * PI / 3) - atan(l0 / d1));  //(2*pi/3 is 120 degrees)

// calculates points a10, a20, b10, b20, c10, and c20
float a10[3] = { -d1, -l0, 0 };
float a20[3] = { -a10[x], a10[y], 0 };
float b10[3] = { l0*sqrt(3.0f)/2.0f + d1/2.0f, l0/2.0f - d1*sqrt(3.0f)/2.0f, 0 };
float b20[3] = { l0*sqrt(3.0f)/2.0f - d1/2.0f, l0/2.0f + d1*sqrt(3.0f)/2.0f, 0 };
float c10[3] = { -b20[x], b20[y], 0 };
float c20[3] = { -b10[x], b10[y], 0 };

// calculates vectors a_10_20, b_10_20, c_10_20,
float a_10_20[3] = { a20[x] - a10[x], a20[y] - a10[y], a20[z] - a10[z] };
float b_10_20[3] = { b20[x] - b10[x], b20[y] - b10[y], b20[z] - b10[z] };
float c_10_20[3] = { c20[x] - c10[x], c20[y] - c10[y], c20[z] - c10[z] };

//calculates normal vectors na, nb, and nc for each side of the triangle
float na[3] = { 0, -1, 0 }; // pointing out
float nb[3] = { sqrt(3.0f)/2.0f, 1/2.0f, 0 }; // pointing out
float nc[3] = { -sqrt(3.0f)/2.0f, 1/2.0f, 0 }; // pointing out

float t = lf/2.0f + m*sqrt(3.0f)/2.0f;
float w = lf*sqrt(3.0f)/2.0f - m/2.0f;

// calculated theta values a1, a2, b1, b2, c1, c2 respectively
float theta[6];                      
float theta_prev[6];

// **PIXY2 CAMERA**
float ball[2];// X and Y co-ords of the ball

// **LQR**
Eigen::Matrix<float, 4, 4> Q;
Eigen::Matrix<float, 2, 2> R;
float g = 9.8e3; // gravity mm/s^2

// A = inv(I-dt*Ac)
Eigen::Matrix<float, 4, 4> Ac, Act, A, Atemp;
Eigen::Matrix<float, 4, 4> I = Eigen::Matrix<float, 4, 4>::Identity();

// B = A*dt*Bc
Eigen::Matrix<float, 4, 2> Bc, B;

// state_error = [error_x, error_y, error_x_dot, error_y_dot]
Eigen::Vector4f state_error = Eigen::Vector4f::Zero();

Eigen::Matrix<float, 4, 4> P, At, temp1, temp4;
Eigen::Matrix<float, 4, 2> temp2;
Eigen::Matrix<float, 2, 2> temp3, temp3_inv, temp6, temp6_inv;
Eigen::Matrix<float, 2, 4> K, Bt, Bct, temp5;

// u_star[2] = [pitch, roll] in radians
Eigen::Vector2f u_star;

// u[2] = [roll, pitch] in degrees
float angles[2] = {0, 0};

float time_i;         // initial time
float time_f;         // final time
float delta_time;     // change in time (rename from 'time')

void setup() {
  Serial.begin(115200);
  //maestroSerial.begin(9600);
  // maestroSerial.begin(115200);
  maestroSerial.begin(250000);
  pixy.init();

  Q << q_pos, 0, 0, 0,
    0, q_pos, 0, 0,
    0, 0, q_vel, 0,
    0, 0, 0, q_vel;
  
  R << r, 0,  
      0, r;

  Ac << 0, 0, 1, 0,
    0, 0, 0, 1,
    0, 0, 0, 0,
    0, 0, 0, 0;
  
  // Bc initialization
  Bc << 0, 0,
    0, 0,
    0 , 0,
    0, 0; 

}

void loop() {
  findBall();  // finds the location of the ball
  if (ball[x] == 4004 && ball[y] == 4004) {
    // sees if ball position (x and y) is 4004, if so then the ball is not detected and platform should be in home position
    InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
    moveServos(vel_home, acc_home);
  } else {
    LQR();
  }
}

// move servo i to an input position at a certain speed and acceleration value
void moveServo(int i, float pos, int spd, int acc) {        
  pos = pos + offset[i];                                    //adds offset amount to the input position (degrees)
  pos = map(pos, range[i][0], range[i][1], abs_0, abs_90);  //converts input pos to ms position
  maestro.setSpeed(i, spd);                                 //sets input speed to servo i
  maestro.setAcceleration(i, spd);                          //sets input acceleration to servo i
  maestro.setTarget(i, pos);                                //drives motor to calculated position
}

// moves all servos to their calculated positions at a certain speed and acceleration value
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

// ends the program and stops all of the servos
void stop() {  
  for (int i = 0; i < 6; i++) {
    maestro.setTarget(i, 0);  //stops servo i
  }
  while (1) {}
}

// find the location of the ball using the pixy2 cam
void findBall() {  
  // grab blocks!
  pixy.ccc.getBlocks();

  // If there is 1 ball detected then collect and print the data
  if (pixy.ccc.numBlocks == 1) {
    //sets current X and Y co-ords
    ball[x] = pixy.ccc.blocks[0].m_x;  // absolute X location of the ball
    ball[y] = pixy.ccc.blocks[0].m_y;  // absolute Y location of the ball
  }
  // If there are multiple balls detected, then print so
  else if (pixy.ccc.numBlocks > 1) {
    Serial.println("MULTIPLE BALLS DETECTED");
    ball[x] = 4004;  // X component of the ball
    ball[y] = 4004;  // Y component of the ball
  }
  // If there is no ball detected, then print so
  else {
    // Serial.println("NO BALL DETECTED");
    ball[x] = 4004;  // X component of the ball
    ball[y] = 4004;  // Y component of the ball
  }
}

// calculates the Proportional, Derivative an Integral values and moves the servos
void LQR() {  

  time_f = millis(); // sets final time
  delta_time = (time_f - time_i) / 1000; // change in time (seconds)
  time_i = millis(); // sets initial time

  // error velocities with low pass filter -> state_error = [error_x, error_y, error_x_dot, error_y_dot]
  state_error(2) = state_error(2) + delta_time/T*( -state_error(2) + (ball[x] - origin[x] - state_error(x)) / delta_time ); 
  state_error(3) =  state_error(3) + delta_time/T*( -state_error(3) + (-(ball[y] - origin[y]) - state_error(y)) / delta_time ); 

  Serial.println(state_error(2));
  Serial.println(state_error(3));

  // the error is a vector pointing from the center of the platform to the center of the ball
  state_error(x) = ball[x] - origin[x];  // x component of error (origin x-axis and camera x-axis are the same)
  state_error(y) = -(ball[y] - origin[y]);  // y component of error (origin y-axis and camera y-axis are opposite)

  // checks if derivative is NaN or INF. If so, set to zero
  if (isnan(state_error(2)) || isinf(state_error(2))) {  // x component of derivative
    state_error(2) = 0;
  }
  if (isnan(state_error(3)) || isinf(state_error(3))) {  // y component of derivative
    state_error(3) = 0;
  }

  // distance from the center of the platfrom to the ball
  float r_ball = sqrt(pow(state_error(0), 2) + pow(state_error(1), 2));
  
  // if ball to far away from center, out of the platform -> move to home position
  if (r_ball > r_platform) {

    state_error(0) = 0;
    state_error(1) = 0;
    state_error(2) = 0;
    state_error(3) = 0;

    Serial.println("Detected ball out of platform");
    InverseKinematics(hx_home, hy_home, hz_home, roll_home, pitch_home, yaw_home);  //hx, hy, hz, roll, pitch, yaw
    moveServos(vel_home, acc_home);
  }
  // if ball is inside the platform -> LQR control -> set thetas to move the motors
  else {  

    Serial.println("Computing LQR");

    // compute A from delta_time
    compute_A();

    // compute B from angles and delta_time 
    compute_B();

    // compute angles[x] = roll, and angles[y] = pitch, from: A, B, C, D, Q, R and state_error
    compute_lqr_k();

    // u[i] = K[i] @ x_error (only last)
    u_star = K * state_error; // K has minus included

    // u_star = [pitch, roll_t]
    angles[x] = u_star(y) * toDeg; // angles[x] = u_star(y) = roll
    angles[y] = u_star(x) * toDeg; // angles[y] = u_star(x) = pitch

    // error prevention
    float r_out = sqrt(pow(angles[x], 2) + pow(angles[y], 2));  //calculates average of the out angle (pitch and roll)
    // if r_out is greater than r_max, then scale down out values
    if (r_out > r_max) {   
      angles[x] = angles[x] * (r_max / r_out);
      angles[y] = angles[y] * (r_max / r_out);
    }

    // compute thetas with IK
    Serial.println("Performing LQR control");
    InverseKinematics(hx_home, hy_home, hz_home, roll_home + angles[x], pitch_home + angles[y], yaw_home);  //hx, hy, hz, roll, pitch, yaw
    
    // move platform
    moveServos(vel_control, acc_control);
  }
}

// compute A from delta_time
void compute_A() {
  // A = inv(I-dt*Ac)
  Atemp = I - delta_time * Ac;

  if (fabs(Atemp.determinant()) < 1e-9) {
    Serial.println("ERROR: NOT INVERTIBLE MATRIX A");
    stop();
  }

  A = Atemp.inverse();
}

// compute B from angles and delta_time
void compute_B() {

  // pixel/s^2
  Bc(2,0) = g * r_platform / 100 * ( pow(cos(angles[y] * toRad),2) - pow(sin(angles[y] * toRad),2) );
  Bc(3,1) = g * r_platform / 100 * ( pow(sin(angles[x] * toRad),2) - pow(cos(angles[x] * toRad),2) ); 
  // Bc(2,0) = g * r_platform / 100 ; // linearized in 0
  // Bc(3,1) = -g * r_platform / 100 ; // linearized in 0

  // B = A*dt*Bc
  B = A * delta_time * Bc;
}

// compute angles[x] = roll, and angles[y] = pitch, from: A, B, C, D, Q, R and state_error
void compute_lqr_k() {

  // u(2x1) = K(2x4) * state_error(4x1)
  // state_error = [error_x, error_y, error_x_dot, error_y_dot]
  // u = [roll, pitch]

  // Initialize P[N] = Qf = Q
  P = Q;
  At = A.transpose();
  Bt = B.transpose();

  // Backward: Riccati (from i=N to 1) -> Compute P[0]
  for (int i = N; i > 0; i--) {

    // P[i-1] = At @ P[i] @ A + Q - (At @ P[i] @ B) @ inv(R + Bt @ P[i] @ B) @ (Bt @ P[i] @ A)
    temp1 = At * P * A;
    temp2 = At * P * B;
    temp3 = R + Bt * P * B;
    if (fabs(temp3.determinant()) < 1e-9) {
      Serial.println("ERROR: NOT INVERTIBLE MATRIX IN BACKWARD RICATTI");
      stop();
    }
    temp3_inv = temp3.inverse();

    // Compute (A^T @ P[i] @ B) @ inv (R + B^T @ P[i] @ B) @ (B^T @ P[i] @ A)
    temp4 = temp2 * temp3_inv * Bt * P * A;

    // Compute P[i-1] = A^T P[i] A + Q - temp4
    P = temp1 + Q - temp4;
  }

  // Forward: only need u with K[0] -> P[0]
  // K[0] = -inv(R + Bt @ P[0] @ B) @ (Bt @ P[0] @ A)
  temp5 = Bt * P * A;
  temp6 = R + Bt * P * B;
  if (fabs(temp6.determinant()) < 1e-9) {
    Serial.println("ERROR: NOT INVERTIBLE MATRIX IN FORWARD RICATTI");
    stop();
  }
  temp6_inv = temp6.inverse();
  K = -temp6_inv * temp5;
}

// compute inverse kinematics
void InverseKinematics(float hx, float hy, float hz, float roll, float pitch, float yaw) { 
  // calculates theta values given hx, hy, hz, roll, pitch, yaw
  
  //-------------------------
  // **STAGE 1 CALCULATIONS**
  // From {hx, hy, hz, roll, pitch, yaw} => get {ax, ay, az; bx, by, bz; cx, cy, cz}
  //-------------------------

  // define point h (center point of platform)
  float h[3] = { hx, hy, hz }; 

  // define rotation matrices for yaw, pitch and roll
  float R_z_yaw[3][3] = {{cos(yaw * toRad), -sin(yaw * toRad), 0}, {sin(yaw * toRad), cos(yaw * toRad), 0}, {0, 0, 1}}; // row, row, row
  float R_y_pitch[3][3] = {{cos(pitch * toRad), 0, sin(pitch * toRad)}, {0, 1, 0}, {-sin(pitch * toRad), 0, cos(pitch * toRad)}}; 
  float R_x_roll[3][3] = {{1, 0, 0}, {0, cos(roll * toRad), -sin(roll * toRad)}, {0, sin(roll * toRad), cos(roll * toRad)}};

  // define initial ha, hb, and hc
  float ha_i[3] = {0, -t, 0};
  float hb_i[3] = {sqrt(3.0f)*t/2.0f, t/2.0f, 0};
  float hc_i[3] = {-sqrt(3.0f)*t/2.0f, t/2.0f, 0};

  // initialize rotated ha, hb, hc
  float ha[3] = {0, 0, 0};
  float hb[3] = {0, 0, 0};
  float hc[3] = {0, 0, 0};
  
  // calculate rotated ha, hb, hc
  rotations(ha, ha_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(hb, hb_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(hc, hc_i, R_z_yaw, R_y_pitch, R_x_roll);

  // calculate af, bf, cf
  float af[3] = {h[0] + ha[0], h[1] + ha[1], h[2] + ha[2]};
  float bf[3] = {h[0] + hb[0], h[1] + hb[1], h[2] + hb[2]};
  float cf[3] = {h[0] + hc[0], h[1] + hc[1], h[2] + hc[2]};
  

  //-------------------------
  // STAGE 2 CALCULATIONS
  // From {ax, ay, az, bx, by, bz, cx, cy, cz}
  // => get {a1fx, a1fy, a1fz; a2fx, a2fy, a2fz; b1fx, b1fy, b1fz; b2fx, b2fy, b2fz; c1fx, c1fy, c1fz; c2fx, c2fy, c2fz}
  //-------------------------

  // define initial vectors af_a1f, af_a2f, bf_bf1, bf_b2f, cf_c1f, cf_c2f
  float af_a1f_i[3] = {-w, 0, 0};
  float af_a2f_i[3] = {w, 0, 0};
  float bf_b1f_i[3] = {w/2.0f, -w*sqrt(3.0f)/2.0f, 0};
  float bf_b2f_i[3] = {-w/2.0f, w*sqrt(3.0f)/2.0f, 0};
  float cf_c1f_i[3] = {w/2.0f, w*sqrt(3.0f)/2.0f, 0};
  float cf_c2f_i[3] = {-w/2.0f, -w*sqrt(3.0f)/2.0f, 0};
  

  // initialize rotated vectors a1f, a2f, b1f, b2f, c1f, c2f
  float af_a1f[3] = {0, 0, 0};
  float af_a2f[3] = {0, 0, 0};
  float bf_b1f[3] = {0, 0, 0};
  float bf_b2f[3] = {0, 0, 0};
  float cf_c1f[3] = {0, 0, 0};
  float cf_c2f[3] = {0, 0, 0};

  // calculate rotated vectors a1f, a2f, b1f, b2f, c1f, c2f
  rotations(af_a1f, af_a1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(af_a2f, af_a2f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(bf_b1f, bf_b1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(bf_b2f, bf_b2f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(cf_c1f, cf_c1f_i, R_z_yaw, R_y_pitch, R_x_roll);
  rotations(cf_c2f, cf_c2f_i, R_z_yaw, R_y_pitch, R_x_roll);

  // calculate a1f, a2f, b1f, b2f, c1f, c2f
  float a1f[3] = {af[0] + af_a1f[0], af[1] + af_a1f[1], af[2] + af_a1f[2]};
  float a2f[3] = {af[0] + af_a2f[0], af[1] + af_a2f[1], af[2] + af_a2f[2]};
  float b1f[3] = {bf[0] + bf_b1f[0], bf[1] + bf_b1f[1], bf[2] + bf_b1f[2]};
  float b2f[3] = {bf[0] + bf_b2f[0], bf[1] + bf_b2f[1], bf[2] + bf_b2f[2]};
  float c1f[3] = {cf[0] + cf_c1f[0], cf[1] + cf_c1f[1], cf[2] + cf_c1f[2]};
  float c2f[3] = {cf[0] + cf_c2f[0], cf[1] + cf_c2f[1], cf[2] + cf_c2f[2]};

  // calculate vector a1 (from a10 to a1f), a2 (from a20 to a2f), b1 (from c10 to c1f), b2 (from b20 to b2f), c1 (from c10 to c1f), c2 (from c20 to c2f)
  float a1[3] = { a1f[0] - a10[0], a1f[1] - a10[1], a1f[2] - a10[2] };
  float a2[3] = { a2f[0] - a20[0], a2f[1] - a20[1], a2f[2] - a20[2] };
  float b1[3] = { b1f[0] - b10[0], b1f[1] - b10[1], b1f[2] - b10[2] };
  float b2[3] = { b2f[0] - b20[0], b2f[1] - b20[1], b2f[2] - b20[2] };
  float c1[3] = { c1f[0] - c10[0], c1f[1] - c10[1], c1f[2] - c10[2] };
  float c2[3] = { c2f[0] - c20[0], c2f[1] - c20[1], c2f[2] - c20[2] };


  //-------------------------
  //**STAGE 3 CALCULATIONS**
  // From {a1fx, a1fy, a1fz; a2fx, a2fy, a2fz; b1fx, b1fy, b1fz; b2fx, b2fy, b2fz; c1fx, c1fy, c1fz; c2fx, c2fy, c2fz}
  // => get {theta_a10, theta_a20, theta_b10, theta_b20, theta_c10, theta_c20}
  //-------------------------

  // save previous thetas
  for (int i = 0; i < 6; i++) {
    theta_prev[i] = theta[i];
  }

  // theta_a1
  float a1s_mod = abs(dot(a1, na)); // magnitude of vector 'a1s'
  float a1s[3] = { na[0] * a1s_mod, na[1] * a1s_mod, na[2] * a1s_mod }; 
  float a1_proj[3] = { a1[0] + a1s[0], a1[1] + a1s[1], a1[2] + a1s[2] }; // projection of vector 'a1' onto the ac plane
  float a1_proj_mod = mag(a1_proj); // magnitude of vector 'a1' projected on the ac plane
  float p2_proj_mod = sqrt(pow(p2, 2) - pow(a1s_mod, 2)); // magnitude of link p2 projected on the ac plane
  theta[0] = acos(dot(a1_proj, a_10_20) / (2 * d1 * a1_proj_mod ) );                                                    
  theta[0] = (theta[0] - acos((pow(a1_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * a1_proj_mod * p1))) * toDeg;

  // theta_a2
  float a2s_mod = abs(dot(a2, na)); // magnitude of vector 'a2s'
  float a2s[3] = { na[0] * a2s_mod, na[1] * a2s_mod, na[2] * a2s_mod }; 
  float a2_proj[3] = { a2[0] + a2s[0], a2[1] + a2s[1], a2[2] + a2s[2] }; // projection of vector 'a2' onto the ab plane
  float a2_proj_mod = mag(a2_proj); // magnitude of vector 'a2' projected on the ab plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(a2s_mod, 2)); // magnitude of link p2 projected on the ab plane
  theta[1] = acos(-dot(a2_proj, a_10_20) / (2 * d1 * a2_proj_mod));                                                         
  theta[1] = (theta[1] - acos((pow(a2_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * a2_proj_mod * p1))) * toDeg; 

  // theta_b1
  float b1s_mod = abs(dot(b1, nb)); // magnitude of vector 'b1s'
  float b1s[3] = { nb[0] * b1s_mod, nb[1] * b1s_mod, nb[2] * b1s_mod }; 
  float b1_proj[3] = { b1[0] + b1s[0], b1[1] + b1s[1], b1[2] + b1s[2] }; // projection of vector 'b1' onto the ab plane
  float b1_proj_mod = mag(b1_proj); // magnitude of vector 'b1' projected on the ab plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(b1s_mod, 2)); // magnitude of link p2 projected on the ab plane
  theta[2] = acos(dot(b1_proj, b_10_20) / (2 * d1 * b1_proj_mod));                                                         
  theta[2] = (theta[2] - acos((pow(b1_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * b1_proj_mod * p1))) * toDeg; 
  
  // theta_b2
  float b2s_mod = abs(dot(b2, nb)); // magnitude of vector 'b2s'
  float b2s[3] = { nb[0] * b2s_mod, nb[1] * b2s_mod, nb[2] * b2s_mod }; 
  float b2_proj[3] = { b2[0] + b2s[0], b2[1] + b2s[1], b2[2] + b2s[2] }; // projection of vector 'b2' onto the cb plane
  float b2_proj_mod = mag(b2_proj); // magnitude of vector 'b2' projected on the cb plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(b2s_mod, 2)); // magnitude of link p2 projected on the cb plane
  theta[3] = acos(-dot(b2_proj, b_10_20) / (2 * d1 * b2_proj_mod));                                                         
  theta[3] = (theta[3] - acos((pow(b2_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * b2_proj_mod * p1))) * toDeg; 
  
  // theta_c1
  float c1s_mod = abs(dot(c1, nc)); // magnitude of vector 'c1s'
  float c1s[3] = { nc[0] * c1s_mod, nc[1] * c1s_mod, nc[2] * c1s_mod }; 
  float c1_proj[3] = { c1[0] + c1s[0], c1[1] + c1s[1], c1[2] + c1s[2] }; // projection of vector 'c1' onto the cb plane
  float c1_proj_mod = mag(c1_proj); // magnitude of vector 'c1' projected on the cb plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(c1s_mod, 2)); // magnitude of link p2 projected on the cb plane
  theta[4] = acos(dot(c1_proj, c_10_20) / (2 * d1 * c1_proj_mod));                                                         
  theta[4] = (theta[4] - acos((pow(c1_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * c1_proj_mod * p1))) * toDeg; 
  
  //theta_c2
  float c2s_mod = abs(dot(c2, nc)); // magnitude of vector 'c2s'
  float c2s[3] = { nc[0] * c2s_mod, nc[1] * c2s_mod, nc[2] * c2s_mod }; 
  float c2_proj[3] = { c2[0] + c2s[0], c2[1] + c2s[1], c2[2] + c2s[2] }; // projection of vector 'c2' onto the ac plane
  float c2_proj_mod = mag(c2_proj); // magnitude of vector 'c2' projected on the ac plane
  p2_proj_mod = sqrt(pow(p2, 2) - pow(c2s_mod, 2)); // magnitude of link p2 projected on the ac plane
  theta[5] = acos(-dot(c2_proj, c_10_20) / (2 * d1 * c2_proj_mod));                                                         
  theta[5] = (theta[5] - acos((pow(c2_proj_mod, 2) + pow(p1, 2) - pow(p2_proj_mod, 2)) / (2 * c2_proj_mod * p1))) * toDeg; 
  
  
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

    //checks theta variation is smaller than 80 degrees
    if (abs(theta[i]-theta_prev[i]) > 80) {
      Serial.println("ERROR: CURRENT VALUES ARE TOO BIG FOR ONE STEP");
      Serial.println("Motor index:");
      Serial.println(i);
      Serial.println("Motor angle variation value:");
      Serial.println(theta[i]-theta_prev[i]);
      stop();
    }

  }
}

// compute rotations
void rotations(float* result, float hi[3], float Rz[3][3], float Ry[3][3], float Rx[3][3]) {
  float temp[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // Rz * Ry
  float combined[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // (Rz * Ry) * Rx

  // Compute Rz * Ry
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        temp[i][j] += Rz[i][k] * Ry[k][j];
      }
    }
  }

  // Compute (Rz * Ry) * Rx
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        combined[i][j] += temp[i][k] * Rx[k][j];
      }
    }
  }

  // Compute (Rz * Ry * Rx) * hi
  for (int i = 0; i < 3; i++) {
    result[i] = 0; // Initialize result
    for (int j = 0; j < 3; j++) {
      result[i] += combined[i][j] * hi[j];
    }
  }
}

//finds the magnitude of an array of size 3
float mag(float array[]) {  
  float mag = 0;
  for (int i = 0; i < 3; i++) {
    mag = mag + pow(array[i], 2);  //adds component i of array squared
  }
  mag = sqrt(mag);
  return mag;
}

//calculates the dot product of two arrays
float dot(float array1[], float array2[]) {  
  return array1[0] * array2[0] + array1[1] * array2[1] + array1[2] * array2[2];
}