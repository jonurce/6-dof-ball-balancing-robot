
// libraries
#include "PololuMaestro.h"
#include <Pixy2SPI_SS.h>

//teensy
#define maestroSerial SERIAL_PORT_HARDWARE_OPEN 

// objects
Pixy2SPI_SS pixy;
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

float X, Y;        // X and Y co-ords of the markers
float r_platform;  //max radius of the platform

float X_total = 0, Y_total = 0;  // the total summation of X and Y values
float r_platform_total = 0;      // the total summation of r_max values
int j = 0;

void setup() {
  Serial.begin(115200);
  maestroSerial.begin(9600);
  pixy.init();

  // moves servos to 0 degree position
  int pos;
  for (int i = 0; i < 6; i++) {
    maestro.setSpeed(0, 0);                                         //sets speed to servo i
    maestro.setAcceleration(0, 0);                                  //setsacceleration to servo i
    pos = map(offset[i], range[i][0], range[i][1], abs_0, abs_90);  //finds servo 0 degree position in microseconds
    maestro.setTarget(i, pos);                                      //drives motor to calculated position
  }
  delay(1000);
}

void loop() {

  // run 100 loops and gets average center location and r platform value
  for (int k = 0; k < 100; k++) { 
    pixy.ccc.getBlocks();

    // If there are 4 markers detected, then collect and print the data
    if (pixy.ccc.numBlocks == 4) {
      //set current X and Y co-ords of the center
      X = 0;
      Y = 0;
      for (int i = 0; i < 4; i++) {
        X = X + pixy.ccc.blocks[i].m_x;
        Y = Y + pixy.ccc.blocks[i].m_y; 
      }

      X = X / 4;
      Y = Y / 4;

      r_platform = 0;
      for (int i = 0; i < 4; i++) {
        // adds magnitude of vector pointing from the center of the platform to the center of the marker
        r_platform = r_platform + sqrt(pow(pixy.ccc.blocks[i].m_x - X, 2) + pow(pixy.ccc.blocks[i].m_y - Y, 2));  
        // adds magnitude of vector pointing from the center of the marker to the corner of the marker
        r_platform = r_platform + sqrt(pow(pixy.ccc.blocks[i].m_width, 2) + pow(pixy.ccc.blocks[i].m_height, 2)) / 2;
      }

      r_platform = r_platform / 4;

      X_total = X_total + X;
      Y_total = Y_total + Y;
      r_platform_total = r_platform_total + r_platform;

      j++;

    }
    // If there are not 4 markers detected, then print so
    else {
      Serial.println("Failed to detect 4 markers");
    }
  }

  X_total = X_total / j;
  Y_total = Y_total / j;
  r_platform_total = r_platform_total / j;

  //prints results: location of the center + radius of platform
  Serial.println((String) "CENTER: [" + int(X_total) + ", " + int(Y_total) + "]"); 
  Serial.println((String) "r platform: " + int(r_platform_total));                  

  while (1) {}
}
