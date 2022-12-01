#include <math.h>
#include <Servo.h>
#include <stdio.h>
#define BAUDRATE 115200
//Motor Pins
#define motorpin_1 
#define motorpin_2
#define motorpin_3

//Control Variables
#define KPx 0.001
#define KPy 0.001
#define KDx 0.001
#define KDy 0.001
#define KIx 0.001
#define KIy 0.001

//System Variables


//Calculation holder variables
float Pp[2];
//


//ISR variables
volatile float x = 0;
volatile float y = 0;
const byte numChars = 32;
volatile char tempChars[numChars];

volatile float phi = 0;
volatile float theta = 0;

//============= setup and loop
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


//============= ISR Functions

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    x = atof(strtokIndx); // convert msg part to float x_coor
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    y = atof(strtokIndx);     // convert this part to an float y_coor


}


//============= Motor angle calculation functions
void map_image_platform() {
  Pp = {x_coor, y_coor, 0};
  }

 void find_next_q() {
  
  }
