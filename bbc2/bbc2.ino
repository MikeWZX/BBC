/* 
// ===== DESCRIPTION ===== //

Filename: bbc2.ino
Authors: 
  Billy Lin (lin14@berkeley.edu)
  Chris Wu (chriswhu@berkeley.edu)
  Karl Skeel (karl_skeel@berkeley.edu)
  Michael Wang (mzxwang@berkeley.edu)
Last edited: 12/07/2022

This script first homes the servos to their original position and pauses
Then it reads in servo positions from serial. It converts these positions
to servo angles using PID, then commands the servos to those positions.
The goal is to tilt the platform such that the ball rests on the center
of the platform

*/

// ===== SETUP ===== //

// imports
#include <Servo.h>

// macros
#define KP 0.25     // 0.12, servos should saturate around 1.0
#define KI 0.0001 // was 0.001
#define KD 0.05    // 0.6
#define L  0.142    // length of rod (m)
#define l  0.035    // length of servo horn (m)
#define r  0.1129   // radius of the platform (m)
#define mountAngle 120.0/180.0*PI                 // angle between mounts on platform (rad)
#define maxServoAngle 50.0/180.0*PI               // max servo angle before servo horn will hit table or servo will reach max range (+/-)
#define maxServoHornHeight l*sin(maxServoAngle)   // max (+/-) distance servo horn can move in z
#define maxTableAngle atan(maxServoHornHeight/r)
#define maxPlatLen 0.160
#define tableRatio maxTableAngle/maxPlatLen
#define max_cumu_error_deg 5.0 // was 0.57
#define MAX_CUMU_ERR max_cumu_error_deg*PI/180.0/KI

// servo setup
Servo servoA, servoB, servoC;

// PID variables
unsigned long currTime, prevTime;
double lastErrorX, lastErrorY;
const int arrLen = 100;
double cumuErrorArr[2][arrLen];
int i = 0;

// positions
double setX, setY;                  // set position (target) (0.0)

void setup() {
  // open the serial port at 9600 bps
  Serial.begin(9600);

  // attach servos to pins
  servoA.attach(9);
  servoB.attach(10);
  servoC.attach(11);

  // home the servo
  home();
  prevTime = millis();
}

void loop() {
  // ===== READ POSITION OF BALL ===== //
  
  // wait for input
  while (!Serial.available()) {}  // blocks the execution before reading the values
  // take input and turn it into two doubles
  String strIn = Serial.readStringUntil('\n');
  // last know position of ball
  int comma_idx = indexOf(strIn, ',');
  double posX = strIn.substring(0,comma_idx).toDouble();
  double posY = strIn.substring(comma_idx+1, strIn.length()).toDouble();

  // ===== CONVERT BALL POSITION TO DESIRED TABLE POSE WITH PID ===== //
  
  currTime = millis();                                   // get current time (of position reading) from clock
  double elapTime = (double)(currTime - prevTime);       // measure time since last position reading

  double errorX = (posX - setX)*tableRatio;                           // set error to equal difference between current position and desired position (for P Term)
  double errorY = (posY - setY)*tableRatio;
  cumuErrorArr[0][i] = errorX;
  cumuErrorArr[1][i] = errorY;
  i = (i+1)%arrLen;
  double cumuErrorX, cumuErrorY;
  for (int j = 0; j < arrLen; j++){
    cumuErrorX += cumuErrorArr[0][j];
    cumuErrorY += cumuErrorArr[1][j];
  }
  // elapTime = max(elapTime, 0.01);
  // cumuErrorX = max(-MAX_CUMU_ERR, min(cumuErrorX + errorX * elapTime, MAX_CUMU_ERR));                      // integrate the  cumulative error (for I term)
  // cumuErrorY = max(-MAX_CUMU_ERR, min(cumuErrorY + errorY * elapTime, MAX_CUMU_ERR));
  
  double rateErrorX = (errorX - lastErrorX)/elapTime;    // derivate the rate error (for D term)
  double rateErrorY = (errorY - lastErrorY)/elapTime;

  double theta = -KP*errorX - KI*cumuErrorX/arrLen - KD*rateErrorX;      // use PID to calculate table theta angle
  double phi = -KP*errorY - KI*cumuErrorY/arrLen - KD*rateErrorY;          // use PID to calculate table phi angle

  // scale down angles if needed
  double maxAng = max(abs(theta), abs(phi));
  if (maxAng > maxTableAngle) {
    double ratio = maxTableAngle / maxAng;
    theta *= ratio;
    phi *= ratio;
  }

  // update errors and previous time
  lastErrorX = errorX;
  lastErrorY = errorY;
  prevTime = currTime;

  // ===== CONVERT DESIRED TABLE POSE TO SERVO ANGLES ===== //

  // tilting from x offset (about y, angle theta)
  double zATheta = r*sin(theta);
  double zBTheta = -r*cos(mountAngle/2)*sin(theta);
  double zCTheta = zBTheta;
  // tilting from y offset (about x, angle phi)
  double zAPhi = 0;
  double zBPhi = -sin(phi)*r*sin(mountAngle/2);
  double zCPhi = -zBPhi;
  // total desired z positions
  double zA = zATheta + zAPhi;
  double zB = zBTheta + zBPhi;
  double zC = zCTheta + zCPhi;

  // scale down z height if one of them exceeds maximum allowed
  double maxZ = max(max(abs(zA), abs(zB)), abs(zC));
  if (maxZ > maxServoHornHeight) {
    double ratio = maxServoHornHeight / maxZ;
    zA *= ratio;
    zB *= ratio;
    zC *= ratio;
  }
  // calculate servo angles from desired z heights
  double alpha = asin(zA/l);
  double beta = asin(zB/l);
  double gamma = asin(zC/l);

  // ===== COMMAND SERVOS TO DESIRED ANLGES ===== //

  // print servo angles
  // Serial.println(alpha,6);
  // Serial.println(beta,6);
  // Serial.println(gamma,6);

  // write servo angles
  servoA.write(angToServoCmd(alpha));
  servoB.write(angToServoCmd(beta));
  servoC.write(angToServoCmd(gamma));
}

// ===== FUNCTIONS ===== //

// move servos to middle position
double home() {
  // center position of servos
  int pause = 500;
  int middle = 60;        
  servoA.write(middle);
  servoB.write(middle);
  servoC.write(middle);
  memset(*cumuErrorArr, 0, 2*arrLen);

  double a1 = 0.8;
  double a2 = -0.3;
  double a3 = 0.0;
  double a4 = -0.6;

  double start[3][4] = {
    {a1,-a1,a3,a3},
    {a2,-a2,a4,-a4},
    {a2,-a2,-a4,a4}
    };

  for (int q = 0; q < 4; q= q + 1) {
  delay(pause);
  servoA.write(angToServoCmd(start[0][q]));
  servoB.write(angToServoCmd(start[1][q]));
  servoC.write(angToServoCmd(start[2][q]));
  
  delay(pause);
  servoA.write(middle);
  servoB.write(middle);
  servoC.write(middle);
  }
  delay(2000);
}

// find the first index of a character within a string
int indexOf(String str, char c) {
  int str_len = str.length() + 1; // +1 to account for null char
  char chr_arr[str_len];
  str.toCharArray(chr_arr, str_len);
  int index = strchr(chr_arr,c)-chr_arr;
  return !index ? -1 : index;
}

double angToServoCmd (double angle) {
  return int(angle*180.0/PI+60.0);
}
