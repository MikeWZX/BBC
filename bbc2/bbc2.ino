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
#define KP 1.0
#define KI 0.001
#define KD 0.00001
#define L  0.112  // length of rod (m)
#define l  0.035  // length of servo horn (m)
#define r  0.010  // radius of the platform (m)
#define mountAngle 120.0/180.0*PI                 // angle between mounts on platform (rad)
#define maxServoAngle 50.0/180.0*PI               // max servo angle before servo horn will hit table or servo will reach max range (+/-)
#define maxServoHornHeight l*sin(maxServoAngle)   // max (+/-) distance servo horn can move in z

// servo setup
Servo servoA, servoB, servoC;

// PID variables
unsigned long currTime, prevTime;
double lastErrorX, lastErrorY;
double cumuErrorX, cumuErrorY;

// positions
double setX, setY;                  // set position (target)

void setup() {
  // open the serial port at 115200 bps
  Serial.begin(115200);

  // attach servos to pins
  servoA.attach(9);
  servoB.attach(10);
  servoC.attach(11);

  // home the servo
  home();
}

void loop() {
  // ===== READ POSITION OF BALL ===== //
  
  // wait for input
  while (!Serial.available()) {}  // blocks the execution before reading the values
  double startTime = millis();
  // take input and turn it into two doubles
  String strIn = Serial.readStringUntil('\n');
  // last know position of ball
  int comma_idx = indexOf(strIn, ',');
  double posX = strIn.substring(0,comma_idx).toDouble();
  double posY = strIn.substring(comma_idx+1, strIn.length()).toDouble();

  // ===== CONVERT BALL POSITION TO DESIRED TABLE POSE WITH PID ===== //
  
  currTime = millis();                                   // get current time (of position reading) from clock
  double elapTime = (double)(currTime - prevTime);       // measure time since last position reading

  double errorX = posX - setX;                           // set error to equal difference between current position and desired position (for P Term)
  double errorY = posY - setY;
  cumuErrorX += errorX * elapTime;                       // integrate the  cumulative error (for I term)
  cumuErrorY += errorY * elapTime;
  double rateErrorX = (errorX - lastErrorX)/elapTime;    // derivate the rate error (for D term)
  double rateErrorY = (errorY - lastErrorY)/elapTime;

  double theta = - KP*errorX;// + KI*cumuErrorX + KD*rateErrorX;      // use PID to calculate table theta angle
  double phi = KP*errorY;// + KI*cumuErrorY + KD*rateErrorY;          // use PID to calculate table phi angle
  Serial.println(String(theta)+" "+String(phi));
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
    Serial.println("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmax");
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
  Serial.println(alpha*180.0/PI+60,6);
  Serial.println(beta*180.0/PI+60,6);
  Serial.println(gamma*180.0/PI+60,6);

  // write servo angles
  servoA.write(alpha*180.0/PI+60);
  servoB.write(beta*180.0/PI+60);
  servoC.write(gamma*180.0/PI+60);
}

// ===== FUNCTIONS ===== //

// move servos to middle position
double home() {
  int middle = 60;        // center position of servos
  servoA.write(middle);
  servoB.write(middle);
  servoC.write(middle);
  delay(5000);            // pause for 5 seconds
}

// find the first index of a character within a string
int indexOf(String str, char c) {
  int str_len = str.length() + 1; // +1 to account for null char
  char chr_arr[str_len];
  str.toCharArray(chr_arr, str_len);
  int index = strchr(chr_arr,c)-chr_arr;
  return !index ? -1 : index;
}
