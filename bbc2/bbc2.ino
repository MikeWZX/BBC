// imports
#include <Servo.h>

// servo setup
Servo servoA, servoB, servoC;

// PID variables
double Kp = 10.5, Kd = 0.00001, Ki = 0.001;          // PID constants
unsigned long currTime, prevTime;
double elapTime;
double errorX, lastErrorX, cumuErrorX, rateErrorX;
double errorY, lastErrorY, cumuErrorY, rateErrorY;


// Platform dimensions
double L = 170.0/1000.0;                // length of rod (m)
double l = 50.0/1000.0;                 // length of servo horn (m)
double r = 70.0/1000.0;                 // radius of platform (m) 
double mountAngle = 120.0/180.0*PI;   // angle between mounts on platform (rad)

// positions
double posX, posY;                  // ball position
double setX, setY;                  // set position (target)

void setup() {
  // open the serial port at 9600 bps
  Serial.begin(115200);
//  Serial.println("running");

  // attach servos to pins
  servoA.attach(9);
  servoB.attach(10);
  servoC.attach(11);

  // home the servo
//  Serial.println("running");
  home();
}

void loop() {
  ////////// Bring in position of ball //////////
  
  // wait for input
  while (!Serial.available()) {}
  double startTime = millis();
  // take input and turn it into two doubles
  String strIn = Serial.readStringUntil('\n');
//  Serial.println(millis()-startTime);
  // last know position of ball
  int comma_idx = indexOf(strIn, ',');
  double posX = strIn.substring(0,comma_idx).toDouble();
  double posY = strIn.substring(comma_idx+1, strIn.length()).toDouble();
  Serial.print(posX);
  Serial.println(posY);

  ////////// use PID to convert ball position to desired table angles //////////
  
  currTime = millis();
  elapTime = (double)(currTime - prevTime);

  errorX = posX - setX;
  errorY = posY - setY;
  cumuErrorX += errorX * elapTime;
  cumuErrorY += errorY * elapTime;
  rateErrorX = (errorX - lastErrorX)/elapTime;
  rateErrorY = (errorY - lastErrorY)/elapTime;

  double theta = - Kp*errorX;// + Ki*cumuErrorX + Kd*rateErrorX;
  double phi = Kp*errorY;// + Ki*cumuErrorY + Kd*rateErrorY;

  // Serial.println(errorX);
  // Serial.println(cumuErrorX);
  // Serial.println(rateErrorX);
  // Serial.println(Ki*cumuErrorX + Kd*rateErrorX);
  // Serial.println(Ki*cumuErrorY + Kd*rateErrorY);

  lastErrorX = errorX;
  lastErrorY = errorY;
  prevTime = currTime;

  ////////// calculate servo angles from desired table angles //////////

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

  // calculate servo angles from desired z heights
  double alpha = asin(zA/l);
  double beta = asin(zB/l);
  double gamma = asin(zC/l);

  ////////// command servos to desired angles //////////
  Serial.println("servo angles");
  Serial.println(alpha*180.0/PI+60,6);
  Serial.println(beta*180.0/PI+60,6);
  Serial.println(gamma*180.0/PI+60,6);
  Serial.println();

  servoA.write(alpha*180.0/PI+60);
  servoB.write(beta*180.0/PI+60);
  servoC.write(gamma*180.0/PI+60);

//  Serial.println(millis()-startTime);
}

// move servos to middle position
double home() {
//  Serial.println("homing");
  int middle = 60;
  servoA.write(middle);
  servoB.write(middle);
  servoC.write(middle);
}

// find the first indext of a character within a string
int indexOf(String str, char c) {
  int str_len = str.length() + 1; // +1 to account for null char
  char chr_arr[str_len];
  str.toCharArray(chr_arr, str_len);
  int index = strchr(chr_arr,c)-chr_arr;
  if (!index) {
    return -1;
  } else {
    return index;
  }
}
