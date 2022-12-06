#include <math.h>
#include <Servo.h>
#include <stdio.h>
#define BAUDRATE 115200
#define CLOCK_FREQUENCY 16000000
//Motor Pins
#define MOTORPIN1
#define MOTORPIN2
#define MOTORPIN3 

//Control Variables
#define KPx 0.001
#define KPy 0.001
#define KDx 0.001
#define KDy 0.001
#define KIx 0.001
#define KIy 0.001

//System Variables
#define S 170
#define A 50
#define R_base 70
#define R_plat 70
#define beta1 0
#define beta2 2*M_PI/3
#define beta3 4*M_PI/3
float T = sqrt(S*S - A*A);
float beta[3] = {beta1, beta2, beta3};
float Kp[2] = {KPx, KPy};
float Kd[2] = {KDx, KDy};
float Ki[2] = {KIx, KIy};
//Calculation holder variables
float Pp[3] = {0, 0, 0};
float Error[3] = {0, 0, 0};
float Error_dot[3] = {0, 0, 0};
float Error_sum[3] = {0, 0, 0};
float target_angles[2] = {0, 0}; //phi theta
float target_q[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

//Motor control variables
float alpha[3] = {0, 0, 0};

//ISR variables
volatile float x_coor = 0;
volatile float y_coor = 0;
const byte numChars = 32;
volatile char tempChars[numChars];    // temporary array for use when parsing
volatile char receivedChars[numChars]; 
volatile boolean newData = false;
int controlLoopRate = 95; //frequency in Hz

//============= ISR Functions
// Timer Setup: frequency needs to be larger than 1 HZ
void setupTimer(int freq){
  // set up Timer
  noInterrupts(); // stop interrupts
  
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  
  // freq: 1~3
  if(freq <= 3){ 
    // set compare match register for freq(<=3) Hz increments
    OCR1A = CLOCK_FREQUENCY/(256*freq)-1; 
    // Set CS12, CS11 and CS10 bits for 256 prescaler
    TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
  }
  // freq: 4~30
  else if(freq <= 30){ 
    // set compare match register for freq(<=30) Hz increments
    OCR1A = CLOCK_FREQUENCY/(64*freq)-1; 
    // Set CS12, CS11 and CS10 bits for 64 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  }

  // freq: 31~244
  else if(freq <= 244){ 
    // set compare match register for freq(<=30) Hz increments
    OCR1A = CLOCK_FREQUENCY/(8*freq)-1; 
    // Set CS12, CS11 and CS10 bits for 8 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  }

  // freq: >=245
  else{ 
    // set compare match register for freq(<=30) Hz increments
    OCR1A = CLOCK_FREQUENCY/(1*freq)-1; 
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  }

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts(); // allow interrupts
}

ISR(TIMER1_COMPA_vect) {
  //grab the current x and y position and push previous onto 
  //read check pyserial, update to new xy coordinate, push old to new array, compute PID
}
  
//============= PID Calculation function
void PID() {
  
  target_angles[0] = -Kp[0]*Error[1]-Kd[0]*Error_dot[1]-Ki[0]*Error_sum[1];  //y distance error requires x axis rotation - "-" to counteract ball
  target_angles[1] = Kp[1]*Error[0]+Kd[1]*Error_dot[0]+Ki[1]*Error_sum[0];  //x distance error requires y axis rotation
  
  //Set saturation angles for theta (y axis rotation) and phi (x axis rotation)
  target_angles[1] = min(target_angles[1], atan2(2*A, R_plat*(cos(M_PI/3)+1))); //min of upper bound
  target_angles[1] = max(target_angles[1], atan2(-2*A, R_plat*(cos(M_PI/3)+1))); //max of lower bound

  target_angles[0] = min(target_angles[0], atan2(2*A,2*R_plat*sin(M_PI/3))); 
  target_angles[0] = max(target_angles[0], atan2(-2*A,2*R_plat*sin(M_PI/3)));
}
//============= Transform calculation functions

void Transformation() {

  float pRod[3][3] = {{R_plat * cos(beta[0]), R_plat * cos(beta[1]), R_plat * cos(beta[2])},
    {R_plat * sin(beta[0]), R_plat * sin(beta[1]), R_plat * sin(beta[2])},
    {0, 0, 0}
  };

  //Transforms point p on platform by (target_angles and T)
  target_q[0][0] = pRod[0][0] * cos(target_angles[1]);
  target_q[0][1] = pRod[0][1] * cos(target_angles[1]);
  target_q[0][2] = pRod[0][2] * cos(target_angles[1]);
  target_q[1][0] = pRod[1][0] * cos(target_angles[0]) + pRod[0][0] * sin(target_angles[0]) * cos(target_angles[1]);
  target_q[1][1] = pRod[1][1] * cos(target_angles[0]) + pRod[0][1] * sin(target_angles[0]) * cos(target_angles[1]);
  target_q[1][2] = pRod[1][2] * cos(target_angles[0]) + pRod[0][2] * sin(target_angles[0]) * cos(target_angles[1]);
  target_q[2][0] = T + pRod[1][0] * sin(target_angles[0]) - pRod[0][0] * cos(target_angles[0]) * sin(target_angles[1]);
  target_q[2][1] = T + pRod[1][1] * sin(target_angles[0]) - pRod[0][1] * cos(target_angles[0]) * sin(target_angles[1]);
  target_q[2][2] = T + pRod[1][2] * sin(target_angles[0]) - pRod[0][2] * cos(target_angles[0]) * sin(target_angles[1]);
}
//============= Motor angle calculation functions

void find_next_q() {
  Pp[0] = x_coor;
  Pp[1] = y_coor;
  Error[0] = -Pp[0];
  Error[1] = -Pp[1];
  PID(); //Write PID function
  Transformation();
}

void find_next_alpha() {
  // coordinates of rods in base frame
  float B[3][3] = {{R_base * cos(beta[0]), R_base * cos(beta[1]), R_base * cos(beta[2])},
    {R_base * sin(beta[0]), R_base * sin(beta[1]), R_base * sin(beta[2])},
    {0, 0, 0}};
    
  float l[3][3];
  
  for (int i = 0; i <= 2; i= i+1) { //calculate l
    for (int j = 0; j <= 2; j= j+1 ) {
      l[i][j] = target_q[i][j] - B[i][j];
    }
  }

  
  for (int i = 0; i <= 2; i= i+1) { //for loop to calculate alpha 1-3
    float l_norm = sqrt(l[0][i] * l[0][i] + l[1][i] * l[1][i] + l[2][i] * l[2][i]);
    float Li = l_norm * l_norm * - (S * S - A * A);
    float xi = l[0][i];
    float yi = l[1][i];
    float zi = l[2][i];
    float Mi = 2 * A * zi;
    float Ni = 2 * A * cos(beta[i]) * xi + sin(beta[i]) * yi;
    alpha[i] = asin(Li / sqrt(Mi * Mi + Ni * Ni) - atan2(Ni, Mi));
  }
}

//============= Read and Parse data
//Reading serial data
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    if (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
//Parsing data
void readParseData() {      // split the data into its parts NEEDS TO UPDATE

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  x_coor = atof(strtokIndx); // convert msg part to float x_coor

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  y_coor = atof(strtokIndx);     // convert this part to an float y_coor
}

//============= setup and loop
void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  setupTimer(controlLoopRate);
  // pin mode for motors


  // set up interrupt for Pin A, B
  //attachInterrupt(0, doEncoderA, CHANGE); 
  //attachInterrupt(1, doEncoderB, CHANGE); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
find_next_q();
find_next_alpha();
}

//===============
