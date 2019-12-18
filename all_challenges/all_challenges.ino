#include <MPU6050.h>
// ******************** Constants ********************************************

// for challenge 1
int mspeed;

// For challenge 3
 static int i;
 static int prev_err;
 int p;
 int d;
 int gas;
 int kp;
 int kd;
 int ki;
 int thresh; // set this to sensor value on black path

 //*****************************************************************************




// ***************** From robot example code  *******************************



// Motor control pins : L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
// I switched pins 4 and 2 to make the motor directions correspond to the test function
const int in3Pin = 2; // Right motor Direction 1
const int in4Pin = 4; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control

enum Motor {LEFT, RIGHT};
#define NUM_ANGLES 7
unsigned char sensorAngle[NUM_ANGLES] = { 60, 70, 80, 90, 100, 110, 120 };
unsigned int distance[NUM_ANGLES];


// Set motor speed: 255 full ahead, -255 full reverse , 0 stop
void go( enum Motor m, int speed)
{
  digitalWrite(m == LEFT ? in1Pin : in3Pin , speed > 0 ? HIGH : LOW);
  digitalWrite(m == LEFT ? in2Pin : in4Pin , speed <= 0 ? HIGH : LOW);
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed);
}

// Initial motor test :
// left motor forward then back
// right motor forward then back
void testMotors ()
{
  static int speed[8] = { 128, 255, 128, 0 ,
  -128, -255, -128, 0};
  go(RIGHT, 0);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(LEFT, speed[i ]), delay (200);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(RIGHT, speed[i ]), delay (200);
}

// ********************************************************************




// *********************** Challenge 1 ***********************************

// For imu
/* 
MPU6050 mpu;
mpu.initialize();
int ax, ay, az
*/

// duration is in ms
// freq is in Hz
void driveSine(int freq, int duration) {
  Serial.print("f = ");
  Serial.println(freq);
  int start = millis();
  int t = millis() - start;
  mspeed = 0;
  while(t < duration) {
    Serial.print("t = ");
    Serial.println(t);
    mspeed = sin(2*PI*freq*t)*255*20;
    if (mspeed > 255) {
      mspeed = 255;
    }
    Serial.print("moter speed = ");
    Serial.println(mspeed);
    go(LEFT,-mspeed);
    go(RIGHT,mspeed);
    t = millis() - start; 
  }
  go(LEFT,0);
  go(RIGHT,0);
  Serial.println("switch direction");
  start = millis();
  t = millis() - start;
  mspeed = 0;
  while(t < duration) {
    mspeed = sin(2*PI*freq*t)*255*20;
    if (mspeed > 255) {
      mspeed = 255;
    }
    go(LEFT,mspeed);
    go(RIGHT,-mspeed);
    t = millis() - start; 
  }
  go(LEFT,0);
  go(RIGHT,0);
  Serial.println("end of driveSine");
}

void sine2(int freq, int duration) {
  int start = millis();
  int t = millis() - start;
  while (t < duration) {
    Serial.println("in while");
    mspeed = sin(2*PI*freq)*255;
    go(LEFT,mspeed);
    go(RIGHT,-mspeed);
    Serial.println(mspeed);
    t = millis() - start;
  }  
}


void driveSinewave(double t, double freq)
{
 //double  T = 1/freq;
  //double duration = T*1000;
 double duration = 1000;
  Serial.println("duration = ");
  Serial.println(duration);
  int start = millis();
  int curr = millis() - start;
  Serial.print("t is");
  Serial.println(curr);
  Serial.println("in drivesine");
  double goSpeed;
  while(curr < duration) {
    goSpeed = sin(2*3.14*freq*t) * 255;
    go(RIGHT, goSpeed);
    go(LEFT, -goSpeed);
    Serial.println("first while");
    Serial.println(goSpeed);
    curr = millis() - start;
  }
int   start2 = millis();
curr = millis() - start2;
  while(curr < duration) {
    Serial.println("second");
    goSpeed = sin(2*3.14*freq*t) * 255;
    go(RIGHT,-goSpeed);
    go(LEFT,goSpeed);
    curr = millis() - start2;
  }
  go(RIGHT,0);
  go(LEFT,0);

}

//**************************************************************************




// ********************* Challenge 2 ************************************************

void Chall_2(int angle, int desired_angle, int kp) {

  //Determine error
  int change = angle-desired_angle;

  //Write gas to motor pin.
  gas = change*kp;

 //Make sure it's between -255 and 255.
 if (gas > 255)
    gas = 255;
 else if (gas < -255)
    gas = -255;


  //Turn left or right.
  if (gas > 0) {
      go(RIGHT, abs(gas));
      go(LEFT,0);
  }
  else {
      go(LEFT, abs(gas));
      go(RIGHT,0);
  }

}

//******************************************************************************



// ****************** Challenge 3*********
void Chall_3 (int kp,int kd, int ki, int level, int thresh) {

  p = level - thresh;
  
  d = p - prev_err;
  prev_err = p;

  i = i + p;
  
  if (p  = 0) {
     i = 0;
  }

 //100 as constant so that it will go straight when no error. (see left below)
  gas = 100 + (d*kd) + (p*kp) + (i*ki);

  if (gas > 255)
    gas = 255;

  //Because traveling counter-clockwise, we will only need to travel right
  go(LEFT, 100);
  go(RIGHT, gas);
}

// ***************************************************



void setup() {
  Serial.begin(9600);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  //servo.attach(servoPin);
  //servo.write(90);
  go(LEFT, 0);
  go(RIGHT, 0);
  testMotors();

  // for challenge 3
  gas = 0;
  i = 0;
  prev_err = 0;
  mspeed = 0;



}

int k = 0;
double freq[] = {0.5, 5, 10, 15, 20, 30, 50};
void loop() {
  delay(2000);
  Serial.println("entered loop");
  //driveSine(20,2000);
  //sine2(50,5000);

  // Challenge
  driveSinewave(1000,freq[k]);
  k++;
  //delay(2000);


  


}
