//#include <MPU6050.h>
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


void driveSinewave(double t, double freq)
{
 double  T = 1/freq;
 double duration = T*1000;
  Serial.println("duration = ");
  Serial.println(duration);
  int start = millis();
  int curr = millis() - start;
  Serial.print("t is");
  Serial.println(curr);
  Serial.println("in drivesine");
  double goSpeed;
  while(curr < duration) {
    goSpeed = sin(2*3.14*20*t) * 255;
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
    goSpeed = sin(2*3.14*20*t) * 255;
    go(RIGHT,-goSpeed);
    go(LEFT,goSpeed);
    curr = millis() - start2;
  }
  go(RIGHT,0);
  go(LEFT,0);

}

//**************************************************************************

}

//******************************************************************************

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
double freq[] = {0.5, 1, 1.5, 2, 3, 5, 8, 10, 50};
void loop() {
  delay(100);
  Serial.println("entered loop");
  //driveSine(20,2000);
  //sine2(50,5000);

  // Challenge
  driveSinewave(1000,freq[k]);
  k++;
  //delay(2000);


  


}
