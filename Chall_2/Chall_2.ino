#include <MPU6050.h>
//#include <Servo.h>

// ******************** Constants ********************************************

// For challenge 3
 static int i;
 static int prev_err;
 static double yaw;
 int error;
 double angle;
 int level;
 int p;
 int d;
 int gas;
 int kp;
 int kd;
 int ki;
 double desired_angle;
 int yaw_int;
 int time_step;
 int16_t ax, ay, az;
 int16_t gy, gx, gz; 
 int thresh; // set this to sensor value on black path
 MPU6050 mpu; 
// *****************************************************************************


// ***************** From robot example code  *******************************

//Servo servo;

// Motor control pins : L298N H bridge
int enAPin = 6; // Left motor PWM speed control
int in1Pin = 7; // Left motor Direction 1
int in2Pin = 5; // Left motor Direction 2
// I switched pins 4 and 2 to make the motor directions correspond to the test function
int in3Pin = 2; // Right motor Direction 1
int in4Pin = 4; // Right motor Direction 2
int enBPin = 3; // Right motor PWM speed control
int photo  = A1; // Photo detector pin level

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


//**************************************************************************




// ********************* Challenge 2 ************************************************

void Chall_2(int error) {


  //Write gas to motor pin.
  gas = error*kp;

 //Make sure it's between -255 and 255.
 if (gas > 255){
    gas = 255;}
 else if (gas < -255){
    gas = -255;}

  //Turn left or right.
  if (gas < 0){
      go(RIGHT, abs(gas));
      go(LEFT,-abs(gas));}
  else if (gas > 0){
      go(LEFT, abs(gas));
      go(RIGHT,-abs(gas));}
  else{
      go(LEFT, 0);
      go(RIGHT, 0);}
}

double get_yaw()
{
  delay(100);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gz, &gz);
  yaw += (0.1*((gz-yaw_int)/1310));
  return yaw;  
}

//******************************************************************************

// ***************************************************



void setup() {
  Serial.begin(9600);
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//  digitalWrite (trigPin, LOW);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  pinMode(photo, INPUT);
//  servo.attach(servoPin);
//  servo.write(90);
//  go(LEFT, 0);
//  go(RIGHT, 0);
//  testMotors();
//  mpu.intialize();
//  mpu.getMotion6(&ax, &ay, &az, &gx, &gz, &gz);
  yaw_int = gz; 
  time_step = 0.1;
  yaw = 0; 
  kp = 250;
  desired_angle = 0; 
  

}


void loop() {

//    // Challenge 2
//    //Determine error
  angle = get_yaw();
  Serial.println(angle); 
  error = angle-desired_angle;
  Chall_2(error);

}
