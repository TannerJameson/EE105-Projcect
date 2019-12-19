#include <MPU6050.h>
#include <Servo.h>

// ******************** Constants ********************************************

// For challenge 3
 static double i;
 static double prev_err;
 double p;
 double d;
 double gas;
 double kp;
 double kd;
 double ki;
 double pitch_int;
 double time_step;
 double desired_pitch;
 double pitch;
 int16_t ax, ay, az;
 int16_t gy, gx, gz; 
 MPU6050 mpu; 
 
 //*****************************************************************************




// ***************** From robot example code  *******************************

Servo servo;

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

// ********************************************************************









void Chall_4 (double pitch)
{

  //PID
  p = pitch - desired_pitch;
  
  d = p - prev_err;
  prev_err = p;

 if(p==0){
    i = 0;} 
 else{
  i+=p;}

 //100 as constant so that it will go straight when no error. (see left below)
  gas = (d*kd) + (p*kp) + (i*ki);

//make sure this is within the motor's ability
  if (gas > 255) {
    gas = 255;}
  else if (gas < -255){
    gas = -255;}

//Write gas to the motors
    go(RIGHT, gas);
    go(LEFT, gas);
    
  }
    

double get_pitch()
{
  delay(100);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitch += time_step*(gy-pitch_int)/131;
  Serial.println(pitch);
  return pitch;  
}

// ***************************************************



void setup() {
  Serial.begin(9600);
  /*
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite (trigPin, LOW);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  servo.attach(servoPin);
  servo.write(90);
  */
  go(LEFT, 0);
  go(RIGHT, 0);
  mpu.initialize();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitch_int = gy; 
  time_step = 0.1;
  pitch = 0; 
  prev_err = 0;
  desired_pitch = -1;
  pitch = 0;
  mpu.initialize();
  //PID
  kp = 5;
  kd = .1;
  ki = .1;

}


void loop() {
  delay(2000);
  Serial.println("entered loop");
  //driveSine(20,15000);
  //delay(2000);

    //Determine error
  pitch = get_pitch();
  Chall_4(pitch);

}
