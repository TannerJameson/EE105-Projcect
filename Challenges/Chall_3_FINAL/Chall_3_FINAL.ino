//#include <MPU6050.h>
//#include <Servo.h>

// ******************** Constants ********************************************

// For challenge 3
 static double i;
 static int prev_err;
 int level;
 int p;
 double d;
 double gas;
 int kp;
 double kd;
 double ki;
 int norm;
 int time_step;
// MPU6050 mpu; 
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

// ****************** Challenge 3*********
void Chall_3(int level) {

  p = level - thresh;  
  
  d = p - prev_err;
  prev_err = p;
  
  if (p == 0){
    i = 0;} 
    else{
      i+=p;  
    }
 // Serial.println(p);
 //norm as constant so that it will go straight when no error. (see left below)
  gas = norm + (p*kp) + (i*ki) + (kd*d);
 // Serial.println(gas);
  
  if (gas > 255){
    gas = 255;}
  
    //This was the most stable version, only using PID on one of the wheels
  go(LEFT, norm - (p*kp));
  go(RIGHT, gas);

}

int get_light()
{
  delay(10);
  level = analogRead(photo);
  return level;
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
  pinMode(photo, INPUT);
  go(LEFT, 0);
  go(RIGHT, 0);
  

 //  for challenge 3
  gas = 0;
  i = 0;
  prev_err = 0;
  kp = 25;
  kd = .3;
  ki = .6;
  thresh = 15;
  norm = 107;
  

}


void loop() {
  
    //Challenge 3
level = get_light();
Chall_3(level);
    

}
