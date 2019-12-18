/*
 * Firmware for the "2WD Ultrasonic Motor Robot Car Kit"
 *
 * Stephen A. Edwards
 *
 * Hardware configuration:
 *  A pair of DC motors driven by an L298N H bridge motor driver
 *  An HC-SR04 ultrasonic range sensor mounted atop a small hobby servo
 */

#include <Servo.h>

// Ultrasonic Module pins
const int trigPin = 13;  // 10 microsecond high pulse causes chirp, wait 50 us
const int echoPin = 12;  // Width of high pulse indicates distance

// Servo motor that aims ultrasonic sensor.
const int servoPin = 11; // PWM output for hobby servo

// Motor control pins: L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 4; // Right motor Direction 1
const int in4Pin = 2; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control

Servo servo;

enum Motor { LEFT, RIGHT };

// Set motor speed: 255 full ahead, -255 full reverse, 0 stop
void go( enum Motor m, int speed)
{
  digitalWrite(m == LEFT ? in1Pin : in3Pin, speed > 0 ? HIGH : LOW );
  digitalWrite(m == LEFT ? in2Pin : in4Pin, speed <= 0 ? HIGH : LOW );
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed );
}

// Read distance from the ultrasonic sensor, return mm
//
// Speed of sound in dry air, 20C is 343 m/s
// pulseIn returns time in microseconds (10^-6)
// 2d = p * 10^-6 s * 343 m/s = p * 0.00343 m = p * 0.343 mm/us
unsigned int readDistance()
{
  digitalWrite( trigPin, HIGH );
  delayMicroseconds(10);
  digitalWrite( trigPin, LOW );
  unsigned long period = pulseIn( echoPin, HIGH );
  return period * 343 / 2000;
}

#define NUM_ANGLES 7

unsigned char sensorAngle[NUM_ANGLES] = { 60, 70, 80, 90, 100, 110, 120 };
unsigned int distance[NUM_ANGLES];

// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed.  This takes a reading, then
// sends the servo to the next angle.  Call repeatedly once every 100 ms or so.
void readNextDistance()
{ 
  static unsigned char angleIndex = 0;
  static signed char step = 1;

  distance[angleIndex] = readDistance();
  
  angleIndex += step;
  if (angleIndex == NUM_ANGLES - 1) step = -1;
  else if (angleIndex == 0) step = 1;

  servo.write( sensorAngle[angleIndex] );
}

// Initial motor test:
//  left motor forward then back 
//  right motor forward then back
void testMotors()
{
  static int speed[8] = { 128, 255, 128, 0 ,
			  -128, -255, -128, 0};
  go(RIGHT, 0);

  for (unsigned char i = 0 ; i < 8 ; i++)
    go(LEFT, speed[i]), delay(200);

  for (unsigned char i = 0 ; i < 8 ; i++)
    go(RIGHT, speed[i]), delay(200);
}

// Initial configuration:
//   Configure the input and output pins
//   Center the servo
//   Turn off the motors
//   Test the motors
//   Scan the surroundings
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);

  servo.attach(servoPin);
  servo.write(90);
  
  go(LEFT, 0);
  go(RIGHT, 0);

  // Initial motor test: verify both left and right motors are working
  // and connected in the right direction
  testMotors();

  // Scan the surroundings before starting off
  servo.write( sensorAngle[0] );
  delay(200);
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++)
    readNextDistance(), delay(200);
}

// Main loop:
//  Get the next sensor reading
//  If anything appears to be too close, back up
//  Otherwise, go forward

void loop() {
  readNextDistance();

  // See if something is too close at any angle
  unsigned char clear = 1;
  for (unsigned char i = 0 ; i < NUM_ANGLES ; i++)
    if (distance[i] < 300)
      clear = 0;

  if (clear) {
    // Nothing in our way: go forward
    go(LEFT, 255);
    go(RIGHT, 255);
  } else {
    // Something's nearby: back up left
    go(LEFT, -180);
    go(RIGHT, -80);
  }

  // Check the next direction in 50 ms
  delay(50);
}
