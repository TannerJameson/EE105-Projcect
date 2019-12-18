#include <MPU6050.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int axVals[1000];
int ayVals[1000];
int azVals[1000];
int gxVals[1000];
int gyVals[1000];
int gzVals[100];
int count;
int yaw_int;
int yaw = 0;
int time_step;


#define OUTPUT_READABLE_ACCELGYRO


void setup() {
   Serial.begin(9600);
   mpu.initialize();
   Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
   count = 0;
   output = createWriter("data.csv");
}

void loop() {
      // read raw accel/gyro measurements from device
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      yaw_int = gz;
      count = 0;
     
  while(count< 1000) {
    delay(0.1);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    yaw += (0.1*gz/131.2);
    Serial.println(yaw);
    // these methods (and a few others) are also available
    //mpu.getAcceleration(&ax, &ay, &az);
    //mpu.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
      /*
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    */
    #endif
/*
    axVals[count] = ax;
    ayVals[count] = ay;
    azVals[count] = az;
    gxVals[count] = gx;
    gyVals[count] = gy; */
    //gzVals[count] = gz;
    count = count + 1;
    //Serial.println("yaw = ");
    //Serial.println(yaw);
    
    
    }
    //Serial.println("gzVals = ");
    //Serial.println(gzVals[10]);

    
}
