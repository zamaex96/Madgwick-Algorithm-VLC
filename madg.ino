
//#include <Arduino_LSM6DS3.h>
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
#include <Arduino_LPS22HB.h> //Include library to read Pressure
#include <Arduino_HTS221.h> //Include library to read Temperature and Humidity
#include <Arduino_APDS9960.h> //Include library for colour, proximity and gesture recognition
#include "SerialTransfer.h"
#include <MadgwickAHRS.h>

SerialTransfer myTransfer;

struct STRUCT {
  //char z;
  float roll, pitch, yaw, mag_x_axis, mag_y_axis, mag_z_axis, pres, temp, humid, prox;
} testStruct;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

#define DELAY 4
//#define DELAY 1
#define LED 4 //Output pin where LED is connected
#define ID 180 //ID of Receiver
#define BITLENGTH 8 //1 Byte
#define FIRSTBIT pow(2,(BITLENGTH-1))//MSB Value
#define TRUE 1
#define delaySensor 1000
int i, j;
int count=0;
//long Timer = millis();
//float voltage;
void writeByte(char);
//int counter=0;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
//float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float mag_x, mag_y, mag_z;
float Pressure;
float Temperature, Humidity;
int Proximity;
int c = 0;

float ax, ay, az;
  float gx, gy, gz;
  float roll_F, pitch_F, heading_F;
  unsigned long microsNow;
  

void setup() {
 //Setting the LED as Output
 pinMode(LED,OUTPUT);
 Serial.begin(115200);
 Serial1.begin(9600);
  myTransfer.begin(Serial1);
  filter.begin(25);
 //while (!Serial);
 // Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);}

    if (!BARO.begin()) //Initialize Pressure sensor
  { Serial.println("Failed to initialize Pressure Sensor!"); while (1);}
  
  if (!HTS.begin()) //Initialize Temperature and Humidity sensor
  { Serial.println("Failed to initialize Temperature and Humidity Sensor!"); while (1);}
  
  if (!APDS.begin()) //Initialize Colour, Proximity and Gesture sensor
  { Serial.println("Failed to initialize Colour, Proximity and Gesture Sensor!"); while (1);}
//delay(20);
  // Call this function if you need to get the IMU error values for your module, keep the mcu still when the error function is called
  calculate_IMU_error();
  delay(20);
   // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}



void loop()
{

  
// === Read acceleromter data === //
    if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    }
     // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);//+AccErrorX  ;//- 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);//+AccErrorY;// + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
   // Define variables to store previous gyroscope readings
    float prevGyroX = 0.0;
    float prevGyroY = 0.0;
    float prevGyroZ = 0.0;
    // Conversion factor from degrees to radians
    float degToRad = PI / 180.0;
    // Calculate change in orientation (angular velocity) for each axis in radians per second
    float deltaGyroX_rad = (GyroX - prevGyroX) * degToRad;
    float deltaGyroY_rad = (GyroY - prevGyroY) * degToRad;
    float deltaGyroZ_rad = (GyroZ - prevGyroZ) * degToRad;
   // Update previous gyroscope readings for the next iteration
    prevGyroX = GyroX;
    prevGyroY = GyroY;
    prevGyroZ = GyroZ;

    // Calculate overall angular velocity (magnitude of the vector)
    float angularVelocity = sqrt(pow(deltaGyroX_rad, 2) + pow(deltaGyroY_rad, 2) + pow(deltaGyroZ_rad, 2));

    // Print or use the overall angular velocity
    Serial.print("Overall Angular Velocity: ");
    Serial.println(angularVelocity);
  
  }
  // Correct the outputs with the calculated error values
 // GyroX = GyroX+GyroErrorX;//+ 0.56; // GyroErrorX ~(-0.56)
 // GyroY = GyroY+GyroErrorY ;//- 2; // GyroErrorY ~(2)
//  GyroZ = GyroZ+GyroErrorZ ;//+ 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  testStruct.yaw  =   testStruct.yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  testStruct.roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
   testStruct.pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
//writeByte(ID); //Sending ID of Receiver
//writeByte(roll);//sending the temperature to the receiver
//writeByte(pitch);//sending the temperature to the receiver
//writeByte(yaw);//sending the temperature to the receiver
//Serial.print(testStruct.roll);
 //  Serial.print("/");
 //  Serial.print(testStruct.pitch);
 //  Serial.print("/");
 //  Serial.println( testStruct.yaw);

     //Magnetometer values 

  if (IMU.magneticFieldAvailable()) {

    IMU.readMagneticField(mag_x, mag_y, mag_z);
   testStruct.mag_x_axis=mag_x;
    testStruct.mag_y_axis=mag_y;
    testStruct.mag_z_axis=mag_z;

  }

  //Read Pressure value

  Pressure = BARO.readPressure();
testStruct.pres=Pressure;

  //Read Temperature value

  Temperature = HTS.readTemperature();
 testStruct.temp=Temperature;

  //Read Humidity value

  Humidity = HTS.readHumidity();
testStruct.humid=Humidity;

  //Proximity value

  if (APDS.proximityAvailable()) {
Proximity = APDS.readProximity();

   
    testStruct.prox=Proximity;}

   //  sendViaSTransfer();
  

  sendViaSerial1();
 // Serial1.print(1);
// PrintAllSensorData();
 Get_Dy_accAndg();
testStruct.roll=0;testStruct.pitch=0; testStruct.yaw=0;testStruct.mag_x_axis=0;testStruct.mag_y_axis=0;testStruct.mag_z_axis=0; testStruct.pres=0; testStruct.temp=0;testStruct.humid=0; //testStruct.prox=0;
accAngleX=0;accAngleY=0;gyroAngleX=0;gyroAngleY=0;
}


//Sending 1byte number
void writeByte(char decimal)
{
//converting the decimal value to binary and sending 8 bit information from MSB to LSB
 int i,binary;
 for(i=0;i<BITLENGTH;i++)
 {
  //int a= decimal % 2;
 binary = (int)decimal/FIRSTBIT; //Getting the first binary bit value
 decimal= (decimal & ((int)FIRSTBIT -1));//Setting the first bit to zero
 decimal=decimal<<1; //Shift all bits by one to left
 if(binary==TRUE)
 //if (a==1)
 {
 digitalWrite(LED,HIGH);
  //analogWrite(A0,255);  
 //Serial.print("1");
 }
 else
 {
 digitalWrite(LED,LOW);
 // analogWrite(A0,0);  
 //Serial.print("0");
 }
 //delay(DELAY);
 delayMicroseconds(1000);
 }
//Serial.println();
//digitalWrite(LED,LOW);
}


void sendViaSTransfer()
{
  // use this variable to keep track of how many
  // bytes we're stuffing in the transmit buffer
  uint16_t sendSize = 0;

  ///////////////////////////////////////// Stuff buffer with struct
  sendSize = myTransfer.txObj(testStruct, sendSize);

  ///////////////////////////////////////// Stuff buffer with array
 // sendSize = myTransfer.txObj(arr, sendSize);

  ///////////////////////////////////////// Send buffer
  myTransfer.sendData(sendSize);
  //delay(500);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
     if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);

  }
   
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
     if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(GyroX, GyroY, GyroZ);
 
  }
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX );
    GyroErrorY = GyroErrorY + (GyroY );
    GyroErrorZ = GyroErrorZ + (GyroZ );
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
}


void sendViaSerial1()
{
  // Serial1.print(1); 
  // Serial.print(1); 
  //  Serial.print(" ; ");
 //  Serial1.print(" ; ");
   Serial1.print(testStruct.roll);
   Serial.print(testStruct.roll);
   Serial.print(" ; ");
   Serial1.print(" ; ");
  //delay(delaySensor);
  
   Serial1.print(testStruct.pitch);
   Serial.print(testStruct.pitch);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.print(testStruct.yaw);
   Serial.print(testStruct.yaw);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.print(testStruct.mag_x_axis);
   Serial.print(testStruct.mag_x_axis);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.print(testStruct.mag_y_axis);
   Serial.print(testStruct.mag_y_axis);
    Serial.print(" ; ");
   Serial1.print(" ; ");
//  delay(delaySensor);
   Serial1.print(testStruct.mag_z_axis);
   Serial.print(testStruct.mag_z_axis);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.print(testStruct.pres);
   Serial.print(testStruct.pres);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.print(testStruct.temp);
   Serial.print(testStruct.temp);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.print(testStruct.humid);
   Serial.print(testStruct.humid);
    Serial.print(" ; ");
   Serial1.print(" ; ");
 // delay(delaySensor);
   Serial1.println(testStruct.prox);
   Serial.println(testStruct.prox);
   // Serial.print(" ; ");
  // Serial1.print(" ; ");
  // Serial1.println(0);
   //Serial.println(0);
 // delay(delaySensor); 
  
}

//void PrintAllSensorData()
//{
//  Serial.print("Accelerometer = "); Serial.print("roll: ");Serial.print(testStruct.roll); Serial.print(", ")Serial.print("pitch: ");Serial.print(testStruct.pitch);Serial.print(", ");Serial.print("yaw: ");Serial.println( testStruct.yaw );
 //    Serial.print("Magnetometer = "); Serial.print("MagX: ");Serial.print(testStruct.mag_x_axis); Serial.print(", "); Serial.print("MagY: ");Serial.print(testStruct.mag_y_axis);Serial.print(", ");; Serial.print("MagZ: ")Serial.println(testStruct.mag_z_axis);
 //    Serial.print("Pressure = ");Serial.println(testStruct.pres);
 //     Serial.print("Temperature = ");Serial.println(testStruct.temp);
 //      Serial.print("Humidity = ");Serial.println(testStruct.humid);
 //       Serial.print("Proximity = ");Serial.println(testStruct.prox);
// }

void Get_Dy_accAndg() {
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

   if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    }
    if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(GyroX, GyroY, GyroZ);}
    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(AccX);
    ay = convertRawAcceleration(AccY);
    az = convertRawAcceleration(AccZ);
    gx = convertRawGyro(GyroX);
    gy = convertRawGyro(GyroY);
    gz = convertRawGyro(GyroZ);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll_F = filter.getRoll();
    pitch_F = filter.getPitch();
    heading_F = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading_F);
    Serial.print(" ");
    Serial.print(pitch_F);
    Serial.print(" ");
    Serial.println(roll_F);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  }

  float convertRawAcceleration(int aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
