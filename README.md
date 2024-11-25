Implementation of Madgwich algorithm on Arduino to calculate dynamic acceleration and angular velocity from the accelerometer and gyroscope readinngs.

# Arduino Sensor Integration Project

This Arduino project integrates multiple sensors to capture various data, process it, and send the information via Serial communication. 

## Included Libraries

1. **Sensor Libraries**:
   - `Arduino_LSM6DS3.h`: For the 6-axis IMU sensor (accelerometer and gyroscope).
   - `Arduino_LSM9DS1.h`: For the 9-axis IMU sensor.
   - `Arduino_LPS22HB.h`: For the pressure sensor.
   - `Arduino_HTS221.h`: For the temperature and humidity sensor.
   - `Arduino_APDS9960.h`: For the color, proximity, and gesture sensor.

2. **Serial Communication**:
   - `SerialTransfer.h`: For struct-based serial communication.
   - `MadgwickAHRS.h`: For the Madgwick filter, which is used for sensor fusion to get orientation data.

## Global Variables and Struct

- **Struct `STRUCT`**: Holds sensor data such as roll, pitch, yaw, magnetic field data, pressure, temperature, humidity, and proximity.
- **Madgwick Filter**: Used for sensor fusion to calculate roll, pitch, and yaw from accelerometer and gyroscope data.
- **Timing and Scaling Variables**: Used for sensor data processing.

## Setup Function

1. **Pin and Serial Initialization**:
   - Sets a pin as output for an LED.
   - Initializes Serial and Serial1 communication.
   - Begins the Madgwick filter.
   - Initializes sensors and prints an error message if any sensor fails to initialize.
   - Calls `calculate_IMU_error()` to calculate sensor errors.

2. **Timing Initialization**:
   - Sets the interval for sensor readings.

## Loop Function

1. **Sensor Data Reading**:
   - Reads accelerometer data and calculates roll and pitch angles.
   - Reads gyroscope data and calculates angular velocity and angles.
   - Reads magnetometer data.
   - Reads pressure, temperature, humidity, and proximity values.

2. **Complementary Filter**:
   - Combines accelerometer and gyroscope data to get the final roll, pitch, and yaw values.

3. **Data Transmission**:
   - Sends the data via Serial1 using the `sendViaSerial1()` function.

4. **Clearing Data**:
   - Resets struct values and angle calculations for the next loop iteration.

## Helper Functions

1. **writeByte(char decimal)**:
   - Converts a decimal value to binary and sends it bit by bit, controlling an LED to indicate binary values.

2. **sendViaSTransfer()**:
   - Sends the struct data using the SerialTransfer library.

3. **calculate_IMU_error()**:
   - Calculates the error values for the accelerometer and gyroscope by taking multiple readings when the IMU is stationary.

4. **sendViaSerial1()**:
   - Sends all the sensor data stored in the struct via Serial1. This allows another device to receive the data.

5. **Get_Dy_accAndg()**:
   - Reads and processes accelerometer and gyroscope data at regular intervals using the Madgwick filter to update orientation (roll, pitch, yaw).

6. **convertRawAcceleration(int aRaw) and convertRawGyro(int gRaw)**:
   - Converts raw sensor data to meaningful physical units (g for acceleration and degrees/second for gyroscope).

## Summary

This code continuously reads data from multiple sensors, processes it to calculate orientation (roll, pitch, yaw) and other environmental parameters (pressure, temperature, humidity, proximity), and transmits this data to another device using Serial1. It uses the Madgwick filter for sensor fusion and applies error corrections to the sensor readings. The setup ensures the program initializes correctly, and the loop function handles ongoing data processing and transmission.

<div align="center">
  <a href="https://maazsalman.org/">
    <img width="70" src="click-svgrepo-com.svg" alt="gh" />
  </a>
  <p> Explore More! 🚀</p>
</div>

