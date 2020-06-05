# BMI090L examples

## Examples overview

### `common.h`
- For the sake of simplicity of example code, repeated code is placed in `common.h`
- Contains definitions of the following functions
  - init_app_board()
  - init_bmi090l_sensor_driver_interface()
  - init_sensor_interface()

### read_sensor_data
- Configures sensor to read accelerometer and gyroscope data.
- Prints accel. data in `g` and gyro. data in `dps`
- Runs on PC, APP2.0 and APP3.0 microcontroller

### any_motion
- Configures sensor to generate interrupt when sensor is moved.
- Adjust threshold according to your application needs.
- Runs on APP2.0 and APP3.0 microcontroller

### data_sync
- For information on data sync. feature, read [DataSync.md](https://github.com/BoschSensortec/BMI08x-Sensor-API/blob/master/DataSync.md)
- Requires shorting of Gyro. data ready interrupt pin with Accel. sync. input interrupt pin
- Runs on APP2.0 and APP3.0 microcontroller

### high_g
- Configures sensor to generate interrupt when acceleration exceeds the set high-g threshold.
- Move the sensor upwards to create condition of high-g
- Increase hysteresis to reduce false triggers
- Runs on APP2.0 and APP3.0 microcontroller

### low_g
- Configures sensor to generate interrupt when acceleration value goes below the set low-g threshold.
- Move the sensor downwards (or) put in a state of free fall to create condition of low-g 
- Increase hysteresis to reduce false triggers
- Runs on APP2.0 and APP3.0 microcontroller

### no_motion
- Configures sensor to generate interrupt when sensor is stationary.
- Adjust threshold according to your application needs.
- Runs on APP2.0 and APP3.0 microcontroller

### orientation
- Configures sensor to generate interrupt when sensor's orientation is changed.
- Shows the below outputs depending on the orientation
  - Portrait upright
  - Portrait upside down
  - Landscape left
  - Landscape right
  - Face up
  - Face down
- Runs on APP2.0 and APP3.0 microcontroller

---
