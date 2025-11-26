# GravOff-robot
## Introduction
This repository presents the code for "parabolic trajectory regulation" in matlab format and low-level control code implemented in teensy 4.0. 

## Prerequisites
### Parabolic trajectory regulation
  1. Ensure you have Matlab installed (the code was tested under 2019b).
  2. Motion capture system will be needed (this project uses luster motion capture system). If you use the same mocap system, change the ip address in the .m file. Be aware of the orientation setting in the mocap system, different settings would causes reversed control reactions and divergence from the reference trajectory.

### low-level control code 
  1. The low-level control of GravOff robot is implemented in Teensy 4.0. Ensure the right hardware is used, teensy 3.5 or 4.1 should also work but pin connections for actuators and sensors need to be redifined. ESP32-S3 support most of the libraries used in this project and should also work, certain libraries may need to be replaced with ESP32 compatible ones. Specifically, servo.h need to be replaced with EPS32Servo.h, and replace TeensyPWM library with library like ESP32_FastPWM (https://github.com/khoih-prog/ESP32_FastPWM).
  2. For teensy 4.0, ensure you have include these libraries for hardware connection, driving, multitasking, and noise filtering, etc. 
      Teensy_PWM.h: https://github.com/khoih-prog/Teensy_PWM
      servo: https://github.com/arduino-libraries/Servo
      TaskScheduler: https://github.com/arkhipenko/TaskScheduler
      sbus: https://github.com/bolderflight/sbus
      Wire: https://github.com/codebendercc/arduino-library-files/tree/master/libraries/Wire
      AS5600: https://github.com/RobTillaart/AS5600
      <wit_c_sdk.h> and <REG.h> library for the JY901S IMU: https://wit-motion.yuque.com/wumwnr/ltst03/rqyk6g?#%20  (The use of IMU is not restricted, alternatives such as          mpu6050 could work). 
      Ranging sensor VL53L5CX: http://librarymanager/All#SparkFun_VL53L5CX
      Simple Kalman filter: https://github.com/denyssene/SimpleKalmanFilter.
