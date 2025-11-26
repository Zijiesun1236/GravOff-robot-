#include <Teensy_PWM.h>
#include <Servo.h>  
#include <TaskScheduler.h>
#include "sbus.h"
#include <Wire.h>
#include "AS5600.h"
#include <REG.h>
#include <wit_c_sdk.h> 
#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX
#include <SimpleKalmanFilter.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.005);
SimpleKalmanFilter simpleKalmanFilter1(2, 2, 0.005);
#define START_INDI "<"
#define END_INDI ">"

#define HWSERIAL Serial7
#define HWSERIAL1 Serial2
#define pinToUse 6
#define pinToUse1 10
// #define pi 3.1415926535
#define motorPin1 6
#define motorPin2 9
#define motorPin3 11
#define motorPin4 12
// #define selectPin 7
int analogPin = A8;
int analogPin1 = A9;
int voltagePin = A13;
int currentPin = A12;


//-----------define macros for JY901S gyro--------------------------//
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t* p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };



//creates teensy pwm instance
Teensy_PWM* PWM_Instance;
Teensy_PWM* PWM_Instance1;
Teensy_PWM* PWM_motor1;
Teensy_PWM* PWM_motor2;
Teensy_PWM* PWM_motor3;
Teensy_PWM* PWM_motor4;

double frequency = 10000.0d;  // multishot from 5 to 25us, for HKARC ESC that controls the leg motors.
double dutyCYCLE = 15.0d;     // middle point of multishot 

//as5600 object declaration
AS5600 as5600(&Wire);
AS5600 as5600_1(&Wire1);



//sbus initialization
/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial5);  // Serialx: here x corresponds to the serial number used, here 5.
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial5);
/* SBUS data */
bfs::SbusData data;
// bfs::SbusData data_send;

void led();
void sbus_read();
void as5600Read();
void as5600Read1();
void p_control();
void p1_control();
void mode_selection();
void gyro();
void tof();
void brakee();
void print_data();
void rateTest();
void hwserialread();
void mpccommand();


//void position_assign();
Task LED(50, TASK_FOREVER, &led);
Task sbusRead(10, TASK_FOREVER, &sbus_read);
Task AS5600READ(0.2, TASK_FOREVER, &as5600Read);  //间隔时间（ms），执行次数，所有函数。
Task AS5600READ1(0.2, TASK_FOREVER, &as5600Read1);
Task P_CONTROL(0.2, TASK_FOREVER, &p_control);
Task P1_CONTROL(0.2, TASK_FOREVER, &p1_control);
Task MODE_SELECTION(1, TASK_FOREVER, &mode_selection);
Task GYRO(0.5, TASK_FOREVER, &gyro);
Task TOF(5, TASK_FOREVER, &tof);
Task BRAKE(10, TASK_FOREVER, &brakee);
Task PRINT_DATA(2, TASK_FOREVER, &print_data);
Task RATETEST(1, TASK_FOREVER, &rateTest);
//locomotion tasks

Task HWSERIALREAD(1, TASK_FOREVER, &hwserialread);
Task MPCCOMMAND(5, TASK_FOREVER, &mpccommand);



Scheduler runner;

float CH1;
float CH2;
float CH3;
float CH4;
float CH5;
float CH6;
float CH7;
float CH8;
float CH9;
float CH10;


float thr_test, count_for_thr = 0;
// read channel values 
void sbus_read() {
  if (sbus_rx.Read()) {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
    float ch1 = 100.0 * (data.ch[0] - 307) / (1693 - 307);
    float ch2 = 100.0 * (data.ch[1] - 1693) / (307 - 1693);
    float ch3 = 100.0 * (data.ch[2] - 307) / (1693 - 307);
    float ch4 = 100.0 * (data.ch[3] - 307) / (1693 - 307);
    float ch5 = map(data.ch[4], 307, 1693, 0, 100);
    float ch6 = 100.0 * (data.ch[5] - 307) / (1693 - 307);
    float ch7 = map(data.ch[6], 307, 1693, 0, 100);
    float ch8 = 100.0 * (data.ch[7] - 307) / (1693 - 307);
    float ch9 = map(data.ch[8], 307, 1693, 0, 100);
    float ch10 = map(data.ch[9], 307, 1693, 0, 100);
    //  Serial.println(String(ch1) + "\t" + ch2 + "\t" + ch3 + "\t" + ch4 + "\t" + ch5 + "\t" + ch6 + "\t" + ch7 + "\t" + ch8 + "\t" + ch9 + "\t" + ch10);
    CH1 = ch1;
    CH2 = ch2;
    CH3 = ch3;
    CH4 = ch4;
    CH5 = ch5;
    CH6 = ch6;
    CH7 = ch7;
    CH8 = ch8;
    CH9 = ch9;
    CH10 = ch10;
  }
}


// void as5600Read(){
//   // prev_p=curr_p;
//   // prev_time=curr_time;
//   // curr_time=micros()/1000000.0;
//   // prev_p_filt=curr_p_filt;
//   // curr_p=as5600.rawAngle()*AS5600_RAW_TO_DEGREES+crr;
//   // curr_p_filt=(1-Bp)*prev_p + Bp*curr_p;
//   // omega=(curr_p_filt-prev_p_filt)/(curr_time-prev_time);
//   // Serial.print("curr_p:");
//   // Serial.print(curr_p_filt);
//   // Serial.print("\t");
//   // Serial.print("angular velocity:");
//   // Serial.println(omega);
//   double y=as5600_1.rawAngle()*AS5600_RAW_TO_DEGREES;

// //  Serial.println(y);
// }
// void as5600Read1(){
//   double y1=as5600_1.rawAngle()*AS5600_RAW_TO_DEGREES;

// }
//VL53L5CX ranging sensor

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

int imageResolution = 0;  //Used to pretty print output
int imageWidth = 0;       //Used to pretty print output
float dist5, dist6, dist9, dist10, dist_avg;
// tof sensor
void tof() {
  //Poll sensor for new data
  if (myImager.isDataReady() == true) {
    if (myImager.getRangingData(&measurementData))  //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality

      // for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      // {
      //   for (int x = imageWidth - 1 ; x >= 0 ; x--)
      //   {
      //     Serial.print("\t");
      //     Serial.print(measurementData.distance_mm[x + y]);
      //   }
      //   Serial.println();
      // }
      // Serial.println();
      //——————————————point height mode with 4*4 and 60 HZ freq——————————————
      dist5 = measurementData.distance_mm[5];
      dist6 = measurementData.distance_mm[6];
      dist9 = measurementData.distance_mm[9];
      dist10 = measurementData.distance_mm[10];
      dist_avg = (dist5 + dist6 + dist9 + dist10) / 4.0;
      //————————————————————————————————————————————————————————————————————
      // Serial.print(dist5);
      // Serial.print("\t");
      // Serial.print(dist6);
      // Serial.println();
      // Serial.print(dist9);
      // Serial.print("\t");
      // Serial.print(dist10);
      // Serial.print("\t");
      // Serial.print("average height: ");
      // Serial.println(dist_avg);
      // Serial.print("\t");
      // Serial.println();
    }
  }
}

// LED on leg
int led_indi = 1;

void led() {

  if (led_indi == 1) {
    digitalWrite(13, HIGH);
  } else if (led_indi == 0) {
    digitalWrite(13, LOW);
  }
}

//-----------------JY 901S gyro main loop----------//
float fAcc[3], fGyro[3], fAngle[3];
int i;
int i_calib = 0;
float gyro_r, gyro_p, gyro_y, angle_r, angle_p, angle_y, acc_x, acc_y, acc_z, angle_roll0, angle_pitch0, roll_err, pitch_err;  // rotation along x corresponds to roll, x is the forward-back direction
void gyro() {

  while (Serial3.available())  // serial1
  {
    WitSerialDataIn(Serial3.read());
  }
  while (Serial.available()) {
    CopeCmdData(Serial.read());
  }
  CmdProcess();
  // fAcc[0]=acceleration x, fAcc[1]=acceleration y, fAcc[2]=acceleration z,
  // fGyro[0]=gyro x, fGyro[1]=gyro y, fGyro[2]=gyro z,
  // fAngle[0]=Angle x, fAngle[1]=Angle y, fAngle[2]=Angle z,
  if (s_cDataUpdate) {
    for (i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
    }
    if (s_cDataUpdate & ACC_UPDATE) {
      // Serial.print("acc:");
      // Serial.print(fAcc[0], 3);
      // Serial.print(" ");
      // Serial.print(fAcc[1], 3);
      // Serial.print(" ");
      // Serial.print(fAcc[2], 3);
      // Serial.print("\t");
      s_cDataUpdate &= ~ACC_UPDATE;
    }
    if (s_cDataUpdate & GYRO_UPDATE) {
      // Serial.print("gyro:");
      // Serial.print(fGyro[0], 1);
      // Serial.print(" ");
      // Serial.print(fGyro[1], 1);
      // Serial.print(" ");
      // Serial.print(fGyro[2], 1);
      // Serial.print("\t");
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if (s_cDataUpdate & ANGLE_UPDATE) {
      // Serial.print("angle:");
      // Serial.print(fAngle[0], 3);
      // Serial.print(" ");
      // Serial.print(fAngle[1], 3);
      // Serial.print(" ");
      // Serial.print(fAngle[2], 3);
      // Serial.print("\t");
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    if (s_cDataUpdate & MAG_UPDATE) {
      // Serial.print("mag:");
      // Serial.print(sReg[HX]);
      // Serial.print(" ");
      // Serial.print(sReg[HY]);
      // Serial.print(" ");
      // Serial.println(sReg[HZ]);
      // Serial.print("\t");
      s_cDataUpdate &= ~MAG_UPDATE;
    }
    s_cDataUpdate = 0;
  }

  // calibrate horizontal plane for the JY901S sensor
  // i_calib += 1;
  // if (i_calib<12000){
  // angle_roll0 += fAngle[0];
  // angle_pitch0 += fAngle[1];
  // }
  // else if (i_calib=12000){
  // roll_err= angle_roll0/12000.0;
  // pitch_err= angle_pitch0/12000.0;
  // }
  // Serial.print("roll_err:");
  // Serial.print(roll_err);
  // Serial.print("\t");
  // Serial.print("pitch_err");
  // Serial.println(pitch_err);
}


double e_integral = 0;
double delta_e_dt = 0;
double u, r, r_assign, crr = 2.5, r1, r_assign1, crr1 = -1.5;  // crr: correction constant used for calibrating the robot; crr+, overall reading +
double last_y, now_y, now_y1, w;
int mode = 0;  // 0: position mode; 1: jump mode without speed feedback; 2: jumping mode with speed feedback; 3: leg contraction with speed feedback.
double t = 0;
double R;
double max_thr0 = 20;  // max_thr, ranges from 10-100, set only for position mode, i.e., mode = 0;
double max_thr1 = 20;
double thr_jump = 50;
float omega_tar = 800, omega_tar1 = 800; 
float omega_damp, omega_damp1;
int safety_indi = 0;  //0 off, 1 on

float Kp_omega = 1.2 / 1000, Kd_omega = 0 / 1000;  // 0.04,0.0008
float max_val = 1.2, min_val = 0.15;               // clamp-p speed controller
float curr_p, prev_p, curr_p1, prev_p1, currTime, prevTime, currTime1, prevTime1, curr_p_filt, prev_p_filt, curr_omega, curr_omega1, prev_omega, prev_omega1, omega_filt, omega_filt1, omega_kalfit, omega_kalfit1;
float curr_OMEGA, prev_OMEGA, curr_OMEGA1, prev_OMEGA1;
float pd, pd1, PD, PD1, i_omega, d_omega, d_omega1;
float Bp = 0.5, Bp1 = 0.5, Bw = 0.5, Bw1 = 0.5;  // low pass filter parameter
float k_4s = 1;


float Time;
float preDampDist = 20, dampDist = 4;
//as5600
void as5600Read() {
  // prev_p=curr_p;
  // prevTime=currTime;
  currTime = micros() / 1000000.0;
  curr_omega = as5600.getAngularSpeed();
  omega_kalfit = simpleKalmanFilter.updateEstimate(curr_omega);
}
void as5600Read1() {
  // prev_p=curr_p;
  // prevTime1=currTime1;
  Time = micros() / 1000000.0;
  curr_omega1 = as5600_1.getAngularSpeed();
  omega_kalfit1 = simpleKalmanFilter1.updateEstimate(curr_omega1);
  // HWSERIAL.print("position:");
  // HWSERIAL.print(as5600_1.rawAngle()*AS5600_RAW_TO_DEGREES+crr);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("angular velocity1:");
  // HWSERIAL.print("\t");
  // HWSERIAL.println(omega_kalfit1);
  // HWSERIAL.print("Time:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(currTime1,3);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("position:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(as5600.rawAngle()*AS5600_RAW_TO_DEGREES+crr);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("target_w:");
  // HWSERIAL.print(omega_tar);
  // HWSERIAL.print("angular velocity:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(omega_kalfit);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("angular velocity1:");
  // HWSERIAL.print("\t");
  // HWSERIAL.println(omega_kalfit1);
}

// print various data wirelessly to laptop, e.g., voltage and current.
double voltage, curnt;
int detach_indi = 0;
void print_data() {

  //  HWSERIAL.print(currTime,3);
  //  HWSERIAL.print("\t");
  //  HWSERIAL.print(now_y+crr);
  //  HWSERIAL.print("\t");
  //  HWSERIAL.print(now_y1+crr1);
  //  HWSERIAL.print("\t");
  
  voltage = analogRead(voltagePin) / 1024.0 * 3.3 * ((2.38 + 9.89) / 2.38) * 1.02508;  // 1.02508 is the calibration term, after comparing the measured data via multimeter 
  curnt = ((analogRead(currentPin) - 512.0) / 1024.0 * 3.3) * 1.0 / 0.04; // relationship between analog value to exact current, according to the document of ACS724LLCTR-50AB-T current sensor; 0 A correponds to Vcc/2, and sensor pin voltage increases with 40 mV/A. 

  //  if (CH7>98){
  HWSERIAL.print(currTime, 3);
  HWSERIAL.print("\t");
  HWSERIAL.print(voltage);
  HWSERIAL.print("\t");
  HWSERIAL.print(curnt);
  HWSERIAL.print("\t");
  HWSERIAL.print(omega_kalfit);
  HWSERIAL.print("\t");
  HWSERIAL.print(omega_kalfit1);
  HWSERIAL.print("\t");
  HWSERIAL.println(dist_avg);
  // }
  //0.0264
  // HWSERIAL.print("Time:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print("test");
  // HWSERIAL.print(Time,3);
  // Serial.println("test");
  // HWSERIAL.print("\t");
  // HWSERIAL.println(fAcc[2],3);
  // HWSERIAL.print(voltage);HWSERIAL.print("\t");
  // HWSERIAL.print(curnt);HWSERIAL.print("\t");
  // HWSERIAL.println(dist_avg);


  // HWSERIAL.print("Time:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(Time,3);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("position:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(as5600.rawAngle()*AS5600_RAW_TO_DEGREES+crr);
  // HWSERIAL.print("\t");
  // HWSERIAL.print(as5600_1.rawAngle()*AS5600_RAW_TO_DEGREES+crr1);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("angular velocity:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(omega_kalfit);
  // HWSERIAL.print("\t");
  // HWSERIAL.print("angular velocity1:");
  // HWSERIAL.print("\t");
  // HWSERIAL.println(omega_kalfit1);
  // HWSERIAL.print("\t");
  // HWSERIAL.println(map(CH8,0,100.0,0,0.8*76.0));
  // HWSERIAL.print("\t");
  // HWSERIAL.print("contact_forces:");
  // HWSERIAL.print("\t");
  // HWSERIAL.print(F_contact);
  // HWSERIAL.print("\t");
  // HWSERIAL.println(F_contact1);
  // HWSERIAL.print("height:");
  // HWSERIAL.print("\t");
  // HWSERIAL.println(dist_avg);
  // matlab_assign=HWSERIAL.read();
  // Serial.println(matlab_assign);
}

float previous, current;
void rateTest() { // test loop rate
  previous = current;
  current = micros() / 1000000.0;
  float cycle = current - previous;
  Serial.println(cycle, 4);
}
void p_control() {
  // double thr_jump=map(CH6,0,100,20,100);
  r = r_assign - crr;
  double Kp = 0.3, Ki = 0, Kd = 0.15;  //20/120
  
  if (r_assign == 0) { // test using command line, input leg motor angle from command line and uncomment code below to print position of the two leg motors.
    // Serial.print("waiting for input r");
    // Serial.print("\t");
    // Serial.print("y0=");
    // Serial.print(as5600.rawAngle()*AS5600_RAW_TO_DEGREES+crr); //note : +crr
    // Serial.print("\t");
    // Serial.print("y1=");
    // Serial.println(as5600_1.rawAngle()*AS5600_RAW_TO_DEGREES+crr1); // note : +crr1

    PWM_Instance->setPWM(pinToUse, frequency, 15.0d);
    PWM_Instance1->setPWM(pinToUse1, frequency, 15.0d);

  } else {
    if (mode == 0) { // position control mode 
      now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
      now_y1 = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;

      // Serial.print("test");
      // Serial.print("\t");
      // Serial.print(now_y+crr);
      // Serial.print("\t");
      // Serial.println(now_y1+crr1);
      // Serial.print("\t r_assign=");
      // Serial.print(r_assign);
      // Serial.print("\t r_assign1=");
      // Serial.println(r_assign1);

      //      Serial.print("\t");
      //      Serial.print("v_as5600:");
      //      Serial.print(as5600.getAngularSpeed());
      //      Serial.print("\t");
      //      Serial.print("v_as5600_1:");
      //      Serial.println(as5600_1.getAngularSpeed());

      double e = now_y - r;
      // satety protection
      // if(now_y+crr>255 || now_y+crr<100){
      //     safety_indi=1;
      // }
      // if (safety_indi==1){
      //   e=0;
      // }

      double sat = 120;   //saturation point
      double tole = 2.7;  // error toleration

      t += 0.0002;
      if (e >= tole && e < sat) {
        double motorValue = (((-12.5) - (-max_thr0)) / (tole - sat)) * (e - tole) + (-12.5);  // -14-（-22） 对应角度减少方向
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
        //   Serial.print(dutyCycle);
      } else if (e <= -tole && e > -sat) {
        double motorValue = ((max_thr0 - 8.5) / (-sat + tole)) * (e + tole) + 8.5;  //20-11  对应角度增加方向
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
        //   Serial.print(dutyCycle);
      } else if (e >= sat) {
        double motorValue = -max_thr0;  //-22
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      } else if (e <= -sat) {
        double motorValue = max_thr0;  //18
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      } else {
        double motorValue = 0;
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      }

    } else if (mode == 1) { // leg extension without speed feedback

      now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
      double e = now_y - r;
      double tole = 2.7;
      // Serial.print(now_y);
      // Serial.print("\t");
      // Serial.println(now_y1);
      if (e >= tole) {
        double motorValue;

        if (e <= preDampDist) {
          motorValue = -((now_y - r) / preDampDist * (9.0 / 11.0) * (0.0495 * omega_tar + 3.96));
        } else {
          motorValue = -(9.0 / 11.0) * (0.0495 * omega_tar + 3.96);
        }
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      } else {
        double motorValue = 0;
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      }

    } else if (mode == 2) { // leg extension with speed feedback 

      now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
      double e = now_y - r;
      double tole = 2.7;

      // prev parameters update
      prev_OMEGA = curr_OMEGA;
      prevTime = currTime;
      //curr parameters update
      curr_OMEGA = omega_kalfit;
      currTime = micros() / 1000000.0;
      float omega_err = omega_tar - fabs(curr_OMEGA);
      float deltaTime = currTime - prevTime;
      // Serial.println(deltaTime,4);
      d_omega = -(curr_OMEGA - prev_OMEGA) / deltaTime;
      // i_omega += omega_err*deltaTime;

      if (e >= tole) {
        double motorValue;
        if (e <= preDampDist) {
          motorValue = -k_4s * ((now_y - r) / preDampDist * (9.0 / 11.0) * (0.0495 * omega_tar + 3.96)); 
        } else {
          if (omega_err >= 0) {
            pd = max_val / 1000.0 * omega_err + min_val;
          } else if (omega_err < 0) {
            pd = max_val / 1000.0 * omega_err - min_val;
          }
          // pd=Kp_omega*omega_err - Kd_omega*d_omega;

          // sum_PD = sum_PD + delta_PD;
          PD = map(pd, -1.0, 1.0, -0.3, 0.3);

          motorValue = -k_4s * (1 + PD) * (9.0 / 11.0) * (0.0495 * omega_tar + 3.96);  //空载情况下转速-command对应经验公式（拟合后）:-(0.0495*omega_tar+3.96) -(Kp_omega*omega_err+ Kd_omega*d_omega) motorValue here is negative  -(0.0495*omega_tar+3.96)
          motorValue = constrain(motorValue, -100, -12.5);
        }
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      } else {
        double motorValue = 0;
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      }
    } else if (mode == 3) { // leg contraction with speed feedback
      // damping for impulse
      now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
      double e = now_y - r;
      double tole = 2.7;
      if (e < -tole) {

        double motorValue;
        if (e >= -dampDist && e < -0.3 * dampDist) {
          motorValue = -k_4s * ((now_y - r) / dampDist * (9.0 / 11.0) * (0.0495 * omega_damp + 3.96));
        } else if (e >= -0.3 * dampDist) {
          motorValue = k_4s * (0.3 * (9.0 / 11.0) * (0.0495 * omega_damp + 3.96));
        } else if (e < -dampDist) {
          motorValue = k_4s * (9.0 / 11.0) * (0.0495 * omega_damp + 3.96);
        }
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);

      } else {
        double motorValue = 0;
        double dutyCycle = map(motorValue, -100, 100, 5, 25);
        PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);
      }
    }
  }
}

// leg motor position and speed controller 1, for another leg 
void p1_control() {

  r1 = r_assign1 - crr1;
  double Kp = 0.3, Ki = 0, Kd = 0.15;  //20/120
  // double thr_jump=map(CH6,0,100,20,100);
  if (r_assign1 == 0) {
  } else {
    if (mode == 0) {
      now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
      now_y1 = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
      double adj_0 = now_y + crr - (now_y1 + crr1);
      double e1 = now_y1 - r1; // error

      double sat = 120;   //saturation point
      double tole = 2.7;  // error toleration
      t += 0.0002;
      if (e1 >= tole && e1 < sat) {
        double motorValue1 = (((-12.5) - (-max_thr1)) / (tole - sat)) * (e1 - tole) + (-12.5);  //-12.5-(-20)对应角度减少方向
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
        //   Serial.print(dutyCycle);
      } else if (e1 <= -tole && e1 > -sat) {
        double motorValue1 = ((max_thr1 - 8.5) / (-sat + tole)) * (e1 + tole) + 8.5;  //20-8.5 对应角度增加方向
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
        //   Serial.print(dutyCycle);
      } else if (e1 >= sat) {
        double motorValue1 = -max_thr1;  //-20
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      } else if (e1 <= -sat) {
        double motorValue1 = max_thr1;  //18
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      } else {
        double motorValue1 = 0;
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      }


    } else if (mode == 1) {  //mode=1; leg extension without speed feedback
      now_y1 = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
      now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
      double e1 = now_y1 - r1;
      double tole = 2.7;
      double adj;

      if (e1 >= tole) {
        adj = now_y + crr - (now_y1 + crr1);
        //       double motorValue=90+0.6*adj;
        double motorValue1;
        if (e1 <= preDampDist) {
          motorValue1 = -((now_y1 - r1) / preDampDist * (9.0 / 11.0) * (0.0495 * omega_tar + 3.96));
        } else {
          motorValue1 = -(9.0 / 11.0) * (0.0495 * omega_tar + 3.96);
        }
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      } else {
        double motorValue1 = 0;
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      }
    } else if (mode == 2) { // leg extension with speed feedback

      now_y1 = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
      double e1 = now_y1 - r1;
      double tole = 2.7;

      // prev parameters update
      prev_OMEGA1 = curr_OMEGA1;
      prevTime1 = currTime1;
      //curr parameters update
      curr_OMEGA1 = omega_kalfit1;
      currTime1 = micros() / 1000000.0;
      float omega_err1 = omega_tar1 - fabs(curr_OMEGA1);
      float deltaTime1 = currTime1 - prevTime1;
      d_omega1 = -(curr_OMEGA1 - prev_OMEGA1) / deltaTime1;
      // i_omega += omega_err*deltaTime;

      if (e1 >= tole) {
        double motorValue1;
        if (e1 <= preDampDist) {
          motorValue1 = -k_4s * ((now_y1 - r1) / preDampDist * (9.0 / 11.0) * (0.0495 * omega_tar1 + 3.96));
        } else {
          // motorValue=-(thr_jump);
          // if (omega_err<=20){
          //   Kd_omega=0.0005;
          // }else{
          //   Kd_omega=0.00025;
          // }

          // delta_PD1=-(Kp_omega*omega_err1 - Kd_omega*d_omega1);
          // sum_PD1 = sum_PD1 + delta_PD1;
          if (omega_err1 >= 0) {
            pd1 = max_val / 1000.0 * omega_err1 + min_val;
          } else if (omega_err1 < 0) {
            pd1 = max_val / 1000.0 * omega_err1 - min_val;
          }
          // pd1=Kp_omega*omega_err1 - Kd_omega*d_omega1;
          // sum_PD = sum_PD + delta_PD;
          PD1 = map(pd1, -1.0, 1.0, -0.3, 0.3);

          motorValue1 = -k_4s * (1.0 + PD1) * (9.0 / 11.0) * (0.0495 * omega_tar1 + 3.96);  //-(0.0495*omega_tar+3.96) -(Kp_omega*omega_err+ Kd_omega*d_omega) motorValue here is negative  -(0.0495*omega_tar+3.96)
          motorValue1 = constrain(motorValue1, -100, -12.5);
        }
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      } else {
        double motorValue1 = 0;
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      }

    } else if (mode == 3) { // leg contraction with speed feedback
      // damping for impulse
      now_y1 = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;
      double e1 = now_y1 - r1;
      double tole = 2.7;
      if (e1 < -tole) {

        double motorValue1;
        if (e1 >= -dampDist && e1 < -0.3 * dampDist) {
          motorValue1 = -k_4s * ((now_y1 - r1) / dampDist * (9.0 / 11.0) * (0.0495 * omega_damp1 + 3.96));
        } else if (e1 >= -0.3 * dampDist) {
          motorValue1 = k_4s * (0.3 * (9.0 / 11.0) * (0.0495 * omega_damp1 + 3.96));
        } else if (e1 < -dampDist) {
          motorValue1 = k_4s * (9.0 / 11.0) * (0.0495 * omega_damp1 + 3.96);
        }
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);

      } else {
        double motorValue1 = 0;
        double dutyCycle1 = map(motorValue1, -100, 100, 5, 25);
        PWM_Instance1->setPWM(pinToUse1, frequency, dutyCycle1);
      }
    }
  }
}


// servo assignment
Servo L1S1;  // leg 1 servo 1 (GDW1906 9-gram servo)
Servo L1S2;  // leg 1 servo 2 (for hip rotation)
Servo L1S3; // this is the thrust vectoring servo  
Servo L2S1;
Servo L2S2;
Servo L2S3; // this is the thrust vectoring servo
Servo brake;
Servo brake1;


void brakee() {
 //leg wheel brake, please define the radio controller channels of your own. 
  if (CH9 <= 2 && CH4 >= 70 && CH4 <= 95) { 
    brake.write(map(CH4, 0, 100.0, 80.0, 100.0)); // clamped below +-10 from the disable position (90)
    brake1.write(map(CH4, 0, 100.0, 80.0, 100.0)); 
  } else if (CH9 <= 2 && CH4 <= 30 && CH4 >= 5) {
    brake.write(map(CH4, 0, 100.0, 80.0, 100.0));
    brake1.write(map(CH4, 0, 100.0, 80.0, 100.0));
  } else {
    brake.write(90);
    brake1.write(90);
  }
}

double U1t, phit, thetat, err_psit, damp_st_indi, epsit, jump_indi, w_tar; 
float current_T, prev_T, delta_T;
String inputString;
void hwserialread() {

  while (HWSERIAL.available()) {
    char c = HWSERIAL.read();


    if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
      inputString += c;
    } else {
      double inputValue = atof(inputString.c_str());
      inputString = "";


      switch (c) {
        case '!':
          U1t = inputValue;
          break;
        case '$':
          phit = inputValue;
          break;
        case '~':
          thetat = inputValue;
          break;
        case '#':
          err_psit = inputValue;
          break;
        case '@':
          damp_st_indi = inputValue;
          break;
        case '^':
          epsit = inputValue;
          break;
        case '`':
          w_tar = inputValue;
        break;

      }
    }
   
  }

  // Serial.println(delta_T);
  // Print the values (optional)
  // Serial.print(dt);;
  // Serial.print(" ");
  // Serial.print(U1t);
  // Serial.print(" ");
  // Serial.print(phit);
  // Serial.print(" ");
  // Serial.print(thetat);
  // Serial.print(" ");
  // Serial.print(err_psit);
  // Serial.print(" ");
  // Serial.print(damp_st_indi);
  // Serial.print(" ");
  // Serial.print(epsit);
  // Serial.print(" ");
  // Serial.println(w_tar);

}


bool firsttime = true;
//intial angle adjustment, tuning them and enable the robot reach the calibration position (Fig S27A)
double i11 = 88;
double i12 = 90;  // i12-35
double i13 = 90;  //  87
double i21 = 90;
double i22 = 90;  //80 i22+50
double i23 = 90;

double l12 = i12 + 45, l22 = i22 - 45;
double l12_return = i12 + 50, l22_return = i22 - 50;
float t_curr, t_prev, DT;
int count_delay = 0, count_adj = 0;
int contact_indi = 0, thr_indi = 0, mpc_indi = 0, damping_region_indi = 0, extension_indi = 0, first_jump_finish = 0, shrink_indi = 0, lean_forward_indi = 0, lean_backward_indi = 0, rate_ctrl_indi = 0, thr_ctrl_indi = 0;  // T_damp= max_thr_indi * loop time
float jump_st_angle = 245.0, jump_end_angle = 120.0, jump_launch_angle, dist_curr, dist_prev;
float heading_count = 0, FB_count = 0, w_count = 0, rpos_count = 0, thrust_count = 0, T_heading = 36.0, left_indi = 0, right_indi = 0, forward_indi = 0, backward_indi = 0, wheeling_indi = 0, rpos_indi = 0, thrust_damp_count = 0, damp_count=0;;
float pos_detect, tilt_trigger = 5, tilt_count = 0, tilt_delay = 200, target_jump_angle = 0, target_jump_angle_value = 0, servo_tilt = 0, servo_tilt_value = 0, target_thrust = 50, M = 20.0;
// 11 tilt for 15-2.0 tracking test
// 11.6 19.1 26.6 34.15 41.7 49.2

//  float des_hypo=0.2, Kp_hypo;
int pre_shrink_count = 0;
int extension_count = 0;
int jump_initiated = 0;
int second_jump_count = 0, second_jump_indi = 0, second_shrink_count = 0, second_shrink_indi = 0, flip_count = 0, v_enhance_count = 0;
float h_gate = 1350, hc = 0.30, psi_gyro, Kp_psi_gyro = 693.0 / 40.0, err_psi_gyro, psi_des, psi_des_1st_mark = 0;
int signal_count = 0;

// 20.0   22.0
// Ft=-0.16262-0.04792*xcmd
void mode_selection() {
  now_y = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
  now_y1 = as5600_1.rawAngle() * AS5600_RAW_TO_DEGREES;

  //--------detach indicator-----------
  if (dist_avg >= 70 && CH10 == 100) {
    detach_indi = 1;
  }
  // ————————————————————mannually set pretilt angle (for field operation)————————————————————————————————————//
  // // target_jump_angle_value=map(CH6,0,10.0,5.0,25.0);  servo_tilt_value=map(CH6,0,10.0,5.0,25.0)+1;
  //————————————————————————————————————————————————————————————————//


  // ————————————————————Yaw controlled to 0; For outdoor experiment where Mocap is not used————————————————————————————————————//
  // Gyro yaw P control
  // if (fAngle[2] >= 0) {
  //   psi_gyro = fAngle[2];
  // } else if (fAngle[2] < 0) {
  //   psi_gyro = fAngle[2] + 360;
  // }
  // if (jump_initiated == 0) {
  //   err_psi_gyro = 0;
  // } else if (jump_initiated == 1) {
  //   err_psi_gyro = psi_des - psi_gyro;
  // }
  // data.ch[3] = -Kp_psi_gyro * err_psi_gyro + 1000;

  //  Serial.println(psi_gyro);
 


 

  if (U1t == 0 || mpc_indi == 0)) {
    if (CH3 <= tilt_trigger) {

      if (extension_indi == 0) {
        data.ch[1] = 1000; 
        data.ch[2] = map(CH3, 0, 100, 307, 1693);
      }

      tilt_count = 0;
    } else if (CH3 > tilt_trigger) {
      // auto tilt
      if (tilt_count < tilt_delay) {
        tilt_count += 1;
      }
      if (extension_indi == 0) {
        target_jump_angle = target_jump_angle_value;
        data.ch[0] = map((tilt_count / tilt_delay) * target_jump_angle, -35.0, 35.0, 307.0, 1693.0);
        // data.ch[2]=map(CH3,0,100.0,307.0,1693.0);
      }
      data.ch[1] = 1000;        
      data.ch[2] = map(target_thrust, 0, 100.0, 307.0, 1693.0);
    }

    //----------------------- back tilt upon landing (when ToF used for outdoor experiment) ------------------------------------------------
    // if (extension_indi == 1) {
    //   if (damping_region_indi == 0) {
    //     data.ch[0] = 1000;
    //   } else if (damping_region_indi == 1) {
    //     if (contact_indi == 0) {
    //       data.ch[0] = map(-target_jump_angle_value, -35.0, 35.0, 307.0, 1693.0);
    //     } else if (contact_indi == 1) {
    //       data.ch[0] = 1000;
    //     }
    //   }
    // }


    sbus_tx.data(data);
    sbus_tx.Write();

  } else if (U1t != 0 && mpc_indi == 1) {
    mpccommand(); // transfer MPC commands from Matlab to the low-level attitude controller and the thrust vectoring servos


    // ——————————————————flip motions: used for the second jump in "back style jumping task"——————————————————————————//
    // flipping motion in the demo for leaping over a 1.27 m bar
    // rotation control
    if (rate_ctrl_indi == 1) {
    
      if (flip_count <= 80) {
        data.ch[0] = 1000;
        data.ch[4] = 1000;
      }
      if (flip_count > 80 && flip_count <= 280) {
        data.ch[4] = 1693;
        data.ch[0] = 1650;
      }
      if (flip_count > 280) {
        data.ch[4] = 1000;
        data.ch[0] = 1000;
      }
    }

    if (thr_ctrl_indi == 1) {
      flip_count += 1;
      if (flip_count <= 80) {
        data.ch[2] = map(83.0, 0, 100.0, 307.0, 1693.0);
      }
      if (flip_count > 80 && flip_count <= 80 + 150) {
        //  data.ch[2]=map( 60/2 + 60/2*cos(PI/14*(flip_count-66)) +75-60, 0,100.0,307.0,1693.0);
        data.ch[2] = map(17.0, 0, 100.0, 307.0, 1693.0);
      }
      if (flip_count > 80 + 150) {
        data.ch[2] = map(50.0, 0, 100.0, 307.0, 1693.0);
      }
      if (extension_indi == 1 && dist_avg < 50) {
        data.ch[2] = map(10.0, 0, 100.0, 307.0, 1693.0);
      }
    }
    // ——————————————————flip——————————————————————————//



    sbus_tx.data(data);
    sbus_tx.Write();
  }



  if (CH7 >= 98 && CH9 == 100) {  // jump initiated; once the conditions in this if statement satisfy.
                                  
    // mpc_indi = 1;  // ---------------------put here for flight mode-----initiate mpc_indi once RC switch is triggered.

    // ————————————————————yaw set to 0 using IMU, uncomment for outdoor experiment————————————————————————————————————//
    // if (jump_indi==1){
    //  *yaw control using gyro* comment out when using mocap system
    // jump_initiated = 1;
    // if (psi_des_1st_mark < 2) { psi_des_1st_mark += 1; }
    // if (psi_des_1st_mark == 1) {
    //   if (fAngle[2] >= 0) {
    //     psi_des = fAngle[2];
    //   } else if (fAngle[2] < 0) {
    //     psi_des = 360.0 + fAngle[2];
    //   }
    // }
    // ————————————————————————————————————————————————————————//



    //define return mapping angle for alpha
    double theta_val = now_y + crr; // ranges from 120° to 245°
    double theta_val1 = now_y1 + crr1;// ranges from 120° to 245°
    // return the crouch pose from the extended pose; these two polynomials are used for active damping on the dynamic jumping and landing on the rotating platform presented in Fig. 9.
    l12_return=1000 * (-0.000000001071134*pow(theta_val,4)  + 0.000000824121177*pow(theta_val,3)  -0.000235650226661*pow(theta_val,2)  + 0.029847374378249*theta_val  -1.286412603563698);
    l22_return=-1000 *(-0.000000001071134*pow(theta_val1,4)  + 0.000000824121177*pow(theta_val1,3)  -0.000235650226661*pow(theta_val1,2) +  0.029847374378249*theta_val1  -1.286412603563698) +(i12+i22);
    // l12,l22,l12_return,l22_return will be assigned to leg servo 1.
   

    if (CH6 <= 10) { 

       // damp_st_indi: damp state indicator: 0: a "no reaction phase" after first jump; 1 damping and jumping; 2: ready for damping by giving the leg motors "soft activation"; 
       // code block below assign leg reactions according to the "damp_st_indi" assigned from matlab; used for continuous leap tasks or bounce-free landing. 
       // the code below works for continuous leap task across inclined target platform, and the leap task against wind disturbance (the damping case)
      if (damp_st_indi == 0) {
        
        // omega_tar=map(CH8,0,100.0,100.0,3000.0); omega_tar1=map(CH8,0,100.0,100.0,3000.0); // manually controlled leg extension speed, map CH8 to range 100-3000 deg/s 
        mode = 2; // into speed controlled extension mode 
        omega_tar=w_tar;
        omega_tar1=w_tar;
        // lateral jumping: using count method
        if (count_adj < 100) { count_adj += 1; } // 100 is a random bound selected for the incrementing the count.  
        // if (count_adj>=16){  // uncomment this to introduce actuation time (phase) difference in the two extension legs.
        r_assign = jump_end_angle;
        // }
        r_assign1 = jump_end_angle;

        // l12, l22 correpond to the mapping from leg motor angle to leg servo 1; only used for the first jump; for the following jumps when conducting continuous hopping, use l12_return, l22_return.
        l12 = i12 + 45.0 - (30.0 / (jump_end_angle - jump_st_angle) * (now_y + crr) - jump_st_angle * 30.0 / (jump_end_angle - jump_st_angle));
        l22 = i22 - 45.0 + (30.0 / (jump_end_angle - jump_st_angle) * (now_y1 + crr1) - jump_st_angle * 30.0 / (jump_end_angle - jump_st_angle));
        
        // ---leg servo---
          L1S1.write(i11);
          L2S1.write(i21);
          if (damping_region_indi==0){
          L1S2.write(l12);  
          L2S2.write(l22);   
          }
            if (extension_indi == 0) {
              if (CH3 > tilt_trigger) {
                L1S3.write(i13 - servo_tilt); 
                L2S3.write(i23 + servo_tilt);  
              } else if (CH3 < tilt_trigger) {
                L1S3.write(map(CH1,100,0,50,130));  
                L2S3.write(map(CH1,0,100,50,130));   
              }
            } else if (extension_indi == 1) {
              // L1S3.write(i13);  //comment out in thrust vectoring mode, use this in fixed rotor (FR) mode
              // L2S3.write(i23);  //comment out in thrust vectoring mode, use this in fixed rotor (FR) mode
              // data.ch[0]=1000; //——————————————————uncomment for open-loop case ————————————————————————————————
            }

      } else if (damp_st_indi == 1) { 
            if (damp_count<3){ // 3 (loops) is the duration between the initiation of next jump and the fully contraction state of previous jump
              mode = 3;  // speed controlled contraction mode      
              r_assign = jump_st_angle;
              r_assign1 = jump_st_angle;
              omega_damp=350; omega_damp1=350; // this is contraction speed (in deg/s) during active damping, mannually set it or automatically assigned in matlab and HWSERIAL_READ() into this variable.
              L1S2.write(l12_return);  
              L2S2.write(l22_return);  
            }else if(damp_count>=3){
              mode = 2; // speed controlled extension mode
              r_assign = jump_end_angle;
              r_assign1 = jump_end_angle;
              omega_tar=w_tar; omega_tar1=w_tar; // w_tar read from MATLAB, assigned in HWSERIAL_READ()
              L1S2.write(l12_return);  
              L2S2.write(l22_return); 
            }
            if ((now_y+crr) > jump_st_angle-5 && (now_y1+crr1) > jump_st_angle-5){
              damp_count+=1; 
            }
          //hip servos keep unchanged
          L1S1.write(i11);
          L2S1.write(i21);

      } else if(damp_st_indi == 2){
        mode = 3;        
        r_assign = jump_st_angle;
        r_assign1 = jump_st_angle;
        omega_damp=60; omega_damp1=60;  // soft activation
        damp_count=0; // refresh damp count 

        //---leg servo-----
          L1S1.write(i11);
          L2S1.write(i21);
          L1S2.write(l12_return);  //+35- ( 35/(120-240) * (now_y+crr)-240*35/(120-240))
          L2S2.write(l22_return);  //-35 +( 35/(120-240) * (now_y1+crr1)-240*35/(120-240))
      }
      


       // if leg extended
      if ((now_y + crr) <= jump_end_angle + 5 || (now_y1 + crr1) <= jump_end_angle + 5) {  
        extension_indi = 1;
        mpc_indi=1;  // mpc_indi = 1 will call mpccommand() and hence initiate parabolic trajectory regulation
      }

      if (extension_indi == 1) {
        
        if (count_delay < 10) {  //the leg shrink after 10 counts.
          count_delay += 1;
        }
        if (count_delay == 10) { damping_region_indi = 1; damp_st_indi=1;}  // damping_region_indi is only used for the case when legs will direcly contract after jumping, clear the bracket if using matlab commands     
       

        

        //—————————————————————————————————Leap through window demo—————————————————————————————————————————————————//
        // if (extension_indi==1){

        //     if(count_delay<15) {count_delay+=1;}

        //     if (count_delay==14) {shrink_indi=1;}

        //     if (now_y+crr >= jump_st_angle-3){
        //       shrink_indi=0;
        //     }

        //     if (damp_st_indi==0 || damp_st_indi==2){
        //         if (shrink_indi==1){
        //           mode=3;omega_damp=600; omega_damp1=600;
        //           r_assign=jump_st_angle; r_assign1= jump_st_angle;
        //           // extension_count +=1;
        //         }else if(shrink_indi==0){
        //           mode=0; max_thr0=17; max_thr1=17;
        //           r_assign=jump_end_angle; r_assign1=jump_end_angle;
        //         }
        //     }
    

        //———————————————————————————————————————leap onto dynamically rotating platform demo——————————————————————————————————//
      
        // if (count_delay<95){
        // count_delay+=1;
        // }

        // if (count_delay==95 && (damp_st_indi==0 ||damp_st_indi==2)){
        // mode=3; omega_damp=45; omega_damp1=45; r_assign=jump_st_angle; r_assign1=jump_st_angle;
        // }

        // if (damp_st_indi==1){
        //   shrink_indi=1;
        // }
        // if (now_y+crr >= jump_st_angle-5){
        //   shrink_indi=0;
        //   if (second_jump_count<3){ second_jump_count+=1; }else if(second_jump_count==3){ second_jump_indi=1;}
        // }
        // if (second_jump_indi==1 || second_shrink_indi == 1){ shrink_indi=0; }

        // if (shrink_indi==1){
        //   mode=3;
        //   omega_damp=700;// 700 for 0.8-0.9 meter high, 550 for 0.6 meter high
        //   omega_damp1=700;
        //   r_assign=jump_st_angle;
        //   r_assign1=jump_st_angle;
        // }
        // if (second_jump_indi==1){
        //   mode=2;omega_tar=1150; omega_tar1=1150;
        //   r_assign=jump_end_angle;
        //   r_assign1=jump_end_angle;
        //   if(now_y+crr < jump_end_angle+5){
        //      thr_ctrl_indi=1;
        //      rate_ctrl_indi=1;
        //     if (second_shrink_count<15) {second_shrink_count+=1;} else if(second_shrink_count==15){ second_shrink_indi=1; }
        //   }
        // }

        // if (second_shrink_indi==1){ second_jump_indi=0; }

        // if (second_shrink_indi==1){
        //   mode=3;omega_damp=500; omega_damp1=500;
        //   r_assign=jump_st_angle; r_assign1=jump_st_angle;
        // }

    



        //——————————————————jump-and-damp case: works for both MPC and open-loop mode ）——————————————————————————//
        
          //  if (damping_region_indi==1){
          //    mode=3; 
          //    omega_damp=500; 
          //    omega_damp1=500;
          //    r_assign=jump_st_angle;
          //    r_assign1=jump_st_angle;
          //  }
          // // back tilt
          // if (dist_avg <= 70) { // 70 mm; dist_avg is obtained from the tof() function
          //    contact_indi = 1;
          // }
          // if (contact_indi == 0) {
          //   data.ch[0] = map(-target_jump_angle_value, -35.0, 35.0, 307.0, 1693.0); // creating back-tilt 
          //   L1S3.write(i13 - (-servo_tilt));  // servo rotating the same angle
          //   L2S3.write(i23 + (-servo_tilt));  // 
          // } else if (contact_indi == 1) {
          //   data.ch[0] = 1000; // if contact the ground, let the pitch angle maintain horizontal
          //   L1S3.write(i13); // also maintaining thrust vectoring angle to be horizontal
          //   L2S3.write(i23);
          // }

      }
    }
  }

// Exit jumping, into a static phase where indicators are refreshed; crawling and wheeling motions are conducted in this else if condition
  else if (CH7 >= 5 && CH7 < 98 && CH9 >= 98) {
    // refresh indicators and counts 
    contact_indi = 0;
    thr_indi = 0;
    extension_indi = 0;
    damp_st_indi = 0;
    damping_region_indi = 0;
    count_delay = 0;
    mpc_indi = 0;
    first_jump_finish = 0;
    pre_shrink_count = 0;
    jump_indi = 0;
    jump_initiated = 0;  // for field operation, when the gyro psi angle is used to control yaw to 0 degrees.
    count_adj = 0;
    second_jump_count = 0;
    second_jump_indi = 0;
    second_shrink_count = 0;
    second_shrink_indi = 0;  
    v_enhance_count = 0;
    rate_ctrl_indi = 0;
    thr_ctrl_indi = 0;
    damp_count=0;
    mode = 0;
    
    // obtained desired leg motor rotating speed from matlab; through hwserial_read() function in this file.
    omega_tar=w_tar;
    omega_tar1=w_tar;
    l12 = i12 + 45.0 - (30.0 / (jump_end_angle - jump_st_angle) * (now_y + crr) - jump_st_angle * 30.0 / (jump_end_angle - jump_st_angle));
    l22 = i22 - 45.0 + (30.0 / (jump_end_angle - jump_st_angle) * (now_y1 + crr1) - jump_st_angle * 30.0 / (jump_end_angle - jump_st_angle));

    if (CH6 < 10) {
      //——————————————————————————————————————————mannual——————————————————————————————————————//
      //  target_jump_angle_value=map(CH6,0,10.0,5.0,25.0); servo_tilt_value=map(CH6,0,10.0,5.0,25.0)+1;
      //——________________________________________mannual——————————————————————————————————————//
      L1S1.write(i11);
      L1S2.write(l12);  //-
                        

      if (CH3 < tilt_trigger) {
        // L1S3.write(map(CH1,100,0,50,130)); // commnet out when using UAV mode
        // L2S3.write(map(CH1,0,100,50,130)); // commnet out when using UAV mode
        L1S3.write(i13);
        L2S3.write(i23);
        // Serial.print(lean_backward_indi); Serial.print(" ");
        // Serial.println(lean_forward_indi);
        //------------------------------------------------mannual---------------------------------------------------//
        //  if (CH3 <= 0.5*tilt_trigger){
        //     lean_forward_indi=0;
        //     lean_backward_indi=0;
        //  }else if (CH3> 0.5*tilt_trigger){
        //     if (CH1>=60){lean_forward_indi=1;}
        //     else if(CH1<40){ lean_backward_indi=1;}
        //   }
        //-------------------------------------------------mannual--------------------------------------------------//
      } else if (CH3 >= tilt_trigger) {
        //uncomment the code when jump direction is used in mannual control
        // if(lean_forward_indi==1) { servo_tilt=servo_tilt_value; target_jump_angle=target_jump_angle_value; }
        //else if(lean_backward_indi==1){servo_tilt=-servo_tilt_value; target_jump_angle=-target_jump_angle_value;  }
        // L1S3.write(i13-(tilt_count/tilt_delay)*servo_tilt); // bar forward
        // L2S3.write(i23+(tilt_count/tilt_delay)*servo_tilt); // bar forward
        L1S3.write(i13 - (tilt_count / tilt_delay) * servo_tilt);  // wheel forward
        L2S3.write(i23 + (tilt_count / tilt_delay) * servo_tilt);  // wheel forward
      }

      L2S1.write(i21);
      L2S2.write(l22);
      

      r_assign = jump_st_angle;
      r_assign1 = jump_st_angle;

    } else if (CH6 >= 10) {  // Use ch6 to change heading direction of a leap

      // T_heading= (76 -> 1hz; 51 -> 1.5hz 38 -> 2hz    152 -> 0.5hz; l=25, H=0.4*25 ) 
      // T_heading=map(CH7,50.0,10.0,152.0,38.0); // this code gives user-defined crawling frequency controlled by the knob of ratio controller (Channel 7).
      T_heading = 76;
      float span = map(CH6, 10.0, 50.0, 0, 40.0);  //maximum span=63 deg, exceeding this value causes mechanical interference.
      float H = 30;                                // parameter defining padding curve depth. 
      span = constrain(span, 0, 40.0);
      r_assign = jump_st_angle;
      r_assign1 = jump_st_angle;

      // -------------------------indicator assignment------------------
      if (CH6 <= 50.0) {

        if (CH4 > 55.0) {
          right_indi = 1;
          left_indi = 0;
          forward_indi = 0;
          backward_indi = 0;
          wheeling_indi = 0;
        } else if (CH4 < 45.0) {
          right_indi = 0;
          left_indi = 1;
          forward_indi = 0;
          backward_indi = 0;
          wheeling_indi = 0;
        } else if (CH2 > 55) {
          right_indi = 0;
          left_indi = 0;
          forward_indi = 1;
          backward_indi = 0;
          wheeling_indi = 0;
        } else if (CH2 < 45) {
          right_indi = 0;
          left_indi = 0;
          forward_indi = 0;
          backward_indi = 1;
          wheeling_indi = 0;
        }
          rpos_indi = 0;
      } else if (CH6 > 50 && CH6 < 70) {
        right_indi = 0;
        left_indi = 0;
        forward_indi = 0;
        backward_indi = 0;
        wheeling_indi = 0;
        rpos_indi = 1;
      } else if (CH6 >= 70 && CH6 <= 100) {
        right_indi = 0;
        left_indi = 0;
        forward_indi = 0;
        backward_indi = 0;
        wheeling_indi = 1;
        rpos_indi = 0;
      }

//--------------------------------------------------------------------------------------


      // Time-sequence-based code for crawling locomotion.
      // left: rotating counterclockwise around the robot's center; 
      // right: rotating counterclockwise around the robot's center;
      // forward: crawling forward (can make turns while crawling forward);
      // backward: crawling backward (can make turns while crawling backward).
      if (left_indi == 1) { 

        if (heading_count <= 2 * T_heading) {
          L1S2.write(-60 / (2 * T_heading) * heading_count + i12 + 40);
          L2S2.write(60 / (2 * T_heading) * heading_count + i22 - 40);
          heading_count += 1;
        }
        if (heading_count > 2 * T_heading && heading_count <= 3 * T_heading) {
          L1S1.write(i11 - 0.5 * span * cos((PI / T_heading) * (heading_count - 2 * T_heading)) + 0.5 * span);
          L1S2.write(i12 - 20 - (-0.5 * H * cos(2 * PI / T_heading * (heading_count - 2 * T_heading)) + 0.5 * H));
          heading_count += 1;  //+40
        }
        if (heading_count > 3 * T_heading && heading_count <= 4 * T_heading) {
          L2S1.write(i21 - 0.5 * span * (cos(PI / T_heading * (heading_count - T_heading - 2 * T_heading))) + 0.5 * span);
          L2S2.write(i22 + 20 - 0.5 * H * cos(2 * PI / T_heading * (heading_count - T_heading - 2 * T_heading)) + 0.5 * H);
          heading_count += 1;  //-40
        }
        if (heading_count > 4 * T_heading && heading_count <= 6 * T_heading) {
          L1S1.write(i11 + 0.5 * span * cos(PI / (2 * T_heading) * (heading_count - 2 * T_heading - 2 * T_heading)) + 0.5 * span);
          L2S1.write(i21 + 0.5 * span * cos(PI / (2 * T_heading) * (heading_count - 2 * T_heading - 2 * T_heading)) + 0.5 * span);
          heading_count += 1;
        }
        if (heading_count > 6 * T_heading && heading_count <= 8 * T_heading) {
          if (CH4 <= 45) {
            heading_count = 2 * T_heading;
          } else if (CH4 >= 55) {
            heading_count = 2 * T_heading;
          } else if (CH4 < 55 && CH4 > 45) {
            L1S2.write(60 / (2 * T_heading) * (heading_count - 6 * T_heading) + i12 - 20);
            L2S2.write(-60 / (2 * T_heading) * (heading_count - 6 * T_heading) + i22 + 20);
            heading_count += 1;
          }
        }
        if (heading_count > 8 * T_heading) {
          heading_count = 0;
          left_indi = 0;  // or right indi
        }
        pos_detect = 1; // pos_detect=1: pose indicator of crawling; 2: wheeling 
      }
      if (right_indi == 1) {

        if (heading_count <= 2 * T_heading) {
          L1S2.write(-60 / (2 * T_heading) * heading_count + i12 + 40);
          L2S2.write(60 / (2 * T_heading) * heading_count + i22 - 40);
          heading_count += 1;
        }
        if (heading_count > 2 * T_heading && heading_count <= 3 * T_heading) {
          L1S1.write(i11 - (-0.5 * span * cos(PI / T_heading * (heading_count - 2 * T_heading)) + 0.5 * span));
          L1S2.write(i12 - 20 - (-0.5 * H * cos(2 * PI / T_heading * (heading_count - 2 * T_heading)) + 0.5 * H));
          heading_count += 1;
        }
        if (heading_count > 3 * T_heading && heading_count <= 4 * T_heading) {
          L2S1.write(i21 - (-0.5 * span * (cos(PI / T_heading * (heading_count - T_heading - 2 * T_heading))) + 0.5 * span));
          L2S2.write(i22 + 20 - 0.5 * H * cos(2 * PI / T_heading * (heading_count - T_heading - 2 * T_heading)) + 0.5 * H);
          heading_count += 1;
        }
        if (heading_count > 4 * T_heading && heading_count <= 6 * T_heading) {
          L1S1.write(i11 - (+0.5 * span * cos(PI / (2 * T_heading) * (heading_count - 2 * T_heading - 2 * T_heading)) + 0.5 * span));
          L2S1.write(i21 - (+0.5 * span * cos(PI / (2 * T_heading) * (heading_count - 2 * T_heading - 2 * T_heading)) + 0.5 * span));
          heading_count += 1;
        }
        if (heading_count > 6 * T_heading && heading_count <= 8 * T_heading) {
          if (CH4 >= 55) {
            heading_count = 2 * T_heading;
          } else if (CH4 <= 45) {
            heading_count = 2 * T_heading;
          } else if (CH4 < 55 && CH4 > 45) {
            L1S2.write(60 / (2 * T_heading) * (heading_count - 6 * T_heading) + i12 - 20);
            L2S2.write(-60 / (2 * T_heading) * (heading_count - 6 * T_heading) + i22 + 20);
            heading_count += 1;
          }
        }
        if (heading_count > 8 * T_heading) {
          heading_count = 0;
          right_indi = 0;  
        }
        pos_detect = 1;
      }

      if (forward_indi==1){
        float span0,span1;
        if (CH1<=48 && CH2>55){
        span0=map(CH2,50.0,100.0,0.0,1.0)*span-map(CH1,0,50.0,1.0,0.0)*span;
        span1=map(CH2,50.0,100.0,0.0,1.0)*span;
        }else if(CH1>=52 && CH2>55){
        span0=map(CH2,50.0,100.0,0.0,1.0)*span;
        span1=map(CH2,50.0,100.0,0.0,1.0)*span-map(CH1,50.0,100.0,0.0,1.0)*span;
        }else if(CH1>48 && CH1<52 && CH2>55){
        span0=map(CH2,50.0,100.0,0,1.0)*span;
        span1=map(CH2,50.0,100.0,0,1.0)*span;
        }

        if (FB_count<=T_heading){
            L1S1.write(i11 -(- 0.5*span0*cos(PI/T_heading*(FB_count))+0.5*span0));
            L2S1.write(i21-0.5*span1*cos(PI/T_heading*FB_count)+0.5*span1);
            L1S2.write(i12 -(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            L2S2.write(i22 +(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            FB_count+=1;
        }
        if (FB_count>T_heading && FB_count <= 2*T_heading){
            L1S1.write(i11+span0*cos(PI/T_heading*FB_count));
            L2S1.write(i21-span1*cos(PI/T_heading*FB_count));
            if (CH8<50){
            L1S2.write(i12-(M/H-map(CH8,0,50.0,0,1.0))*H-map(CH8,0,50.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            L2S2.write(i22+(M/H-map(CH8,0,50.0,0,1.0))*H+map(CH8,0,50.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            }else if (CH8>=60){
            L1S2.write(i12-(M/H-map(CH8,60.0,100.0,0,1.0))*H-map(CH8,60.0,100.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            L2S2.write(i22+(M/H-map(CH8,60.0,100.0,0,1.0))*H+map(CH8,60.0,100.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            }
            FB_count += 1;
        }
        if (FB_count>2*T_heading && FB_count<=3*T_heading){
            L1S1.write(i11+span0*cos(PI/T_heading*FB_count));
            L1S2.write(i12-M -(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            L2S1.write(i21-span1*cos(PI/T_heading*FB_count));
            L2S2.write(i22+M+(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            FB_count += 1;
        }
        if (FB_count>3*T_heading){
            if (CH2>45 && CH2<55){
            forward_indi=0;
            }else{
            FB_count=T_heading;
            }
        }
        pos_detect=1;

      }

      if (backward_indi==1){
        float span0,span1;
        if (CH1<=48 && CH2<45){
        span0=map(CH2,0,50.0,1.0,0)*span-map(CH1,0,50.0,1.0,0.0)*span;
        span1=map(CH2,0,50.0,1.0,0)*span;
        }else if(CH1>=52 && CH2<45){
        span0=map(CH2,0,50.0,1.0,0)*span;
        span1=map(CH2,0,50.0,1.0,0)*span-map(CH1,50.0,100.0,0.0,1.0)*span;
        }else if(CH1>48 && CH1<52 && CH2<45){
        span0=map(CH2,0,50.0,1.0,0)*span;
        span1=map(CH2,0,50.0,1.0,0)*span;
        }

         if (FB_count<=T_heading){
            L1S1.write(i11 +(- 0.5*span0*cos(PI/T_heading*(FB_count))+0.5*span0));
            L1S2.write(i12-M -(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            L2S1.write(i21-(-0.5*span1*cos(PI/T_heading*FB_count)+0.5*span1));
            L2S2.write(i22+M +(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            FB_count+=1;
        }
        if (FB_count>T_heading && FB_count <= 2*T_heading){
            L1S1.write(i11-span0*cos(PI/T_heading*FB_count));
            L2S1.write(i21+span1*cos(PI/T_heading*FB_count));
            if (CH8<50){
            L1S2.write(i12-(M/H-map(CH8,0,50.0,0,1.0))*H-map(CH8,0,50.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            L2S2.write(i22+(M/H-map(CH8,0,50.0,0,1.0))*H+map(CH8,0,50.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            }else if(CH8>=60){
            L1S2.write(i12-(M/H-map(CH8,60.0,100.0,0,1.0))*H-map(CH8,60.0,100.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            L2S2.write(i22+(M/H-map(CH8,60.0,100.0,0,1.0))*H+map(CH8,60.0,100.0,0,1.0)*H*cos(2*PI/T_heading*FB_count));
            }
            FB_count += 1;
        }
        if (FB_count>2*T_heading && FB_count<=3*T_heading){
            L1S1.write(i11-span0*cos(PI/T_heading*FB_count));
            L1S2.write(i12-M -(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            L2S1.write(i21+span1*cos(PI/T_heading*FB_count));
            L2S2.write(i22+M+(- 0.5*H*cos(2*PI/T_heading*(FB_count))+0.5*H));
            FB_count += 1;
        }
        if (FB_count>3*T_heading){
            if (CH2>45 && CH2<55){
            backward_indi=0;
            }else{
            FB_count=T_heading;
            }
        }
        pos_detect=1;

      }
      

      if (wheeling_indi == 1) {
        float alpha = map(CH3, 0, 100, 0.8, 0.8);
        float alpha1 = 0.2;
        float A1 = 80.0, A2 = 45.0, A3 = 65.0;
        rpos_count = 0;  //notice ****//
        T_heading = 200;
        
        if (w_count < T_heading / 4) {
          L1S1.write(i11+(-A1/2*cos(PI/(2*T_heading)*w_count)+A1/2));
          L1S2.write(i12 - 20 - (-A2 / 2 * cos(4 * PI / T_heading * w_count) + A2 / 2));
          L2S1.write(i21-(-A1/2*cos(PI/(2*T_heading)*w_count)+A1/2));
          L2S2.write(i22 + 20 + (-A2 / 2 * cos(4 * PI / T_heading * w_count) + A2 / 2));
        }
        if (w_count >= T_heading / 4 && w_count < 2 * T_heading) {
          L1S1.write(i11+(-A1/2*cos(PI/(2*T_heading)*w_count)+A1/2));
          L1S2.write(i12 - A3); 
          L2S1.write(i21-(-A1/2*cos(PI/(2*T_heading)*w_count)+A1/2));
          L2S2.write(i22 + A3);
        }

        if (w_count >= 2 * T_heading && w_count < 5 * T_heading / 2) {
          L1S1.write(i11+A1);
          L1S2.write(i12-(-A3/2*cos(2*PI/T_heading*(w_count-T_heading/2))+A3/2 ));
          L2S1.write(i21-A1);
          L2S2.write(i22-A3/2*cos(2*PI/T_heading*(w_count-T_heading/2))+A3/2);
        }

        // only for rotor pitch turning ↓
        if (w_count > 2 * T_heading && w_count < 3 * T_heading) {
          L1S3.write(i13 + (-alpha * 90 / 2 * cos(PI / T_heading * w_count) + alpha * 90 / 2));    // rear servo
          L2S3.write(i23 + (-alpha1 * 90 / 2 * cos(PI / T_heading * w_count) + alpha1 * 90 / 2));  //front servo
        }
        if (w_count >= 3 * T_heading) {
          L1S3.write(i13 + alpha * 90);
          L2S3.write(i23 + alpha1 * 90);  // inversed here
        }
        if (w_count >= 5 * T_heading / 2) {
          L1S1.write(i11+A1);
          L1S2.write(i12);
          L2S1.write(i21-A1);
          L2S2.write(i22);
        }

        if (w_count <= 3 * T_heading) {
          w_count += 1;
        }
        pos_detect = 2; // wheeling pose
      }

      if (rpos_indi == 1) { // return standard crouch pose indicator
        float alpha = map(CH3, 0, 100, 0.8, 0.8);
        float alpha1 = 0.2;
        float A1 = 75.0, A2 = 45.0, A3 = 65.0;
        T_heading = 200;
        w_count = 0;           
        if (pos_detect == 2) {  // if in wheeling mode

          if (rpos_count <= T_heading / 2) {
            L1S1.write(i11+A1);
            L1S2.write(i12-(-A3/2*cos(2*PI/T_heading*rpos_count)+A3/2));
            L2S1.write(i21-A1);
            L2S2.write(i22-A3/2*cos(2*PI/T_heading*rpos_count)+A3/2);
            L1S3.write(i13 + (-alpha * 90 / 2 * cos(2 * PI / T_heading * (rpos_count - T_heading / 2)) + alpha * 90 / 2));
            L2S3.write(i23 + (-alpha1 * 90 / 2 * cos(2 * PI / T_heading * (rpos_count - T_heading / 2)) + alpha1 * 90 / 2));
          }
          //only for L1S3, L2S3
          if (rpos_count > T_heading / 2) {
            L1S3.write(i13);
            L2S3.write(i23);
          }
          if (rpos_count > T_heading / 2 && rpos_count < 3 * T_heading / 2) {
            L1S1.write(i11-A1/2*cos(PI/T_heading*(rpos_count+T_heading/2))+A1/2);
            L1S2.write(i12 - 65);
            L2S1.write(i21-(-A1/2*cos(PI/T_heading*(rpos_count+T_heading/2))+A1/2));
            L2S2.write(i22 + 65);
          }
          if (rpos_count >= 3 * T_heading / 2 && rpos_count < 7 * T_heading / 4) {
            L1S1.write(i11);
            L1S2.write(i12 - 20 - (-A2 / 2 * cos(4 * PI / T_heading * (rpos_count - T_heading / 4)) + A2 / 2));
            L2S1.write(i21);
            L2S2.write(i22 + 20 + (-A2 / 2 * cos(4 * PI / T_heading * (rpos_count - T_heading / 4)) + A2 / 2));
          }
          if (rpos_count >= 7 * T_heading / 4) {
            L1S1.write(i11);
            L1S2.write(i12 - 20);
            L2S1.write(i21);
            L2S2.write(i22 + 20);
          }

          rpos_count += 1;

        } else if (pos_detect == 1) {
          // maintain the crawling pose
          L1S1.write(i11);
          L1S2.write(i12 - 20);
          L2S1.write(i21);
          L2S2.write(i22 + 20);
          L1S3.write(i13);
          L2S3.write(i23);
        }
      }
    }

  }

}


double Kp_psi = 693.0 / 40.0, U1t_prev, phit_prev, thetat_prev, psit_prev;
void mpccommand() {
  if (U1t >= 0 && U1t <= 5.6) {
    // float x_cmd_4s = (U1t - 0.63888) / 0.07708;
    // data.ch[2] = map(x_cmd_4s, 0, 100.0, 307.0, 1693.0);  // 4s battery
    data.ch[2]=map(U1t,0.31, 3.606,307.0,307.0+(1693-307.0)*0.7); // 3s battery: U1t 0% --> 0.31N; 70% -->3.606N based on fitted curve
    // vector 模式方向phit 和thetat反向，yb轴朝前，xb轴朝右，zb朝上。
    data.ch[0] = map(phit, -35.0, 35.0, 1693.0, 307.0);
    data.ch[1] = map(thetat, -35.0, 35.0, 307.0, 1693.0);

    data.ch[3] = -Kp_psi * err_psit + 1000;  // check at 1000, whether it is the center point of angular rotation around z axis
    L1S3.write(i13 - epsit); // epsit + servo +
    L2S3.write(i23 + epsit);
    data.ch[0] = constrain(data.ch[0], 604.0, 1396.0);
    data.ch[1] = constrain(data.ch[1], 802.0, 1198.0);
    data.ch[2] = constrain(data.ch[2], 307.0, 1693.0);
    data.ch[3] = constrain(data.ch[3], 307, 1693);
  }
  // sbus_tx.data(data);
  // sbus_tx.Write();
}


void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  HWSERIAL.begin(115200);
  HWSERIAL1.begin(115200);
  // Serial5.begin(115200);
  // pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  // pinMode(7, OUTPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  

  delay(500);

  // initiate Teensy PWM instances
  PWM_Instance = new Teensy_PWM(pinToUse, frequency, dutyCYCLE);
  PWM_Instance1 = new Teensy_PWM(pinToUse1, frequency, dutyCYCLE);
  delay(2000);
 
  // sbus begin
  while (!Serial5) {}  

  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();
  delay(1000);

  // initiate I2C 
  Wire.begin();
  Wire1.begin();


  //ESC setup
  PWM_Instance->setPWM(pinToUse, frequency, 15.0d); // 15 is the middle of dutycycle for the 10000 hz multishot 
  PWM_Instance1->setPWM(pinToUse1, frequency, 15.0d);
  // delay(5000);

  //------setup JY901S gyro----------------------//
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  Serial.print("\r\n********************** wit-motion normal example  ************************\r\n");
  AutoScanSensor();


  //intializing as5600
  as5600.begin();                                 
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // values increase when motor rotating counterclockwise (top view) 
  int b = as5600.isConnected();
  as5600_1.begin();                         
  as5600_1.setDirection(AS5600_CLOCK_WISE); 
  int b1 = as5600_1.isConnected();

  Serial.print("connection ok?");
  Serial.print(b);
  Serial.print("\t");
  Serial.print(b1);

  // initializing ToF sensor
  Wire2.begin();            //This resets to 100kHz I2C
  Wire2.setClock(1000000);  //Sensor has max I2C freq of 400kHz
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1)
      ;
  }
  //set the resolution of range sensor
  myImager.setResolution(4 * 4);  // enable only 16 pads
  myImager.setRangingFrequency(60); 

  imageResolution = myImager.getResolution();  //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);          //Calculate printing width
  myImager.startRanging();


  //task scheduler initiation
  runner.init();
  runner.addTask(LED);
  runner.addTask(sbusRead);
  runner.addTask(AS5600READ);
  runner.addTask(AS5600READ1);
  runner.addTask(PRINT_DATA);
  runner.addTask(P_CONTROL);
  runner.addTask(P1_CONTROL);
  runner.addTask(MODE_SELECTION);
  runner.addTask(GYRO);
  runner.addTask(TOF);
  runner.addTask(BRAKE);
  runner.addTask(HWSERIALREAD);


  // Enable tasks
  LED.enable();
  sbusRead.enable();
  AS5600READ.enable();
  AS5600READ1.enable();
  PRINT_DATA.enable();
  // RATETEST.enable(); // for loop rate testing 
  P_CONTROL.enable();
  P1_CONTROL.enable();
  // GYRO.enable(); // uncomment this for field operations when gyro is needed and Mocap not used.
  MODE_SELECTION.enable();
  TOF.enable();
  BRAKE.enable();
  HWSERIALREAD.enable(); // wireless serial receiver; Enabled when executing parabolic trajectory regualtion, and obtained the MPC's calculations


  // Attaching servos
  L1S1.attach(0, 500, 2500);
  L1S2.attach(1, 500, 2500);
  L1S3.attach(2, 725, 2275);
  L2S1.attach(3, 500, 2500);
  L2S2.attach(4, 500, 2500);
  L2S3.attach(5, 725, 2275);
  brake.attach(12, 775, 2225);
  brake1.attach(11, 775, 2225);
  
  // assigning initial values to the servos 
  L1S1.write(i11);
  L1S2.write(i12+45);  //+35- ( 35/(120-240) * (now_y+crr)-240*35/(120-240))
  L1S3.write(i13);
  L2S1.write(i21);
  L2S2.write(i22-45);  //-35 +( 35/(120-240) * (now_y1+crr1)-240*35/(120-240))
  L2S3.write(i23);
  brake.write(90); // ranges from 0-180, assign 90 to disable the brake; values below or above 90 unlock or lock the wheels.

}


void loop() {
  runner.execute();
  

  // The block of code below is used for inputting angle position from serial monitor, the inputted value will be directly assigned to the leg motors (both leg) in position control. 
  // The purpose of this block is for calibration of the leg motor. Note the inputted values value should be ranged from 120-245 degrees (the rotation range of the leg), with 180 
  // being the desired calibration position. 
  // r_assign, r_assign1 correspond to positions of leg motor. i.e., \vartheta discussed in SI.   
  static String inputString;
  if (Serial.available()) {
    char c = Serial.read();
    // Serial.println(c);
    // Serial.println("verified");
    if (c != '\n') {
      inputString += c;

    } else {
      // Input complete, parse and process it
      double inputValue = atof(inputString.c_str());  //convert string to C string first and then convert to double
      inputString = "";                               // Reset the input buffer
                                                      //        Serial.print("Input: ");
      Serial.println(inputString);
      if (inputValue != 0) {
        // Serial.println()
        r_assign = inputValue;
        r_assign1 = inputValue;
      }

    }
  }

}






// --------------------Dependencies for the JY901S gyro sensor------------------------------------------------//
void CopeCmdData(unsigned char ucData) {
  static unsigned char s_ucData[50], s_ucRxCnt = 0;

  s_ucData[s_ucRxCnt++] = ucData;
  if (s_ucRxCnt < 3) return;  //Less than three data returned
  if (s_ucRxCnt >= 50) s_ucRxCnt = 0;
  if (s_ucRxCnt >= 3) {
    if ((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
      s_cCmd = s_ucData[0];
      memset(s_ucData, 0, 50);
      s_ucRxCnt = 0;
    } else {
      s_ucData[0] = s_ucData[1];
      s_ucData[1] = s_ucData[2];
      s_ucRxCnt = 2;
    }
  }
}
static void ShowHelp(void) {
  Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
  Serial.print("\r\n************************          HELP           ************************\r\n");
  Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
  Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
  Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
  Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
  Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
  Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
  Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void) {
  switch (s_cCmd) {
    case 'a':
      if (WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
      break;
    case 'm':
      if (WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'e':
      if (WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'u':
      if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'U':
      if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'B':
      if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else {
        Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
        Serial.print(" 115200 Baud rate modified successfully\r\n");
      }
      break;
    case 'b':
      if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else {
        Serial1.begin(c_uiBaud[WIT_BAUD_9600]);
        Serial.print(" 9600 Baud rate modified successfully\r\n");
      }
      break;
    case 'r':
      if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'R':
      if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
      else Serial.print("\r\nSet Baud Success\r\n");
      break;
    case 'C':
      if (WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c':
      if (WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'h':
      ShowHelp();
      break;
    default: break;
  }
  s_cCmd = 0xff;
}
static void SensorUartSend(uint8_t* p_data, uint32_t uiSize) {
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}
static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

static void AutoScanSensor(void) {
  int i, iRetry;

  for (i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    Serial1.begin(c_uiBaud[i]);
    Serial1.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial3.available()) {
        WitSerialDataIn(Serial3.read());
      }
      if (s_cDataUpdate != 0) {
        Serial.print(c_uiBaud[i]);
        Serial.print(" baud find sensor\r\n\r\n");
        ShowHelp();
        return;
      }
      iRetry--;
    } while (iRetry);
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}

