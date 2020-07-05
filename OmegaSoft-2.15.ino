/* Delta Space Systems
      Version 2.18
    June, 28th 2020 */

/* Function Guide - README
 *  
 *  
 *  
 */










//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <MPU6050_tockn.h>
#include <math.h>
#define SEALEVELPRESSURE_HPA (1013.25)


MPU6050 mpu6050(Wire);

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();


const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, previouslog, OutX, OutY, OutZ, OreX, OreY, OreZ;
double PreviousGyroX, PreviousGyroY, PreviousGyroZ, IntGyroX, IntGyroY, RADGyroX, RADGyroY, RADGyroZ, RawGyZ, DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ, matrix1, matrix2, matrix3;
double matrix4, matrix5, matrix6, matrix7, matrix8, matrix9, PrevGyX, PrevGyY, PrevGyZ, RawGyX, RawGyY;



//Upright Angle of the Flight Computer
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoX_offset = -110;
int servoY_offset = 118;

//Position of servos through the startup function
int servoXstart = 110;
int servoYstart = 109;

//The amount the servo moves by in the startup function
int servo_start_offset = 20;

//Ratio between servo gear and tvc mount
float servo_gear_ratio = 5.8;

imu::Vector<3> Gyro;
imu::Vector<3> Accel;

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
double Ax;
double Ay;
double Az;
double accAngleX;
double accAngleY;
double yaw;
double GyroX;
double gyroAngleX;
double gyroAngleY;
double pitch;
int rollmax = 180;
int rollmin = -180;
int ledblu = 2;    // LED connected to digital pin 9
int ledgrn = 5;    // LED connected to digital pin 9
int ledred = 6;    // LED connected to digital pin 9
int mosfet = 25;
int teensyled = 13;

Servo servoX;
Servo servoY;

int pyro1 = 24;
int buzzer = 21;

double dt, currentTime, previousTime;

//"P" Constants
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants
float pidX_d = 0;
float pidY_d = 0;

int mass = 450;
int pos;

//PID Gains
double kp = 0.15;
double ki = 0.02;
double kd = 0.04;


unsigned long pitchoverTime = 0;
const unsigned long period = 5;

double rocketmass = 0.5; //rocket mass in kg
double CoG = 8;
double torque;
double theta;
/*System State: 
 * 0 = Go/No Go before launch
 * 1 = PID Controlled Ascent
 * 2 = Parachute Controlled Descent
 */
int state;
 const long flightint = 100;
 double prevdt = 0;

void setup(){
 
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  
  servoX.attach(29);
  servoY.attach(30);
  
  pinMode(mosfet, OUTPUT);
  pinMode(ledblu, OUTPUT);
  pinMode(ledgrn, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  pinMode(teensyled, OUTPUT);

  
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  
  startup();
  launchpoll();
  
}
void loop() {
  
  //Defining Time Variables      
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  launchdetect();
  //sdwrite();
  
  previousTime = currentTime;  

}

void rotationmatrices () {
  
  PreviousGyroX = RADGyroX;
  PreviousGyroY = RADGyroY;
  PreviousGyroZ = RADGyroZ;
  
  RADGyroX = mpu6050.getGyroAngleY() * (PI / 180);
  RADGyroY = mpu6050.getGyroAngleX() * (PI / 180);
  RADGyroZ = mpu6050.getGyroAngleZ() * (PI / 180);
  
  
  DifferenceGyroX = (RADGyroX - PreviousGyroX);
  DifferenceGyroY = (RADGyroY - PreviousGyroY);
  DifferenceGyroZ = (RADGyroZ - PreviousGyroZ);

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;
  
 //X Matrices
  matrix1 = (cos(DifferenceGyroZ) * cos(DifferenceGyroY));
  matrix2 = (((sin(DifferenceGyroZ) * -1) * cos(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix3 = ((sin(DifferenceGyroZ) * sin(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));
  
 //Y Matrices
  matrix4 = sin(DifferenceGyroZ) * cos(DifferenceGyroY);
  matrix5 = ((cos(DifferenceGyroZ) * cos(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix6 = (((cos(DifferenceGyroZ) * -1) * sin(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));

 //Z Matrices
  matrix7 = (sin(DifferenceGyroY)) * -1;
  matrix8 = cos(DifferenceGyroY) * sin(DifferenceGyroX);
  matrix9 = cos(DifferenceGyroY) * cos(DifferenceGyroX);

 
 OrientationX = ((OreX * matrix1)) + ((OreY * matrix2)) + ((OreZ * matrix3));
 OrientationY = ((OreX * matrix4)) + ((OreY * matrix5)) + ((OreZ * matrix6));
 OrientationZ = ((OreX * matrix7)) + ((OreY * matrix8)) + ((OreZ * matrix9));



OutX = OrientationX * 60;
OutY = OrientationY * -60;

Ax = asin(OrientationX) * (-180 / PI);
Ay = asin(OrientationY) * (180 / PI);
Az = asin(OrientationZ) * (180 / PI);
pidcompute();

}


void pidcompute () {
previous_errorX = errorX;
previous_errorY = errorY; 

errorX = Ax - desired_angleX;
errorY = Ay - desired_angleY;

//Defining "P" 
pidX_p = kp*errorX;
pidY_p = kp*errorY;

//Defining "D"
pidX_d = kd*((errorX - previous_errorX)/dt);
pidY_d = kd*((errorY - previous_errorY)/dt);

//Defining "I"
pidX_i = ki * (pidX_i + errorX * dt);
pidY_i = ki * (pidY_i + errorY * dt);

//Adding it all up
PIDX = pidX_p + pidX_i + pidX_d;
PIDY = pidY_p + pidY_i + pidY_d;

pwmY = ((PIDY * servo_gear_ratio) + servoY_offset);


pwmX = -1*((PIDX * servo_gear_ratio) + servoX_offset); 

Serial.println(previous_errorX);

//Servo outputs
servoX.write(pwmX);
servoY.write(pwmY);

}
void startup () {
  
  digitalWrite(ledgrn, HIGH);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
  delay(1000);
  digitalWrite(ledgrn, LOW);
  digitalWrite(buzzer, HIGH);
  digitalWrite(ledred, HIGH);
  servoX.write(servoXstart + servo_start_offset);
  delay(200);
  digitalWrite(buzzer, LOW);
  servoY.write(servoYstart + servo_start_offset);
  delay(200);
  digitalWrite(ledred, LOW);
  digitalWrite(buzzer, HIGH);
  digitalWrite(ledblu, HIGH);
  servoX.write(servoXstart);
  delay(200);
  digitalWrite(buzzer, LOW);
  servoY.write(servoYstart);
  delay(200);
  digitalWrite(ledblu, LOW);
  digitalWrite(buzzer, HIGH);
  digitalWrite(ledgrn, HIGH);
  digitalWrite(teensyled, HIGH);
  servoX.write(servoXstart - servo_start_offset);
  delay(200);
  digitalWrite(buzzer, LOW);
  
  servoY.write(servoYstart - servo_start_offset);
  delay(200);
  servoX.write(servoXstart);
  delay(200);
  servoY.write(servoYstart);
  delay(500);
  
 }
void launchdetect () {
  mpu6050.update();
  if (state == 0 && mpu6050.getAccZ() > 1.6) {
  state++;
 }
  if (state == 1) {
  rotationmatrices();
  datadump();
  
 }
}
void datadump () {
//Serial.print("Acc Data");
//Serial.println(mpu6050.getAccX());
//Serial.println(mpu6050.getAccZ() * 10);
//Serial.println(mpu6050.getAccY());
}


void sdwrite () {
  if (dt - prevdt >= flightint) {
    prevdt = dt;
 
 String datastring = "";
      
 datastring += String(Ay);
 datastring += "\t";
 datastring += "yaw";
 
 datastring += String(Ax);
 datastring += "\t";
 datastring += "pitch";

 
  File dataFile = SD.open("FrontierF2.txt", FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(datastring);
    dataFile.close();
   
   
  }
  }
}


void recov () {
  
if (state == 1 && mpu6050.getAccZ() < -1) {
  state++;
}

if (state == 2) {
  Serial.println("Recovery detected, prepping parachutes for deployment");
  
}
}
void launchpoll () {
  state == 0;
  delay(750);
  Serial.println("Omega is Armed");
  mpu6050.calcGyroOffsets(true); 
  delay(500);
  Serial.println("Xenon has calibrated the Gyros, we are ready for launch");
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledred, HIGH);
}

void serialread () {
  //if (Serial.available() > 0) {
    //launchbyte = Serial.read();
    //if (launchbyte == "Launch") {
      //Serial.println("Launch Detected");
   // }
  //}
}
