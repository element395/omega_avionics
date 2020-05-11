/* Delta Space Systems
      Version 1.3
    May, 11th 2020 */

//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include<Wire.h>


const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, OrientationX, OrientationY;

//Upright Angle of the Flight Computer
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoX_offset = 100;
int servoY_offset = 100;

//Position of servos through the startup function
int servoXstart = 80;
int servoYstart = 80;

//The amount the servo moves by in the startup function
int servo_start_offset = 20;

//Ratio between servo gear and tvc mount
float servo_gear_ratio = 5.8;

double accAngleX;
double accAngleY;
double yaw;
double GyroX;
double gyroAngleX;
double gyroAngleY;
double pitch;

int ledblu = 2;    // LED connected to digital pin 9
int ledgrn = 5;    // LED connected to digital pin 9
int ledred = 6;    // LED connected to digital pin 9
int mosfet = 24;

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


int pos;

//PID Gains
double kp = 0.19;
double ki = 0.02;
double kd = 0.05;

int state = 0;
//KalmanFilter kalman(0.001, 0.003, 0.03);

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  Serial.begin(9600);
  
  servoX.attach(29);
  servoY.attach(30);
  
  pinMode(mosfet, OUTPUT);
   pinMode(ledblu, OUTPUT);
  pinMode(ledgrn, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  
 startup();

 
}
void loop() {
  
  //Defining Time Variables
  previousTime = currentTime;        
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
 
  accAngleX = (atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI) + 1.58; // 
 
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyroX=Wire.read()<<8|Wire.read();  
  GyroY=Wire.read()<<8|Wire.read();  
  GyroZ=Wire.read()<<8|Wire.read();  
  datadump();
  launchdetect();
  
}
void accel_degrees () {
  double prevgyroX = pitch;
  double prevgyroY = yaw;
  
  //converting angular acceleration to degrees
  gyroAngleX +=  (((pitch + prevgyroX) / 2)* dt); // deg/s * s = deg
  gyroAngleY +=  (((yaw+ prevgyroY) / 2)* dt);
  
  //complimentary filter
  OrientationX = 0.98 * gyroAngleX + 0.02 * accAngleX;
  OrientationY = 0.98 * gyroAngleY + 0.02 * accAngleY;
  
  //divided by 32.8 as recommended by the datasheet
  pitch = GyroZ / 131;
  yaw = GyroY / 131;
  pidcompute();
}

void servowrite() {

 servoX.write(pwmX);
 servoY.write(pwmY); 

}
void pidcompute () {
  
previous_errorX = errorX;
previous_errorY = errorY; 

errorX = OrientationX - desired_angleX;
errorY = OrientationY - desired_angleY;

//Defining "P" 
pidX_p = kp*errorX;
pidY_p = kp*errorY;

//Defining "D"
pidX_d = kd*((errorX - previous_errorX)/dt);
pidY_d = kd*((errorY - previous_errorY)/dt);

//Defining "I"
pidX_i = ki * (pidX_i + errorX * dt);
pidY_i = ki * (pidY_i + errorY * dt);

PIDX = pidX_p + pidX_i + pidX_d;
PIDY = pidY_p + pidY_i + pidY_d;

pwmX = ((PIDX * servo_gear_ratio) + servoX_offset);
pwmY = ((PIDY * servo_gear_ratio) + servoY_offset);
servowrite();

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
  if (AcX > 16000) {
  state = 1;
 }
  if (state == 1) {
  accel_degrees();
 }
}
void datadump () {
  Serial.print(pitch);
  Serial.print(yaw);
  Serial.println(servoX.read());
  Serial.println(AcX);
  Serial.print(AcZ);
}
