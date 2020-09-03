/* Delta Space Systems
      Version 1.2
    May, 8th 2020*/

#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include<Wire.h>

//#include <KalmanFilter.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
 double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, gyroXX, gyroYY;
 
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

int servoX_offset = 100;
int servoY_offset = 100;

int servoXstart = 80;
int servoYstart = 80;

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

float pidX_p = 0;
float pidX_i = 0;
float pidX_d = 0;
float pidY_p = 0;
float pidY_i = 0;
float pidY_d = 0;

int pos;

double kp = 0.15;
double ki = 0.0;
double kd = 0.0;

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
  double prevgyroX = GyroZ;
  double prevgyroY = GyroY;
  
  //converting angular acceleration to degrees
  gyroAngleX +=  (((GyroZ + prevgyroX) / 2)* dt); // deg/s * s = deg
  gyroAngleY +=  (((GyroY + prevgyroY) / 2)* dt);
  
  //complimentary filter
  double OrientationX = 0.94 * gyroAngleX + 0.06 * accAngleX;
  double OrientationY = 0.94 * gyroAngleY + 0.06 * accAngleY;
  
  //divided by 32.8 as recommended by the datasheet
  pitch = OrientationX / 32.8;
  yaw = OrientationY / 32.8;
  pidcompute();
}

void servowrite() {

 servoX.write(pwmX);
 servoY.write(pwmY); 

}
void pidcompute () {
  
previous_errorX = errorX;
previous_errorY = errorY; 

errorX = pitch - desired_angleX;
errorY = yaw - desired_angleY;

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
  servoX.write(servoXstart + 20);
  delay(200);
  digitalWrite(buzzer, LOW);
  servoY.write(servoYstart + 20);
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
  servoX.write(servoXstart - 20);
  delay(200);
  digitalWrite(buzzer, LOW);
  
  servoY.write(servoYstart - 20);
  delay(200);
  servoX.write(servoXstart);
  delay(200);
  servoY.write(servoYstart);
  delay(500);
 }
void launchdetect () {
   if (AcX > 17000) {
  state = 1;
 }
 if (state == 1) {
  accel_degrees();
 }
}
void datadump () {
Serial.println(AcX);
Serial.print(AcZ);
}
