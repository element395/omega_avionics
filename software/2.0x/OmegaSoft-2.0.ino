/* Delta Space Systems
      Version 2.0
    May, 18th 2020 */

//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include<Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <MPU6050_tockn.h>

#define SEALEVELPRESSURE_HPA (1013.25)


MPU6050 mpu6050(Wire);

Adafruit_BMP3XX bmp;

const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, OrientationX, OrientationY, previouslog;

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

//SD CARD CS
const int chipSelect = BUILTIN_SDCARD;

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
double kp = 0.6;
double ki = 0.0;
double kd = 0.0;

int state = 0;


void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  gyrocalibrate();
  
  servoX.attach(29);
  servoY.attach(30);
  
  pinMode(mosfet, OUTPUT);
  pinMode(ledblu, OUTPUT);
  pinMode(ledgrn, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  
  startup();
  sdstart();
 
}
void loop() {
  
  //Defining Time Variables
  previousTime = currentTime;        
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  datadump();
  launchdetect();
  mpu6050.update();
  sdwrite();

}

void gyrocalibrate () {
  mpu6050.calcGyroOffsets(true); 
}
void servowrite() {

 servoX.write(pwmX);
 servoY.write(pwmY); 

}
void pidcompute () {

previous_errorX = errorX;
previous_errorY = errorY; 

errorX = mpu6050.getAngleX() - desired_angleX;
errorY = mpu6050.getAngleY() - desired_angleY;

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
//abortstart();

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
  if (mpu6050.getAccZ() > 1) {
  state = 1;
 }
  if (state == 1) {
  pidcompute();
  
 }
}
void datadump () {
  Serial.println(mpu6050.getAngleY());
  
  /*Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");*/

  
  
}
void abortstart () {
 
  if (OrientationX > 40 || OrientationY > 40) {
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledred, HIGH);
}
    else if (OrientationX < 40 || OrientationY < 40) {
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledred, HIGH);
}
}


void sdstart () { 
if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
}

void sdwrite () {
      String datastring = "";
  
 datastring += String(OrientationX);
 datastring += "\t";
 
 datastring += String(OrientationY);
 datastring += "\t";
 
 datastring += String(AcX);
 datastring += "\t";
 
 datastring += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
 datastring += "\t";

 datastring += String(bmp.temperature);
 datastring += "\t";
 
 
  File dataFile = SD.open("dat.txt", FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(datastring);
    dataFile.close();
   // Serial.println(datastring);
   
  }
}
