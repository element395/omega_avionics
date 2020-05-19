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
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, previouslog, OutX, OutY, OutZ, OreX, OreY, OreZ;
double PreviousGyroX, PreviousGyroY, PreviousGyroZ, IntGyroX, IntGyroY, IntGyroZ, DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ, matrix1, matrix2, matrix3;
double matrix4, matrix5, matrix6, matrix7, matrix8, matrix9, Ax, Ay;

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

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
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
double kp = 0.2;
double ki = 0.0;
double kd = 0.05;

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
  mpu6050.update();
  //Defining Time Variables
  previousTime = currentTime;        
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  datadump();
  launchdetect();
  sdwrite();
  rotationmatrices();

}

void gyrocalibrate () {
  mpu6050.calcGyroOffsets(true); 
}


void rotationmatrices () {
  mpu6050.update();
  PreviousGyroX = IntGyroX;
  PreviousGyroY = IntGyroY;
  PreviousGyroZ = IntGyroZ;

  IntGyroX = mpu6050.getGyroAngleX() * (PI / 180);
  IntGyroY = mpu6050.getGyroAngleY() * (PI / 180);
  IntGyroZ = mpu6050.getGyroAngleZ() * (PI / 180);

  DifferenceGyroX = (IntGyroX - PreviousGyroX);
  DifferenceGyroY = (IntGyroY - PreviousGyroY);
  DifferenceGyroZ = (IntGyroZ - PreviousGyroZ);

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

 
Serial.println(Ax);


 
OutX = OrientationX * 60;
OutY = OrientationY * -60;

Ax = asin(OrientationX) * (-180 / PI);
Ay = asin(OrientationY) * (180 / PI);


pidcompute();

  
}


void servowrite() {

 servoX.write(pwmX);
 servoY.write(pwmY); 

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
  if (mpu6050.getAccZ() > 1) {
  state = 1;
 }
  if (state == 1) {
  
  
 }
}
void datadump () {
  
  
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
