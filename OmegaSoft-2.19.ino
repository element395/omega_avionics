/* Delta Space Systems
      Version 2.18
    July, 5th 2020 */

/* Tuning Guide - README
 *  To tune the servo offsets for your TVC Mount go to line 52. 
    To tune just raise the values until the servo switches directions then home in on the perfect value.
 *  To change the ratio between the Servo and the TVC mount go to line 64. 
    Use a seperate sketch and raise the degree the servo moves until it hits the end of the TVC Mount. 
    Then divide the servo degree by 5.
 *  For PID Value Changes go to line 112 - 114.
 *  To change what I/O pins the servos are connected to go to line 139 - 140.  
 */


/*System State: 
 * 0 = Go/No Go before launch
 * 1 = PID Controlled Ascent
 * 2 = Parachute Controlled Descent
 */


//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_BMP3XX.h"
#include <MPU6050_tockn.h>
#include <math.h>
#define SEALEVELPRESSURE_HPA (1013.25)


MPU6050 mpu6050(Wire);

Adafruit_BMP3XX bmp;

const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, previouslog, OutX, OutY, OutZ, OreX, OreY, OreZ;
double PreviousGyroX, PreviousGyroY, PreviousGyroZ, IntGyroX, IntGyroY, RADGyroX, RADGyroY, RADGyroZ, RawGyZ, DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ, matrix1, matrix2, matrix3;
double matrix4, matrix5, matrix6, matrix7, matrix8, matrix9, PrevGyX, PrevGyY, PrevGyZ, RawGyX, RawGyY;



//Upright Angle of the Flight Computer
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoY_offset = -87;
int servoX_offset = 157;

//Position of servos through the startup function
int servoXstart = 87;
int servoYstart = 157;
//X Servo

//The amount the servo moves by in the startup function
int servo_start_offset = 20;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 7;
float servoY_gear_ratio = 9;

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
double Ax;
double Ay;
double Az;

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


//PID Gains
double kp = 0.08;
double ki = 0.02;
double kd = 0.03;

 
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
  sdstart();
  launchpoll();
  
}
void loop() {
  
  //Defining Time Variables      
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  launchdetect();
  datadump();
  sdwrite();
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

pwmY = ((PIDY * servoY_gear_ratio) + servoX_offset);
pwmX = -1*((PIDX * servoX_gear_ratio) + servoY_offset); 

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
  
  if ((servoX.read() == servoXstart + servo_start_offset) && (servoY.read() == servoYstart + servo_start_offset)) {
  digitalWrite(teensyled, HIGH);
  
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
  else {
    setup();
  }
 }
void launchdetect () {
  mpu6050.update();
  if (state == 0 && mpu6050.getAccZ() > 1.6) {
  state++;
 }
  if (state == 1) {
  rotationmatrices();
  recov();
  
  
 }
}
void datadump () {
  if (state >= 1) { 
/*Serial.print("Pitch:  ");
Serial.print(Ax);
Serial.print(" Roll:  ");
Serial.print(Ay);
Serial.print(" Yaw:  ");
Serial.print(mpu6050.getGyroAngleZ());*/
//Serial.print(mpu6050.getAccZ());
  }
//Serial.print(" System State:  ");
//Serial.println(state);

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
 
 datastring += "Pitch: "; 
 datastring += String(Ax);
 datastring += "\t";

 datastring += "Yaw: ";
 datastring += String(Ay);
 datastring += "\t";
 
 datastring += "Roll: ";
 datastring += String(mpu6050.getGyroAngleZ());
 datastring += "\t";
 
 datastring += "System State: ";
 datastring += String(state);
 datastring += "\t";

 datastring += "X Axis TVC Angle: ";
 datastring += String(servoX.read() / servoX_gear_ratio);
 datastring += "\t";

 datastring += "Y Axis TVC Angle: ";
 datastring += String(servoY.read() / servoY_gear_ratio);
 datastring += "\t";
 
 datastring += "Z Axis Accel: ";
 datastring += String(mpu6050.getAccZ());
 datastring += "\t";

 datastring += "Time: ";
 datastring += String(millis() / 10);
 datastring += "\t";
  
 
  File dataFile = SD.open("fv.txt", FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(datastring);
    dataFile.close();
   
  }
  }



void recov () {
  
if (state == 1 && mpu6050.getAccZ() < -0.125 && (((Ax > 15 || Ax < -15) && (Ay > 0 || Ay < 0)) || ((Ax > 0 || Ax < 0) && (Ay > 15 || Ay < -15)))) {
  state++;
  digitalWrite(teensyled, LOW);
  delay(600);
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
