/* Delta Space Systems
      Version 2.2
    August, 8th 2020 */

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
 * 2 = MECO
 * 3 = Abort
 */


//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <math.h>
#include "BMI088.h"

/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

//MPU6050 mpu6050(Wire); Un-comment for the mpu6050 IMU

double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, previouslog, OutX, OutY, OutZ, OreX, OreY, OreZ;
double PreviousGyroX, PreviousGyroY, PreviousGyroZ, IntGyroX, IntGyroY, RADGyroX, RADGyroY, RADGyroZ, RawGyZ, DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ, matrix1, matrix2, matrix3;
double matrix4, matrix5, matrix6, matrix7, matrix8, matrix9, PrevGyX, PrevGyY, PrevGyZ, RawGyX, RawGyY, GyroAngleX, GyroAngleY, GyroAngleZ, GyroRawX, GyroRawY, GyroRawZ;

//Upright Angle of the Flight Computer
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoY_offset = -100;
int servoX_offset = 157;

//Position of servos through the startup function
int servoXstart = servoY_offset * -1;
int servoYstart = servoX_offset;

//The amount the servo moves by in the startup function
int servo_start_offset = 20;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 8;
float servoY_gear_ratio = 8;

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
double Ax;
double Ay;

int ledred = 2;    // LED connected to digital pin 9
int ledblu = 5;    // LED connected to digital pin 9
int ledgrn = 6;    // LED connected to digital pin 9
int pyro2 = 25;
int pyro1 = 24;
int buzzer = 21;
int teensyled = 13;

Servo servoX;
Servo servoY;

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
double kp = 0.11;
double ki = 0.08;
double kd = 0.035;

int state;
 
void setup(){
 
  Serial.begin(9600);
  Wire.begin();
  //mpu6050.begin(); Un-comment fo the mpu6050
  servoX.attach(29);
  servoY.attach(30);
  
  pinMode(pyro2, OUTPUT);
  pinMode(ledblu, OUTPUT);
  pinMode(ledgrn, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  pinMode(teensyled, OUTPUT);

  
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
  previousTime = currentTime;  

}

void rotationmatrices () {
  
  //Divide Gyro rates by 65.5 for the sensitivity
  GyroRawX = (gyro.getGyroX_rads() / 65.5);
  GyroRawY = (gyro.getGyroY_rads() / 65.5);
  GyroRawZ = (gyro.getGyroZ_rads() / 65.5);

  //Integrate over time to get Local Orientation
  GyroAngleX += GyroRawX * dt;
  GyroAngleY += GyroRawY * dt;
  GyroAngleZ += GyroRawZ * dt;

  PreviousGyroX = RADGyroX;
  PreviousGyroY = RADGyroY;
  PreviousGyroZ = RADGyroZ;
  
  RADGyroX = GyroAngleX;
  RADGyroY = GyroAngleY;
  RADGyroZ = GyroAngleZ;
  
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

Ax = asin(OrientationX) * (-180 / PI);
Ay = asin(OrientationY) * (180 / PI);
pidcompute();

}

void pidcompute () {
previous_errorX = errorX;
previous_errorY = errorY; 

errorX = Ax - desired_angleX;
errorY = Ay - desired_angleY;

//Defining "P" 
pidX_p = kp * errorX;
pidY_p = kp * errorY;

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
  
  digitalWrite(ledblu, HIGH);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
  delay(1000);
  digitalWrite(ledblu, LOW);
  digitalWrite(ledgrn, HIGH);
  servoX.write(servoXstart + servo_start_offset);
  delay(200);
  servoY.write(servoYstart + servo_start_offset);
  digitalWrite(teensyled, HIGH);
  delay(200);
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledred, HIGH);
  servoX.write(servoXstart);
  delay(200);
  servoY.write(servoYstart);
  delay(200);
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, HIGH);
  servoX.write(servoXstart - servo_start_offset);
  delay(200);
  
  servoY.write(servoYstart - servo_start_offset);
  delay(200);
  servoX.write(servoXstart);
  delay(200);
  servoY.write(servoYstart);
  delay(500);
  
  
 }
void launchdetect () {
  gyro.readSensor();
  accel.readSensor();
  if (state == 0 && accel.getAccelZ_mss() > 12) {
  state++;
  }
  if (state == 1) {
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, HIGH);
  //burnout();
  //abortsystem();
  sdwrite();
  rotationmatrices();
 }
}
void datadump () {
  if (state >= 1) { 
//Serial.print("Pitch:  ");
//Serial.println(Ax);
//Serial.print(" Roll:  ");
//Serial.print(Ay);
//Serial.print(" Yaw:  ");
//Serial.print(mpu6050.getGyroAngleZ());
//Serial.println(mpu6050.getAccZ());

  }
Serial.print(" System State:  ");
Serial.println(state);

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
 
 datastring += "Pitch,"; 
 datastring += String(Ax);
 datastring += ",";

 datastring += "Yaw,";
 datastring += String(Ay);
 datastring += ",";
 
 datastring += "Roll,";
 datastring += String(GyroAngleZ);
 datastring += ",";
 
 datastring += "System_State,";
 datastring += String(state);
 datastring += ",";

 datastring += "X_Axis_TVC_Angle,";
 datastring += String(servoX.read() / servoX_gear_ratio);
 datastring += ",";

 datastring += "Y_Axis_TVC_Angle,";
 datastring += String(servoY.read() / servoY_gear_ratio);
 datastring += ",";
 
 datastring += "Z_Axis_Accel,";
 datastring += String(accel.getAccelZ_mss());
 datastring += ",";

 datastring += "Time,";
 datastring += String(millis() - 10000);
 datastring += ",";

 datastring += "TVC_X_int,";
 datastring += String(pidX_i);
 datastring += ",";

 datastring += "TVC_Y_int,";
 datastring += String(pidY_i);
 datastring += ",";

 datastring += "IMU_Temp,";
 datastring += String(accel.getTemperature_C());
 //datastring += ",";
  
 
  File dataFile = SD.open("f8.txt", FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(datastring);
    dataFile.close();
   
  }
  }



void burnout () { 
if (state == 1 && accel.getAccelZ_mss() <= 2) {
  state++;
  digitalWrite(teensyled, LOW);
  digitalWrite(ledred, HIGH);
  //Serial.println("Burnout Detected");
  delay(1000);
  recov();
  
}
}

void recov () {
digitalWrite(pyro2, HIGH);
}

void launchpoll () {
  state == 0;
  delay(750);
  Serial.println("Omega is Armed");
  //mpu6050.calcGyroOffsets(true); Un-comment for the mpu6050 IMU
     int status;
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error"); 
    while (1) {}
  }
  delay(500);
  Serial.println("Frontier has calibrated the Gyros, we are ready for launch");
  digitalWrite(ledgrn, HIGH);
  digitalWrite(ledred, HIGH);
  digitalWrite(ledblu, LOW);
  
}
void abortsystem () {
  if (state == 1 && (Ax > 25 || Ax < -25) || (Ay > 25 || Ay < -25)){
    //Serial.println("Abort Detected");
    digitalWrite(ledblu, LOW);
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledred, HIGH);
    digitalWrite(teensyled, LOW);
    state++;
    state++;
    recov();
  }
}
