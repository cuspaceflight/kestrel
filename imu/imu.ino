//Libraries
#include <Wire.h> //The Wire library is used for I2C communication
#include <math.h> //Maths Library
#include <SD.h> //SD library
#include <MatrixMath.h> //matrix library

//I2C devices from https://github.com/jrowberg/i2cdevlib
#include <I2Cdev.h>
#include <ITG3200.h>
#include <ADXL345.h>

ADXL345 accelerometer;
ITG3200 gyro;

//Global variables
File textFile; //holds information on file ebing written to on SD card
File dataFile;

const int greenLedPin = 8;  // green LED is connected to pin 8
const int redLedPin = 7;  // red LED is connected to pin 8
const int recoveryPin =6; //pin attached to mosfet for deploying chute

uint32_t time_for_loop; //time for a loop
uint32_t launch_time;

float Gsensitivity = 1 / 14.375; // deg.s^-1 / LSB
float GsensitivityR; //sensitivity in radians
float Asensitivity = 1 / 3.262;  // m.s^-2 / LSB: 26.093 for 2g/full resolution, 13.048g for 4g, 6.524 for 8g, 3.262 for 16g, these are typical value and may not be accurate
float GxOff, GyOff, GzOff; //gyro offset values
int Ax, Ay, Az;
float Accx, Accy, Accz, Mag_acc, Acc_error; //variables for acceleration in m/s^2
int Ax_off, Ay_off, Az_off;

//direction variables
float w[3]; //angular velocity vector
float initialR[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} }; //indentity matrix
float initialR2[3][3]; 
float R1[3][3]; //rotation matrix 1
float R2[3][3]; //rotation matrix 2

//lateral variables
float acceleration[3]= {0,0,0}; // (x,y,z) vector
float velocity[3]= {0,0,0}; // (x,y,z) vector

void sendDataBack() {
  dataFile.seek(0);
  while (dataFile.available()) {
    Serial.write(dataFile.read());
  }
}

bool hasLaunched = false;

//prototypes
void PID();
void closedown();
void serialcubeout(float* M, int length);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");
  delay(2000);

  GsensitivityR = Gsensitivity * M_PI / 180; //convert degrees per second to radians per second
  Matrix.Copy((float*) initialR, 3, 3, (float*) R1);

  pinMode(10, OUTPUT); //set SD CS pin to output
  pinMode(greenLedPin, OUTPUT); // Set the LED 1 pin as output
  pinMode(redLedPin, OUTPUT); // Set the LED 2 pin as output
  
  Wire.begin(); //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.
  
  /*
  SD.begin(); //begin SDness

  textFile = SD.open("Log_text.txt", FILE_WRITE); //create file on SD card

  // create a new file 
  char filename[] = "LOG00.dat"; 
  for (uint8_t i = 0; i < 100; i++) { 
    filename[3] = i/10 + '0'; 
    filename[4] = i%10 + '0'; 
    if (! SD.exists(filename)) { 
      // only open a new file if it doesn't exist 
      dataFile = SD.open(filename, FILE_WRITE); 
      break; // leave the loop! 
      } 
    }
    
  textFile.print("data file is ");
  textFile.print(filename);
   */
  
  if(accelerometer.testConnection()) Serial.println(F("Accelerometer connected!"));
  if(gyro.testConnection()) Serial.println(F("Gyro connected!"));
  
  //configure the gyro
  gyro.setFullScaleRange(ITG3200_FULLSCALE_2000); //only ITG3200_FULLSCALE_2000 is documented
  gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);
  gyro.setRate(9); // = 100Hz - I think the time for a sample should match the loop time

  // configure accelerometer
  accelerometer.initialize();
  accelerometer.setRange(ADXL345_RANGE_16G);
  accelerometer.setRate(ADXL345_RATE_100); //Hz
  accelerometer.setMeasureEnabled(true);
  accelerometer.setOffset(0, 0, 0);  //for some strange reason setting these to 0 gave more false readings

  delay(1000); //make sure everything is static
  
  Serial.print(" w[0] ");
  Serial.print(gyro.getRotationX());
  Serial.print(" w[1] ");
  Serial.print(gyro.getRotationY());
  Serial.print(" w[2] ");
  Serial.println(gyro.getRotationZ());
     //Gyro Calibration
  long GxCal=0, GyCal=0, GzCal=0; 
  int n=200;
  for (uint8_t i=0; i<n; i++) {
    //Read the x,y and z output rates from the gyroscope and take an average of 50 results
    GxCal += gyro.getRotationX();
    GyCal += gyro.getRotationY();
    GzCal += gyro.getRotationZ();
    delay(10);
  }
  
  Serial.print("GxCal ");
  Serial.print(GxCal);
  Serial.print(" GyCal "); 
  Serial.print(GyCal);
  Serial.print(" GzCal ");
  Serial.println(GzCal);
  //use these to find gyro offsets
  GxOff=1.0*GxCal/n;
  GyOff=1.0*GyCal/n;
  GzOff=1.0*GzCal/n;
  Serial.print("Gxoff ");
  Serial.print(GxOff);
  Serial.print(" Gyoff "); 
  Serial.print(GyOff);
  Serial.print(" Gzoff ");
  Serial.println(GzOff);
  
  
  w[0] = (1.0*gyro.getRotationX()-GxOff)*Gsensitivity; 
  w[1] = (1.0*gyro.getRotationY()-GyOff)*Gsensitivity; 
  w[2] = (1.0*gyro.getRotationZ()-GzOff)*Gsensitivity;
  Serial.print(" w[0] ");
  Serial.print(gyro.getRotationX());
  Serial.print(" w[1] ");
  Serial.print(gyro.getRotationY());
  Serial.print(" w[2] ");
  Serial.println(gyro.getRotationZ());
  
  Serial.print(" w[0] ");
  Serial.print(w[0]);
  Serial.print(" w[1] ");
  Serial.print(w[1]);
  Serial.print(" w[2] ");
  Serial.println(w[2]);
  
  //accelerometer calibration
  acceleration[0] = accelerometer.getAccelerationX();
  acceleration[1] = accelerometer.getAccelerationY();
  acceleration[2] = accelerometer.getAccelerationZ();
  Serial.print(" ax ");
  Serial.print(acceleration[0]);
  Serial.print(" ay ");
  Serial.print(acceleration[1]);
  Serial.print(" az ");
  Serial.println(acceleration[2]);
  Ax_off = 0;
  Ay_off = 1;
  Az_off = -2;
  
  acceleration[0] = (1.0*accelerometer.getAccelerationX() - Ax_off);
  acceleration[1] = (1.0*accelerometer.getAccelerationY() - Ay_off);
  acceleration[2] = (1.0*accelerometer.getAccelerationZ() - Az_off);
  Mag_acc = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]); //calucate the magnitude
  Asensitivity=9.81/Mag_acc;
  
  Serial.print("Ax_off ");
  Serial.print(Ax_off);
  Serial.print(" Ay_off "); 
  Serial.print(Ay_off);
  Serial.print(" Az_off ");
  Serial.println(Az_off);
  Serial.print("100*Asensitivity ");
  Serial.println(100*Asensitivity);

  //print column headers
  //textFile.println(F("Time\tTime for loop in ms\tAcc x\tAcc y\tAcc z\tGx Rate\tGy Rate\tGzRate"));

  //if(textFile) digitalWrite(greenLedPin, HIGH );   // turn green LED on if file has been created successfully
  Serial.print("setup finished "); 
}

uint32_t loop_start;

void loop() {
  while(true) {
    loop_start = micros();
    //Accelerometer Read data from each axis, 2 registers per axis
    acceleration[0] = (accelerometer.getAccelerationX() - Ax_off)*Asensitivity;
    acceleration[1] = (accelerometer.getAccelerationY() - Ay_off)*Asensitivity ;
    acceleration[2] = (accelerometer.getAccelerationZ() - Az_off)*Asensitivity;
    Mag_acc = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]); //calucate the magnitude
    /*Serial.print(" ax ");
    Serial.print(acceleration[0]);
    Serial.print(" ay ");
    Serial.print(acceleration[1]);
    Serial.print(" az ");
    Serial.println(acceleration[2]);
    */
    
    //TODO set intial rotation matrix using acceleration data.
    if(hasLaunched != true){
      initialR[0][1]=-acceleration[0]/Mag_acc;
      initialR[1][1]=-acceleration[1]/Mag_acc;
      initialR[2][1]=-acceleration[2]/Mag_acc;
      //to find the 2 other axis take an arbitary vector that is roughly at right angles to the the y vector, and take the cross product
      Matrix.Copy((float*) initialR, 3, 3, (float*) initialR2);
      Matrix.Cross((float*)initialR, (float*)initialR, 3, 2, 1, (float*)initialR2, 3, 1);
      // then cross these to find the third axis
      Matrix.Cross((float*)initialR2, (float*)initialR2, 3, 1, 2, (float*)initialR2, 3, 3);
      
      Matrix.Copy((float*) initialR2, 3, 3, (float*) R1);
      launch_time = micros();
    }
    
    //If statement to detect launch
    PID(); //time to do some control
    
    serialcubeout((float*) R1, 9);
    
    //if statement to stop the loop after 30 minutes
    if(micros() > 1E9) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
       break;
    }
    
    uint32_t record_time = micros() - launch_time; //time spent recording data can be calculated
   // Serial.print("time : ");
    //Serial.println(record_time/1E6);

    //if statement to stop the loop after motor burn has ended and deploy chute
    if(record_time > 3E7) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
     delay(1000); //wait a second
     break;
   }
  uint32_t loop_end = micros();
  time_for_loop = (loop_end - loop_start);
  while(time_for_loop < 10E3) { //make the loop time 10ms
    loop_end = micros();
    time_for_loop = (loop_end - loop_start);
    delayMicroseconds(10);
  }
 }
 closedown();
}

float gx_total=0, gy_total=0, gz_total=0;
float gx_total2=0, gy_total2=0, gz_total2=0;
int nloops=0;

void PID(){
  hasLaunched = true;
  digitalWrite(redLedPin, HIGH ); //turn on red led if rocket has launched
  //Read the x,y and z output rates from the gyroscope.
  //angular velocity vector need to align/adjust gyro axis to rocket axis, clockwise rotations are positive
  w[0] = (1.0*gyro.getRotationX()-GxOff)*GsensitivityR; // gyro appears to use left hand coordinate system
  w[1] = (1.0*gyro.getRotationY()-GyOff)*GsensitivityR; 
  w[2] = (1.0*gyro.getRotationZ()-GzOff)*GsensitivityR;
  /*
  float Gx, Gy, Gz;
  Gx = (1.0*gyro.getRotationX()-GxOff)*Gsensitivity;
  Gy = (1.0*gyro.getRotationY()-GyOff)*Gsensitivity;
  Gz = (1.0*gyro.getRotationZ()-GzOff)*Gsensitivity;
  
  float time_for_loop_s=time_for_loop*1E-6;
  gx_total += Gx*time_for_loop_s; // gyro appears to use left hand coordinate system
  gy_total += Gy*time_for_loop_s; 
  gz_total += Gz*time_for_loop_s;
  
  gx_total2 += sq(Gx); // gyro appears to use left hand coordinate system
  gy_total2 += sq(Gy); 
  gz_total2 += sq(Gz);
  */
  nloops += 1;
  /*
  //gyro angle, found by intergration 
  // R1 is the previous rotation matrix which rotates the inertial 
    coordinate system to the body one
    R2 is the new matrix updated with the rotation rates from the body frame
    w(1) is rate of rotation about the rocket's x axis 
    w(2) is rate of rotation about the rocket's y axis
    w(3) is rate of rotation about the rocket's z axis
  
   */
  float time_for_loop_s=time_for_loop*1E-6;
  
  // cross product of w converted to matrix form multiplied by time for loop and + identity matrix
  float M[3][3] = {
    {1.0, -w[2]*time_for_loop_s, w[1]*time_for_loop_s},
    {w[2]*time_for_loop_s, 1.0, -w[0]*time_for_loop_s},
    {-w[1]*time_for_loop_s, w[0]*time_for_loop_s, 1.0}
  }; 
   
  Matrix.Multiply((float*)R1,(float*)M,3,3,3,(float*)R2);
  Matrix.Copy((float*) R2, 3, 3, (float*) R1);

  //Renormalization of R
  Matrix.NormalizeTay3x3((float*)R1); //remove errors so dot product doesn't go complex
  //note R1 is the matrix to transform a vector in the rocket coord sys to the world sys
  // the inverse (which is just the transpose) will transforms a vector in the world coord sys to the rocket coord sys
  //we could then use it to transform J from the world coord sys to the rocket coord sys 
  //however as this just picks out the middle column we can extract the 2 values straight from R1
 
  // subtract gravity vector from acceleration vector
  float g = -9.81;
  acceleration[0] -= g*R1[0][1];
  acceleration[1] -= g*R1[1][1];
  acceleration[2] -= g*R1[2][1];
 
 //integrate accelerations to get velocities
 velocity[0] += acceleration[0]*time_for_loop_s;
 velocity[1] += acceleration[1]*time_for_loop_s;
 velocity[2] += acceleration[2]*time_for_loop_s;
  
  //print data to file on SD card, using commas to seperate
 /* float data[] = {
    loop_start,
    time_for_loop,
    Ax,
    Ay,
    Az,
    w[0],
    w[1],
    w[2],
  };*/
  /*
  Serial.print(" ax ");
  Serial.print(acceleration[0]);
  Serial.print(" ay ");
  Serial.print(acceleration[1]);
  Serial.print(" az ");
  Serial.println(acceleration[2]);
  Serial.print(" Vx ");
  Serial.print(velocity[0]);
  Serial.print(" Vy ");
  Serial.print(velocity[1]);
  Serial.print(" Vz ");
  Serial.println(velocity[2]);
  /*
  Serial.print(" w[0] ");
  Serial.print(100*w[0]);
  Serial.print(" w[1] ");
  Serial.print(100*w[1]);
  Serial.print(" w[2] ");
  Serial.println(100*w[2]);
  /*
  Serial.print(" gyroX ");
  Serial.print(gyro.getRotationX());
  Serial.print(" gyroY ");
  Serial.print(gyro.getRotationY());
  Serial.print(" gyroZ ");
  Serial.println(gyro.getRotationZ());
  Matrix.Print((float*)R1, 3, 3, "R1");
  /*
  Serial.print(" gx total ");
  Serial.print(gx_total);
  Serial.print(" gy total ");
  Serial.print(gy_total);
  Serial.print(" gz total ");
  Serial.println(gz_total);
  */
  
  
  //dataFile.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));   //write data to sd card
}

void closedown() {
  dataFile.close();
  textFile.close(); //close and save SD file, otherwise precious data will be lost
  digitalWrite(greenLedPin, LOW);    // turn LED off
  digitalWrite(redLedPin, LOW);    // turn LED off
  /*
  Serial.print(" gx total ");
  Serial.print(gx_total);
  Serial.print(" gy total ");
  Serial.print(gy_total);
  Serial.print(" gz total ");
  Serial.println(gz_total);
  Serial.print(" gx total 2 ");
  Serial.print(gx_total2);
  Serial.print(" gy total 2 ");
  Serial.print(gy_total2);
  Serial.print(" gz total 2 ");
  Serial.println(gz_total2);*/
  Serial.print(" n ");
  Serial.println(nloops);  
  
  delay(100000000); //is there a way to break out of the loop
}
  
void serialcubeout(float* M, int length){
  char cmd;
  cmd = Serial.read();
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  if(cmd == 'q'){
    for(int i=0; i<length; i++){
      serialFloatPrint(M[i]);
      Serial.print(",");
    }
    Serial.println();
  }
}
  
   
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}

char serial_busy_wait() {
  while(!Serial.available()) {
    ; // do nothing until ready
  }
  return Serial.read();
}
