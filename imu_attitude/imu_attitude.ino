
//Libraries
#include <Wire.h> //The Wire library is used for I2C communication
#include <math.h> //Maths Library
#include <SD.h> //SD library
#include <MatrixMath.h> //matrix library

//I2C devices from https://github.com/jrowberg/i2cdevlib
#include <I2Cdev.h>
#include <ITG3200.h>
#include <ADXL345.h>
#include <HMC5883L.h>

ADXL345 accelerometer;
ITG3200 gyro;
HMC5883L magnetometer;

//Global variables
File textFile; //holds information on file ebing written to on SD card
File dataFile;

const int greenLedPin = 8;  // green LED is connected to pin 8
const int redLedPin = 7;  // red LED is connected to pin 8

uint32_t time_for_loop; //time for a loop
uint32_t launch_time;

float Gsensitivity = 1 / 14.375; // deg.s^-1 / LSB
float GsensitivityR; //sensitivity in radians
float Asensitivity = 1 / 3.262;  // m.s^-2 / LSB: 26.093 for 2g/full resolution, 13.048g for 4g, 6.524 for 8g, 3.262 for 16g, these are typical value and may not be accurate
float GxOff, GyOff, GzOff; //gyro offset values
int Ax, Ay, Az;
float Accx, Accy, Accz, Mag_acc, Acc_error; //variables for acceleration in m/s^2
int Ax_off = 0.506, Ay_off = -0.995, Az_off = -1.011;
float Axscale = 0.9998, Ayscale = 0.9850, Azscale = 1.0156;

//direction variables
float w[3]; //angular velocity vector
float initialR[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} }; //indentity matrix
float initialR2[3][3]; 
float R1[3][3]; //rotation matrix 1
float R2[3][3]; //rotation matrix 2
float gref[3]; //gravity vector
float w_Icorrection[3]={0,0,0};
int16_t Mx, My, Mz;
float Mxoff = 1.5, Myoff = -108.5, Mzoff = -34.5;
float Mxscale = 0.9554, Myscale = 0.9492, Mzscale = 1.1114;
float mref[3];
float Mag_mag;

//lateral variables
float acceleration[3]= {0,0,0}; // (x,y,z) vector

bool hasLaunched = false;

//prototypes
void GetData();
void updateR();
void drift_correction();
void closedown();
void serialcubeout(float* M, int length);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

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
  if(magnetometer.testConnection()) Serial.println(F("Magnetometer connected!"));
  
  //configure the gyro
  gyro.setFullScaleRange(ITG3200_FULLSCALE_2000); //only ITG3200_FULLSCALE_2000 is documented
  gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);
  gyro.setRate(9); // = 100Hz - I think the time for a sample should match the loop time

  // configure accelerometer
  accelerometer.initialize();
  accelerometer.setRange(ADXL345_RANGE_16G);
  accelerometer.setRate(ADXL345_RATE_100); //Hz
  accelerometer.setMeasureEnabled(true);
  accelerometer.setOffset(0, 0, 0);  
  
  //configure magnetometer
  magnetometer.initialize();
  magnetometer.setSampleAveraging(HMC5883L_AVERAGING_1);
  magnetometer.setDataRate(HMC5883L_RATE_75); //75Hz
  magnetometer.setMeasurementBias(HMC5883L_BIAS_NORMAL); 
  magnetometer.setGain(HMC5883L_GAIN_1090);  
  //magnetometer.setMode(HMC5883L_MODE_CONTINUOUS); // can't get this to work
  //Serial.println(magnetometer.getMode());
  
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
  /*
  acceleration[0] = (1.0*accelerometer.getAccelerationX() - Ax_off);
  acceleration[1] = (1.0*accelerometer.getAccelerationY() - Ay_off);
  acceleration[2] = (1.0*accelerometer.getAccelerationZ() - Az_off);
  Mag_acc = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]); //calucate the magnitude
  Asensitivity=9.81/Mag_acc;
  */
  
  Serial.print("Ax_off ");
  Serial.print(Ax_off);
  Serial.print(" Ay_off "); 
  Serial.print(Ay_off);
  Serial.print(" Az_off ");
  Serial.println(Az_off);
  Serial.print("100*Asensitivity ");
  Serial.println(100*Asensitivity);
  
  //need to find the gravity vector with the R matrix zeroed
  acceleration[0] = (accelerometer.getAccelerationX() - Ax_off)*Asensitivity*Axscale; // flip accelerometer axis so that they match gyro's
  acceleration[1] = (accelerometer.getAccelerationY() - Ay_off)*Asensitivity*Ayscale;
  acceleration[2] = (accelerometer.getAccelerationZ() - Az_off)*Asensitivity*Azscale;
  Mag_acc = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]); //calucate the magnitude
  gref[0] = acceleration[0]/Mag_acc;
  gref[1] = acceleration[1]/Mag_acc;
  gref[2] = acceleration[2]/Mag_acc;
  
  //need to find the magnetic vector with the R matrix zeroed
  magnetometer.getHeading(&My, &Mx, &Mz); //note the axes don't match the other sensors
  Mx -= Mxoff;
  My -= Myoff;
  Mz -= Mzoff;
  Mx *= Mxscale;
  My *= Myscale;
  Mz *= Mzscale;
  Mag_mag = sqrt(1.0*Mx*Mx + 1.0*My*My + 1.0*Mz*Mz); //calucate the magnitude
  mref[0] = 1.0*Mx/Mag_mag;
  mref[1] = -1.0*My/Mag_mag;
  mref[2] = 1.0*Mz/Mag_mag;
  
  //print column headers
  //textFile.println(F("Time\tTime for loop in ms\tAcc x\tAcc y\tAcc z\tGx Rate\tGy Rate\tGzRate"));

  //if(textFile) digitalWrite(greenLedPin, HIGH );   // turn green LED on if file has been created successfully
  Serial.print("setup finished "); 
}

uint32_t loop_start;
int n=0;

void loop() {
  launch_time = micros();
  while(true) { 
    loop_start = micros();
    
    GetData(); 
    
    drift_correction();
    
    updateR(); 
    
    serialcubeout((float*) R1, 9);
    
    uint32_t record_time = micros() - launch_time; //time spent recording data 
    //Serial.print("time : ");
    //Serial.println(record_time/1E6);
  
    //if statement to stop the loop after a set time
    if(record_time > 1E9) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
     break;
   }
   uint32_t loop_end = micros();
   time_for_loop = (loop_end - loop_start);
   
   while(time_for_loop < 10E3) { //make the loop time 10ms
     loop_end = micros();
     time_for_loop = (loop_end - loop_start);
     delayMicroseconds(10);
   }
   //Serial.println(time_for_loop);
 }
 closedown();
}

void GetData(){
  //get sensor data
  //Read the x,y and z output rates from the gyroscope.
  w[0] = (1.0*gyro.getRotationX()-GxOff)*GsensitivityR; 
  w[1] = (1.0*gyro.getRotationY()-GyOff)*GsensitivityR; 
  w[2] = (1.0*gyro.getRotationZ()-GzOff)*GsensitivityR;
  //Accelerometer Read data from each axis, 2 registers per axis
  acceleration[0] = (accelerometer.getAccelerationX() - Ax_off)*Asensitivity*Axscale;
  acceleration[1] = (accelerometer.getAccelerationY() - Ay_off)*Asensitivity*Ayscale;
  acceleration[2] = (accelerometer.getAccelerationZ() - Az_off)*Asensitivity*Azscale;
  Mag_acc = sqrt(acceleration[0]*acceleration[0]+acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]); //calucate the magnitude
  //read magnetometer
  magnetometer.getHeading(&My, &Mx, &Mz);
  Mx -= Mxoff;
  My -= Myoff;
  Mz -= Mzoff;
  Mx *= Mxscale;
  My *= Myscale;
  Mz *= Mzscale;
  Mag_mag = sqrt(1.0*Mx*Mx + 1.0*My*My + 1.0*Mz*Mz); //calucate the magnitude

   /* Serial.print("Mx ");
    Serial.print(Mx);
    Serial.print(" My "); 
    Serial.print(My);
    Serial.print(" Mz ");
    Serial.println(Mz);/*
    if( Serial.read() == ' ') {
      Serial.print("Ax ");
      Serial.print(acceleration[0]);
      Serial.print(" Ay "); 
      Serial.print(acceleration[1]);
      Serial.print(" Az ");
      Serial.println(acceleration[2]);
    } */
}

void updateR(){
  float time_for_loop_s=time_for_loop*1E-6;
  
  // update matrix
  float M[3][3] = {
    {1.0, w[2]*time_for_loop_s, -w[1]*time_for_loop_s},
    {-w[2]*time_for_loop_s, 1.0, w[0]*time_for_loop_s},
    {w[1]*time_for_loop_s, -w[0]*time_for_loop_s, 1.0}
  }; 
   
  Matrix.Multiply((float*)R1,(float*)M,3,3,3,(float*)R2); //R2 = R1 * M
  Matrix.Copy((float*) R2, 3, 3, (float*) R1); // R1 = R2
  //Renormalization of R
  Matrix.NormalizeTay3x3((float*)R1); //remove errors so dot product doesn't go complex
}

void drift_correction(){
  //calculate direction of g using R
  float gest[3]; 
  Matrix.Multiply((float*)R1,(float*)gref,3,3,1,(float*)gest); //gest = R1 * gref
  float correction_acc[3], gmeas[3];
  gmeas[0] = acceleration[0]/Mag_acc;
  gmeas[1] = acceleration[1]/Mag_acc;
  gmeas[2] = acceleration[2]/Mag_acc;
  Matrix.Cross((float*) gest, (float*) gmeas, 1, 1, 1, (float*) correction_acc, 1, 1);
  
  //same approach but with the gravity vector
  float mest[3]; 
  Matrix.Multiply((float*)R1,(float*)mref,3,3,1,(float*)mest); //gest = R1 * gref
  float correction_mag[3], mmeas[3];
  mmeas[0] = 1.0*Mx/Mag_mag;
  mmeas[1] = -1.0*My/Mag_mag;
  mmeas[2] = 1.0*Mz/Mag_mag;
  Matrix.Cross((float*) mest, (float*) mmeas, 1, 1, 1, (float*) correction_mag, 1, 1);
  
  n++;
  if (n>20){
    Serial.print("mestx ");
    Serial.print(mest[0]);
    Serial.print(" mesty "); 
    Serial.print(mest[1]);
    Serial.print(" mestz ");
    Serial.println(mest[2]);
    Serial.print("mmeasx ");
    Serial.print(mmeas[0]);
    Serial.print(" mmeasy "); 
    Serial.print(mmeas[1]);
    Serial.print(" mmeasz ");
    Serial.println(mmeas[2]);
    
    Serial.print("gestx ");
    Serial.print(gest[0]);
    Serial.print(" gesty "); 
    Serial.print(gest[1]);
    Serial.print(" gestz ");
    Serial.println(gest[2]);
    Serial.print("gmeasx ");
    Serial.print(gmeas[0]);
    Serial.print(" gmeasy "); 
    Serial.print(gmeas[1]);
    Serial.print(" gmeasz ");
    Serial.println(gmeas[2]);
    n=0;
  }
  
  //combine corrections from accelerometer and magnetometer
  float total_correction[3];
  total_correction[0] = correction_acc[0] + correction_mag[0];
  total_correction[1] = correction_acc[1] + correction_mag[1];
  total_correction[2] = correction_acc[2] + correction_mag[2];
  
  float w_correction[3];
  float wKp= 0.1, wKi = 0;
  float time_for_loop_s=time_for_loop*1E-6;
  w_Icorrection[0] += wKi * total_correction[0] * time_for_loop_s;
  w_Icorrection[1] += wKi * total_correction[1] * time_for_loop_s;
  w_Icorrection[2] += wKi * total_correction[2] * time_for_loop_s;
  w_correction[0] = wKp*total_correction[0] + w_Icorrection[0]; 
  w_correction[1] = wKp*total_correction[1] + w_Icorrection[1]; 
  w_correction[2] = wKp*total_correction[2] + w_Icorrection[2]; 
  
  //combine correction term with reading
  w[0] = w[0] + w_correction[0];
  w[1] = w[1] + w_correction[1];
  w[2] = w[2] + w_correction[2];
}

void closedown() {
  dataFile.close();
  textFile.close(); //close and save SD file, otherwise precious data will be lost
  digitalWrite(greenLedPin, LOW);    // turn LED off
  digitalWrite(redLedPin, LOW);    // turn LED off
  delay(100000000); //is there a way to break out of the loop
}
    
void serialcubeout(float* M, int length){
  //function to send data to cube visualization
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
    Serial.println("");
  }
}
  
   
void serialFloatPrint(float f) {
  //function to print floats in hex, used by serialcubeout
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
