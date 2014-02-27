//Libraries
#include <Wire.h> //The Wire library is used for I2C communication
#include <math.h> //Maths Library
#include <SD.h> //SD library
#include <Servo.h> //servo library
#include <MatrixMath.h> //matrix library

//I2C devices from https://github.com/jrowberg/i2cdevlib
#include <I2Cdev.h>
#include <BMP085.h>
#include <HMC5883L.h>
#include <ITG3200.h>
#include <ADXL345.h>

BMP085 barometer;
HMC5883L compass;
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
const float Asensitivity = 1 / 3.262;  // m.s^-2 / LSB: 26.093 for 2g/full resolution, 13.048g for 4g, 6.524 for 8g, 3.262 for 16g, these are typical value and may not be accurate
float GxOff, GyOff, GzOff; //gyro offset values
int Ax, Ay, Az;
float Accx, Accy, Accz, Mag_acc; //variables for acceleration in m/s^2
float accel_angle_x, accel_angle_y; //tilt angles from accelerometer
float accel_center_x = 0, accel_center_y = 0, accel_center_z = 0; //alternative offsets for accelerometer which are used in the software, these seem better

//inorder to calculate alltitude need to know
//const float p0 = 100400;     // Pressure at sea level (Pa) standard 101325
//float altitude, previous_altitude = 0;

//create servo variables
Servo servo_1;  // create servo object to control a servo
Servo servo_2;  // create servo object to control a servo
Servo servo_3;  // create servo object to control a servo
                // a maximum of eight servo objects can be created

//check servo centering
int pos1 = 90, pos2 = 90, pos3 = 90, s1, s2, s3, k = 0;    // variable to store the servo position
const int smax = 150, smin = 30;
float rP = 0, rY = 0, s_a, s_b, s_c;

// PID constants
const float Kp = 25, Ki = 2, Kd = 8;

//direction variables
float w[3]; //angular velocity vector
const float eye[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} }; //indentity matrix
float R1[3][3]; //rotation matrix 1
float R2[3][3]; //rotation matrix 2
//const float J[3] = {0, 1, 0}; //unit vector inertial system
float Pitch = 0, Yaw = 0; // used to calculate gimbal movement
float IntegralP = 0, IntegralY=0;
float DerP = 0, DerY=0;
const float r_max = 9.5; //to be determined
float previous_Pitch = 0, previous_Yaw=0;
float previous_rP = 0, previous_rY=0, previous_r=0;

void sendDataBack() {
  dataFile.seek(0);
  while (dataFile.available()) {
    Serial.write(dataFile.read());
  }
}

bool hasLaunched = false;
uint32_t barometer_ready_time = 0;
bool barometer_is_temperature = false;

//prototypes
void PID();
void closedown();
int freeRam();

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Gsensitivity = Gsensitivity * M_PI / 180; //convert degrees per second to radians per second

  pinMode(10, OUTPUT); //set SD CS pin to output
  pinMode(greenLedPin, OUTPUT); // Set the LED 1 pin as output
  pinMode(redLedPin, OUTPUT); // Set the LED 2 pin as output

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

  Wire.begin(); //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.

  if(accelerometer.testConnection()) textFile.println(F("Accelerometer connected!"));
  if(compass.testConnection()) textFile.println(F("Compass connected!"));
  if(gyro.testConnection()) textFile.println(F("Gyro connected!"));
  if(barometer.testConnection()) textFile.println(F("Barometer connected!"));
  textFile.print("data file is ");
  textFile.print(filename);

  // TODO: compass.initialize();
  compass.setMode(HMC5883L_MODE_CONTINUOUS);

  barometer.initialize(); //calibrate Barometer

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
  
     //Gyro Calibration
  int GxCal=0, GyCal=0, GzCal=0; 
  
  for (uint8_t i=0; i<50; i++) {
    //Read the x,y and z output rates from the gyroscope and take an average of 50 results
    GxCal = GxCal + gyro.getRotationX();
    GyCal = GyCal + gyro.getRotationY();
    GzCal = GzCal + gyro.getRotationZ();
    delay(10);
  }
  //use these to find gyro offsets
  GxOff=GxCal/50;
  GyOff=GyCal/50;
  GzOff=GzCal/50;
  
  //attach servos 
  servo_1.attach(2);  // attaches the servo on pin 2 to the servo object 1
  servo_2.attach(3);  // attaches the servo on pin 3 to the servo object 2
  servo_3.attach(4);  // attaches the servo on pin 4 to the servo object 3
    
  servo_1.write(pos1);              // tell servo to go to position in variable 'pos1' 
  servo_2.write(pos2);              // tell servo to go to position in variable 'pos2' 
  servo_3.write(pos3);              // tell servo to go to position in variable 'pos3'
  
  textFile.print(F("free ram = "));
  textFile.println(freeRam());

  //print column headers
  textFile.println(F("Time\tTime for loop in ms\tAcc x\tAcc y\tAcc z\tGx Rate\tGy Rate\tGzRate\tMx\tMy\tMz\tTemp\tPressure"));

  if(textFile) digitalWrite(greenLedPin, HIGH );   // turn green LED on if file has been created successfully

}

uint32_t loop_start;

void loop() {
  while(true) {
    loop_start = micros();
    //Accelerometer Read data from each axis, 2 registers per axis
    Ax = accelerometer.getAccelerationX(); 
    Ay = - accelerometer.getAccelerationY(); //make upwards positive
    Az = accelerometer.getAccelerationZ();
    Mag_acc=sqrt(Ax*Ax+Ay*Ay+Az*Az)*Asensitivity; //calucate the magnitude
    
    //If statement to detect launch
    if(Mag_acc>15 or hasLaunched==true) {
    PID(); //time to do some control
    }
    else launch_time = micros();
    
    //if statement to stop the loop after 30 minutes
    if(micros() > 1E9) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
       break;
    }
    
    uint32_t record_time = micros() - launch_time; //time spent recording data can be calculated

    //if statement to stop the loop after motor burn has ended and deploy chute
    if(record_time > 10E6) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
     delay(1000); //wait a second
     digitalWrite(recoveryPin, HIGH); //deploy chute
     delay(3000); //wait 3 seconds
     digitalWrite(recoveryPin, LOW); //turn off mosfet
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

void PID(){
  hasLaunched = true;
  digitalWrite(redLedPin, HIGH ); //turn on red led if rocket has launched
  //Read the x,y and z output rates from the gyroscope.
  //angular velocity vector need to align/adjust gyro axis to rocket axis, clockwise rotations are positive
  w[0] = (-1.0*gyro.getRotationX()-GxOff)/Gsensitivity; // gyro appears to use left hand coordinate system
  w[1] = (1.0*gyro.getRotationY()-GyOff)/Gsensitivity; 
  w[2] = (1.0*gyro.getRotationZ()-GzOff)/Gsensitivity;
  
  //gyro angle, found by intergration 
  /* R1 is the previous rotation matrix which rotates the inertial 
    coordinate system to the body one
    R2 is the new matrix updated with the rotation rates from the body frame
    w(1) is rate of rotation about the rocket's x axis 
    w(2) is rate of rotation about the rocket's y axis
    w(3) is rate of rotation about the rocket's z axis
  */
   
   float time_for_loop_s=time_for_loop*1E-6;
  // cross product of w converted to matrix form multiplied by time for loop and + identity matrix
  float M[3][3] = {
    {1,-w[2]*time_for_loop_s,w[1]*time_for_loop_s},
    {w[2]*time_for_loop_s,1,-w[0]*time_for_loop_s},
    {-w[1]*time_for_loop_s,w[0]*time_for_loop_s,1}
  }; 
   
  Matrix.Multiply((float*)R1,(float*)M,3,3,3,(float*)R2);
  Matrix.Copy((float*) R2, 3, 3, (float*) R1);

  //Renormalization of R
  Matrix.Normalize3x3((float*)R1); //remove errors so dot product doesn't go complex
  //note R1 is the matrix to transform a vector in the rocket coord sys to the world sys
  // the inverse (which is just the transpose) will transforms a vector in the world coord sys to the rocket coord sys
  //we could then use it to transform J from the world coord sys to the rocket coord sys 
  //however as this just picks out the middle column we can extract the 2 values straight from R1
  Pitch = R1[1][2]; //component in z direction
  Yaw = R1[1][0]; //component in x direction
  
  //feed these into two PIDs
  if (previous_r < r_max) IntegralP = IntegralP + Pitch * time_for_loop_s; //stop integral causing windup      
  DerP = (Pitch - previous_Pitch)/ time_for_loop_s; // might want a low pass filter on here
  previous_Pitch = Pitch;
  rP = Kp * Pitch + Ki * IntegralP + Kd * DerP;
  previous_rP = rP;
    
  if (previous_r < r_max) IntegralY = IntegralY + Yaw * time_for_loop_s; //stop integral causing windup      
  DerY = (Yaw - previous_Yaw)/ time_for_loop_s; // might want a low pass filter on here
  previous_Yaw = Yaw;
  rY = Kp * Yaw + Ki * IntegralY + Kd * DerY;
  previous_rY = rY;
  
  float CoC[2]={ //vector from centre of rocket to desired motor centre
    rY,
    rP
  };
  
  float r_mag =sqrt(rY*rY +rP*rP);
  if (r_mag > r_max) {
    //scale CoC
    float scale= r_max/r_mag;
    CoC[0]=CoC[0]*scale;
    CoC[1]=CoC[1]*scale;
  }
  
  const int d = 20, D = 48, horn = 12, link = 28; //geometric values
  
  float s_A[2]={
    D - CoC[0],
    -(CoC[1] + d)
  };
  float s_B[2]={
    0.866*D - (CoC[0] + 0.866*d),
    -0.5*D - (CoC[1] - 0.5*d)
  };
  float s_C[2]={
    -0.866*D - (CoC[0] - 0.866*d),
    -0.5*D - (CoC[1] - 0.5*d)
  };
  
  
  s_a = sqrt(s_A[0]*s_A[0] + s_A[1]*s_A[1]) - link;
  s_b = sqrt(s_B[0]*s_B[0] + s_B[1]*s_B[1]) - link;
  s_c = sqrt(s_C[0]*s_C[0] + s_C[1]*s_C[1]) - link;
  
  // convert into servo degrees 180/M_PI = 57
  s1 = 57*asin(s_a/horn) + pos1;
  s2 = 57*asin(s_b/horn) + pos2;
  s3 = 57*asin(s_c/horn) + pos3;

  
  //update servos with new positions
  servo_1.write(s1);              
  servo_2.write(s2);               
  servo_3.write(s3);   
  
  //read magnetometer
  int16_t Mx, My, Mz;
  compass.getHeading(&Mx, &My, &Mz);

  float temperature = NAN;
  float pressure = NAN;
  if(static_cast<int32_t>(micros() - barometer_ready_time) > 0) {
    if(barometer_is_temperature) {
      temperature = barometer.getTemperatureC();
      barometer.setControl(BMP085_MODE_PRESSURE_0);
      barometer_is_temperature = false;
    }
    else {
      pressure = barometer.getPressure();
      barometer.setControl(BMP085_MODE_TEMPERATURE);
      barometer_is_temperature = true;
    }
    barometer_ready_time = micros() + barometer.getMeasureDelayMicroseconds();
  }
  
  //print data to file on SD card, using commas to seperate
  float data[] = {
    loop_start,
    time_for_loop,
    Ax,
    Ay,
    Az,
    w[0],
    w[1],
    w[2],
    Mx,
    My,
    Mz,
    temperature,
    pressure
  };

  dataFile.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));   
}

void closedown() {
  dataFile.close();
  textFile.close(); //close and save SD file, otherwise precious data will be lost
  servo_1.write(pos1);              // tell servo to go to position in variable 'pos'
  servo_2.write(pos2);              // tell servo to go to position in variable 'pos'
  servo_3.write(pos3);              // tell servo to go to position in variable 'pos'
  digitalWrite(greenLedPin, LOW);    // turn LED off
  digitalWrite(redLedPin, LOW);    // turn LED off
  delay(100000000); //is there a way to break out of the loop
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
  
