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

uint32_t time_for_loop = 0; //time for a loop
uint32_t launch_time;

float Gsensitivity = 1 / 14.375; // deg.s^-1 / LSB
float Asensitivity = 1 / 3.262;  // m.s^-2 / LSB: 26.093 for 2g/full resolution, 13.048g for 4g, 6.524 for 8g, 3.262 for 16g, these are typical value and may not be accurate
float GxOff, GyOff, GzOff; //gyro offset values
float Accx, Accy, Accz, Mag_acc; //variables for acceleration in m/s^2
float accel_angle_x, accel_angle_y; //tilt angles from accelerometer
float accel_center_x = 0, accel_center_y = 0, accel_center_z = 0; //alternative offsets for accelerometer which are used in the software, these seem better

//inorder to calculate alltitude need to know
//const float p0 = 100400;     // Pressure at sea level (Pa) standard 101325
//float altitude, previous_altitude = 0;

//creat servo variables
Servo servo_1;  // create servo object to control a servo
Servo servo_2;  // create servo object to control a servo
Servo servo_3;  // create servo object to control a servo
                // a maximum of eight servo objects can be created

int pos1 = 95, pos2 = 90, pos3 = 90, s1, s2, s3, smax = 150, smin = 30, k = 0;    // variable to store the servo position
float r = 0, theta = 0, s_a, s_b, s_c, temp1, temp2, temp3;
float d = 20, D = 48, horn = 12, link = 28; //geometric values
//const float pi = 3.14;

// PID constants
float Kp = 50, Ki = 0, Kd = 0;

//direction variables
float w[3]; //angular velocity vector
float eye[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} }; //indentity matrix
float R1[3][3]; //rotation matrix 1
float R2[3][3]; //rotation matrix 2
//int I[3][1] = { {1}, {0}, {0} }; //unit vector inertial system
float J[3] = {0, 1, 0}; //unit vector inertial system
//int K[3][1] = { {0}, {0}, {1} }; //unit vector inertial system
float Pitch = 0, Yaw = 0; // used to calculate gimbal movement
float Heading; // angle between rocket vertical and inertial vertical
float Integral = 0;
float Der = 0;
float r_max = 9.5; //to be determined
float previous_Heading = 0;
float previous_r = 0;

void sendDataBack() {
  dataFile.seek(0);
  while (dataFile.available()) {
    Serial.write(dataFile.read());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Gsensitivity = Gsensitivity * M_PI / 180; //convert degrees per second to radians per second

  //Serial.begin(115200); //Create a serial connection using a 115200bps baud rate.

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

    if(dataFile) digitalWrite(redLedPin, HIGH );   // turn LED on if file has been created successfully

  Wire.begin(); //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.


  if(accelerometer.testConnection()) textFile.println("Accelerometer connected!");
  if(compass.testConnection()) textFile.println("Compass connected!");
  if(gyro.testConnection()) textFile.println("Gyro connected!");
  if(barometer.testConnection()) textFile.println("Barometer connected!");
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

  //print column headers
  textFile.println("Time\tTime for loop in ms\tAcc x\tAcc y\tAcc z\tGx Rate\tGy Rate\tGzRate\tMx\tMy\tMz\tTemp\tPressure");

  if(textFile) digitalWrite(greenLedPin, HIGH );   // turn LED on if file has been created successfully

  launch_time = micros();
}

bool hasLaunched = false;
uint32_t barometer_ready_time = 0;
bool barometer_is_temperature = false;


void loop() {
  uint32_t loop_start = micros();


  //Read the x,y and z output rates from the gyroscope.
  //angular velocity vector need to align/adjust gyro axis to rocket axis, clockwise rotations are positive
  w[0] = -1.0*gyro.getRotationX();
  w[1] = 1.0*gyro.getRotationY(); // gyro appears to use left hand coordinate system
  w[2] = 1.0*gyro.getRotationZ();

  //Accelerometer Read data from each axis, 2 registers per axis
  int Ax = accelerometer.getAccelerationX(); 
  int Ay = - accelerometer.getAccelerationY(); //make upwards positive
  int Az = accelerometer.getAccelerationZ();

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

  uint32_t record_time = loop_start - launch_time; //time spent recording data can be calculated

  //if statement to stop the loop after 60 minutes or after 60 seconds of recording
  if(record_time > 300E6) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
    dataFile.close();
    textFile.close(); //close and save SD file, otherwise precious data will be lost
    servo_1.write(pos1);              // tell servo to go to position in variable 'pos'
    servo_2.write(pos2);              // tell servo to go to position in variable 'pos'
    servo_3.write(pos3);              // tell servo to go to position in variable 'pos'
    digitalWrite(greenLedPin, LOW);    // turn LED off
    digitalWrite(redLedPin, LOW);    // turn LED off

    delay(100000000); //is there a way to break out of the loop
  }

  uint32_t loop_end = micros();
  time_for_loop = (loop_end - loop_start);
  while(time_for_loop < 10E3) { //make the loop time 10ms
  loop_end = micros();
  time_for_loop = (loop_end - loop_start);
  delayMicroseconds(10);
  }

  loop_end = micros();
  time_for_loop = (loop_end - loop_start);
}
