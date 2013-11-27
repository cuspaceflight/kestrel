//Libraries
#include <Wire.h> //The Wire library is used for I2C communication
#include <math.h> //Maths Library
#include <SD.h> //SD library
#include <Servo.h> //servo library
#include "../libraries/MatrixMath/MatrixMath.h" //matrix library

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

unsigned long time = 0, start_time = 0, record_time = 0; //long variables for dealing with time

float time_for_loop = 0; //time for a loop
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

  textFile = SD.open("Log_All.txt", FILE_WRITE); //create file on SD card
  dataFile = SD.open("log.dat", FILE_WRITE);

  Wire.begin(); //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.


  if(accelerometer.testConnection()) textFile.println("Accelerometer connected!");
  if(compass.testConnection()) textFile.println("Compass connected!");
  if(gyro.testConnection()) textFile.println("Gyro connected!");
  if(barometer.testConnection()) textFile.println("Barometer connected!");

  // TODO: compass.initialize();
  compass.setMode(HMC5883L_MODE_CONTINUOUS);

  barometer.initialize(); //calibrate Barometer

  //configure tehe gyro
  gyro.setFullScaleRange(2); // unofficially 1000°/sec, only ITG3200_FULLSCALE_2000 is documented
  gyro.setDLPFBandwidth(ITG3200_DLPF_BW_98);
  gyro.setRate(9); // = 100Hz - I think the time for a sample should match the loop time

  // configure accelerometer
  accelerometer.initialize();
  accelerometer.setRange(ADXL345_RANGE_16G);
  accelerometer.setRate(ADXL345_RATE_100); //Hz
  accelerometer.setMeasureEnabled(true);
  accelerometer.setOffset(5, 5, 5);  //for some strange reason setting these to 0 gave more false readings

  delay(1000); //make sure everything is static

  //Accelerometer Calibration
  {
    float AxCal = 0, AyCal = 0, AzCal = 0;
    //find average values at rest
    for (int i = 0; i<25; i++) {
      //Read the x,y and z output rates from the accelerometer.
      AxCal = AxCal + accelerometer.getAccelerationX();
      AyCal = AyCal - accelerometer.getAccelerationY(); //make upwards positive
      AzCal = AzCal + accelerometer.getAccelerationZ();
      delay(10);
    }

    //Alternatvie accelerometer calibration
    accel_center_x = (AxCal/25)*Asensitivity; //in g then in mg
    accel_center_y = (AyCal/25)*Asensitivity + 9.81; //account for gravity
    accel_center_z = (AzCal/25)*Asensitivity;
  }

    //Gyro Calibration
  int GxCal = 0, GyCal = 0, GzCal = 0;

  for (int i = 0; i<50; i++) {
    //Read the x,y and z output rates from the gyroscope and take an average of 50 results
    GxCal = GxCal + gyro.getRotationX();
    GyCal = GyCal + gyro.getRotationY();
    GzCal = GzCal + gyro.getRotationZ();
    delay(10);
  }
  //use these to find gyro offsets
  GxOff = GxCal/50;
  GyOff = GyCal/50;
  GzOff = GzCal/50;

  //print column headers
  textFile.println("Time\tTime for loop in ms\tAcc x\tAcc y\tAcc z\tGx Rate\tGy Rate\tGzRate\tMx\tMy\tMz\tTemp\tPressure");

  if(textFile) digitalWrite(greenLedPin, HIGH );   // turn LED on if file has been created successfully

}

bool hasLaunched = false;

void loop() {
  // Wait for a request for data
  if(Serial.available()) {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    sendDataBack();
    digitalWrite(redLedPin, LOW);
    while(1);
  }

  time = micros(); //time at start of loop, in micro seconds

  //Read the x,y and z output rates from the gyroscope.
  //angular velocity vector need to align/adjust gyro axis to rocket axis, clockwise rotations are positive
  w[0] = -(1.000*gyro.getRotationX() - GxOff)*Gsensitivity;
  w[1] = (1.000*gyro.getRotationY() - GyOff)*Gsensitivity; // gyro appears to use left hand coordinate system
  w[2] = (1.000*gyro.getRotationZ() - GzOff)*Gsensitivity;


  //Accelerometer Read data from each axis, 2 registers per axis
  int Ax = accelerometer.getAccelerationX();
  Accx = Ax*Asensitivity -accel_center_x; //convert to SI units and zero
  int Ay = - accelerometer.getAccelerationY(); //make upwards positive
  Accy = Ay*Asensitivity-accel_center_y;
  int Az = accelerometer.getAccelerationZ();
  Accz = Az*Asensitivity-accel_center_z;
  Mag_acc = sqrt(Accx*Accx+Accy*Accy+Accz*Accz); //calucate the magnitude

  if(Mag_acc<20 && !hasLaunched) {
    //if the rocket hasn't experienced an accleration over 30 m/s^2, it hasn't yet lacunhed
    // Serial.println("Low acceleration mode");
  }
  else {
    digitalWrite(redLedPin, HIGH ); //turn red led on
    //Serial.println("High acceleration mode");

    //read magnetometer
    int16_t Mx, My, Mz;
    compass.getHeading(&Mx, &My, &Mz);

    //read pressure sensor
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    delayMicroseconds(barometer.getMeasureDelayMicroseconds());
    float temperature = barometer.getTemperatureC(); //read temperature from the barometer which has a temp sesnors, and convert to degrees*10
    barometer.setControl(BMP085_MODE_PRESSURE_0);
    delayMicroseconds(barometer.getMeasureDelayMicroseconds());
    float pressure = barometer.getPressure(); // read pressure from barometer, and convert to Pa

    //print data to file on SD card, using commas to seperate
    float data[] = {
      time*0.000001,
      time_for_loop*1000,
      Accx,
      Accy,
      Accz,
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

    if(!hasLaunched) {
      // first high-acceleration pass - we just launched
      hasLaunched = 1;
      start_time = micros();
    }
  }

  record_time = micros() - start_time; //time spent recording data can be calculated

  //if statement to stop the loop after 60 minutes or after 60 seconds of recording
  if(micros() > 2E9 || (record_time > 30E6 && hasLaunched)) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
    dataFile.close();
    textFile.close(); //close and save SD file, otherwise precious data will be lost
    servo_1.write(pos1);              // tell servo to go to position in variable 'pos'
    servo_2.write(pos2);              // tell servo to go to position in variable 'pos'
    servo_3.write(pos3);              // tell servo to go to position in variable 'pos'
    digitalWrite(greenLedPin, LOW);    // turn LED off
    digitalWrite(redLedPin, LOW);    // turn LED off


    delay(100000000); //is there a way to break out of the loop
  }

  time_for_loop = (micros()-time)*0.000001; //time taken from start of loop
}
