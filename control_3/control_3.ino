//Libraries
#include <Wire.h> //The Wire library is used for I2C communication
#include <math.h> //Maths Library
#include <SD.h> //SD library
#include <Servo.h> //servo library
#include <MatrixMath.h> //matrix library


//Sensor addresses
//I2C devices each have an address. The address is defined in the datasheet for the device. The ITG-3200 breakout board can have different address depending on how
//the jumper on top of the board is configured. By default, the jumper is connected to the VDD pin. When the jumper is connected to the VDD pin the I2C address
//is 0x69.
#define Aaddress 0x53 // I2C 7bit address of ADXL345 of accelerometer
#define Gaddress 0x68 //address of ITG-3200 gyro
#define Maddress 0x1E //0011110b, I2C 7bit address of HMC5883
#define Baddress 0x77  // I2C address of BMP085


//This is a list of registers in the ITG-3200. Registers are parameters that determine how the sensor will behave, or they can hold data that represent the
//sensors current status.
//To learn more about the registers on the ITG-3200, download and read the datasheet.
char WHO_AM_I = 0x00;
char SMPLRT_DIV= 0x15;
char DLPF_FS = 0x16;
char GYRO_XOUT_H = 0x1D;
char GYRO_XOUT_L = 0x1E;
char GYRO_YOUT_H = 0x1F;
char GYRO_YOUT_L = 0x20;
char GYRO_ZOUT_H = 0x21;
char GYRO_ZOUT_L = 0x22;


//This is a list of settings that can be loaded into the registers.
//DLPF, Full Scale Register Bits
//FS_SEL must be set to 3 for proper operation
//Set DLPF_CFG to 3 for 1kHz Fint and 42 Hz Low Pass Filter
char DLPF_CFG_0 = 1<<0;
char DLPF_CFG_1 = 1<<1;
char DLPF_CFG_2 = 1<<2;
char DLPF_FS_SEL_0 = 1<<3;
char DLPF_FS_SEL_1 = 1<<4;
 
 
//Global variables
File myFile; //holds information on file ebing written to on SD card

int i=1, n=0, a=0, b=0; //used in for and if statements
int Ax, Ay, Az; //triple axis data for accelormeter 
int Mx, My, Mz; //triple axis data for magnetometer
int led1Pin = 8;  // green LED is connected to pin 8
int led2Pin = 7;  // red LED is connected to pin 8
int recoveryPin = 6;  // parachute charge is set off by pin 9

unsigned long time=0, start_time = 0, record_time = 0; //long variables for dealing with time 

float time_for_loop = 0; //time for a loop
float Gsensitivity = 14.375; //convert to degrees per second, times by correct factor according to range.
float GxOff, GyOff, GzOff; //gyro offset values  
float Asensitivity = 3.262; //Senstivity LMB/m/s^2, 26.093 for 2g/full resolution, 13.048g for 4g, 6.524 for 8g, 3.262 for 16g, these are typical value and may not be accurate
float Accx, Accy, Accz, Mag_acc; //variables for acceleration in m/s^2
float accel_angle_x, accel_angle_y; //tilt angles from accelerometer
float accel_center_x = 0, accel_center_y = 0, accel_center_z = 0; //alternative offsets for accelerometer which are used in the software, these seem better

//Barometer variables (lots)
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature;
long pressure;

//inorder to calculate alltitude need to know
//const float p0 = 100400;     // Pressure at sea level (Pa) standard 101325
//float altitude, previous_altitude=0;

//creat servo variables
Servo servo_1;  // create servo object to control a servo 
Servo servo_2;  // create servo object to control a servo 
Servo servo_3;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

int pos1 = 95, pos2 = 90, pos3 = 90, s1, s2, s3, smax=150, smin=30, k=0;    // variable to store the servo position 
float r=0, theta=0, s_a, s_b, s_c, temp1, temp2, temp3;
int d=20, D=48, horn=12, link=28; //geometric values hopefully these can be integers
//const float pi=3.14;                

// PID constants
float Kp=50, Ki=0, Kd=0;

//direction variables
float w[3]; //angular velocity vector
float eye[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} }; //indentity matrix
float R1[3][3]; //rotation matrix 1
float R2[3][3]; //rotation matrix 2
//int I[3][1] = { {1}, {0}, {0} }; //unit vector inertial system
float J[3] = {0, 1, 0}; //unit vector inertial system
//int K[3][1] = { {0}, {0}, {1} }; //unit vector inertial system
float Pitch=0, Yaw=0; // used to calculate gimbal movement
float Heading; // angle between rocket vertical and inertial vertical
float Integral = 0;
float Der = 0;
float r_max = 9.5; //to be determined
float previous_Heading = 0;
float previous_r = 0;


void setup() //setup instructions
{
  /* 
  //to set the values as below comment out the value assigned to phi and zeta later
  Freq_cut_off_1 = 0.08; //cut off frequency for pitch and yaw
  zeta = 1/(2*Freq_cut_off_1*M_PI); //adjust this for how responsive the angles should be/dependedent on gyro
  Freq_cut_off_2 = 0.04; //cut off frequency for roll
  phi = 1/(2*Freq_cut_off_2*M_PI); //time constant for roll
  */
  Gsensitivity = Gsensitivity * 180 / M_PI; //convert degrees per second to radians per second, note gyro output gets divided by this value
  Matrix.Copy((float*) eye, 3, 3, (float*) R1);
  
  //Serial.begin(115200); //Create a serial connection using a 115200bps baud rate.
  
  pinMode(10, OUTPUT); //set SD CS pin to output
  pinMode(led1Pin, OUTPUT); // Set the LED 1 pin as output
  pinMode(led2Pin, OUTPUT); // Set the LED 2 pin as output
  pinMode(recoveryPin, OUTPUT); // Set the recovery pin as output
  digitalWrite(recoveryPin, LOW); //should default to low
  
  SD.begin(); //begin SDness
  
  myFile = SD.open("Log_C3.txt", FILE_WRITE); //create file on SD card
    
  Wire.begin(); //Initialize the I2C communication. This will set the Arduino up as the 'Master' device.
  
    
  writeTo(Maddress, 0x02, 0x00); //Put the HMC5883 IC into the correct operating mode,continuous measurement mode
  
  bmp085Calibration(); //calibrate Barometer
  
 //Configure the gyroscope
  //Set the gyroscope scale for the outputs to +/-2000 degrees per second
  //bits 3 and 4 must be 1 1 for operation, bits 1, 2 and 3 set low pass filter
  //unofficially FS_SEL= 0 is 250°/sec, FS_SEL= 1 is 500°/sec, FS_SEL= 2 is 1000°/sec, now needs a different sensitivity factor
  //so set value to 2 (for 100Hz Low pass filter bandwidth) + 16 (1000°/sec)
  writeTo(Gaddress, DLPF_FS, 0x1A);
  //Set the sample rate to 100 hz this can be changed This register determines the sample rate of the ITG-3200 gyros. The gyros outputs are sampled internally at either 1kHz or 8kHz, determined by the DLPF_CFG setting (see register 22). This sampling is then filtered digitally and delivered into the sensor registers after the number of cycles determined by this register. The sample rate is given by the following formula:
  //Fsample = Finternal / (divider+1), where Finternal is either 1kHz or 8kHz
  //As an example, if the internal sampling is at 1kHz, then setting this register to 7 would give the following:
  //Fsample = 1kHz / (7 + 1) = 125Hz, or 8ms per sample
  writeTo(Gaddress, SMPLRT_DIV, 9); //I think the time for a sample should match the loop time
  
   //Set the range on the ADXL345
  writeTo(Aaddress, 0x31, 3); //0=2g, 1=4g, 2=8g, 3=16g  range, add 4 for full_res mode, not sure how that works 

  //Set the frequency on the ADXL345
  writeTo(Aaddress, 0x2C, 10); //set to 100Hz

      //Set power mode on the ADXL345
  writeTo(Aaddress, 0x2D, 8); //put ADXL345 into measure mode

   
  //set accelerometer offsets to 0
  int AxOff=5, AyOff=5, AzOff=5; //for some strange reason setting these to 0 gave more false readings
  writeTo(Aaddress, 0x1E, AxOff);
  writeTo(Aaddress, 0x1F, AyOff);
  writeTo(Aaddress, 0x20, AzOff);
   
  delay(1000); //make sure everything is static
  
    //Accelerometer Calibration
  float AxCal=0, AyCal=0, AzCal=0; 
  //find average values at rest 
  for (i=0; i<25; i++) {
    //Read the x,y and z output rates from the accelerometer.
    AxCal = AxCal + readAX();
    AyCal = AyCal - readAY(); //make upwards positive
    AzCal = AzCal + readAZ();
    delay(10);
  }

  //Alternatvie accelerometer calibration
  accel_center_x=(AxCal/25)/Asensitivity; //in g then in mg
  accel_center_y=(AyCal/25)/Asensitivity + 9.81; //account for gravity
  accel_center_z=(AzCal/25)/Asensitivity; 
  
    //Gyro Calibration
  int GxCal=0, GyCal=0, GzCal=0; 
  
  for (i=0; i<50; i++) {
    //Read the x,y and z output rates from the gyroscope and take an average of 50 results
    GxCal = GxCal + readGX();
    GyCal = GyCal + readGY();
    GzCal = GzCal + readGZ();
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
     
   //Serial.println();
  
  //print column headers
  //myFile.println("Time, Time for loop in ms, Acc x, Acc y, Acc z, Gx Rate, Gy Rate, GzRate,  Mx,   My,   Mz, Temp, Pressure, Pitch, Yaw, Roll, Mag accel, Theta, r, s1, s2, s3");
  myFile.println("Time for loop in ms, Heading");
  //myFile.println("Time for loop in ms, Gx Rate, Gy Rate, GzRate, Pitch, Yaw, Theta, r");
  if(myFile) digitalWrite(led1Pin, HIGH);   // turn LED on if file has been created successfully
 
}



void loop()
{
  time = micros(); //time at start of loop, in micro seconds

  //Read the x,y and z output rates from the gyroscope.
  //angular velocity vector need to align/adjust gyro axis to rocket axis, clockwise rotations are positive
  w[0] = -(1.000*readGX() - GxOff)/Gsensitivity;
  w[1] = (1.000*readGY() - GyOff)/Gsensitivity; // gyro appears to use left hand coordinate system
  w[2] = (1.000*readGZ() - GzOff)/Gsensitivity;
  
  //gyro angle, found by intergration 
  /* R1 is the previous rotation matrix which rotates the inertial 
    coordinate system to the body one
    R2 is the new matrix updated with the rotation rates from the body frame
    w(1) is rate of rotation about the rocket's x axis 
    w(2) is rate of rotation about the rocket's y axis
    w(3) is rate of rotation about the rocket's z axis
  */
   
  // cross product of w converted to matrix form multiplied by time for loop and + identity matrix
  float M[3][3] = {
    {1,-w[2]*time_for_loop,w[1]*time_for_loop},
    {w[2]*time_for_loop,1,-w[0]*time_for_loop},
    {-w[1]*time_for_loop,w[0]*time_for_loop,1}
  }; 
   
  Matrix.Multiply((float*)R1,(float*)M,3,3,3,(float*)R2);
  Matrix.Copy((float*) R2, 3, 3, (float*) R1);
   
  if(a==0) { // only read accelerometer when on the ground
  //Accelerometer Read data from each axis, 2 registers per axis
    Ax = readAX(); 
    Accx = Ax/Asensitivity -accel_center_x; //convert to SI units and zero
    Ay = - readAY(); //make upwards positive
    Accy = Ay/Asensitivity-accel_center_y;
    Az = readAZ();
    Accz = Az/Asensitivity-accel_center_z;
    Mag_acc=sqrt(Accx*Accx+Accy*Accy+Accz*Accz); //calucate the magnitude
  }
    
    //Serial.println(Mag_acc);
   
  if(Mag_acc<30 and a==0) { //if the rocket hasn't experienced an accleration over 30 m/s^2, a is used so that it doesn't revert if the acceleration drops back below 30
     // Serial.println("Low acceleration mode");
      //angles from gyro will drift slowly, these need to be kept constant while the sensor is stationary
      //so set them to 0 whilst on the ground 
    //set R1 = to indentity matrix
    Matrix.Copy((float*) eye, 3, 3, (float*) R1);
  }
  else {
    digitalWrite(led2Pin, HIGH ); //turn red led on
    //Serial.println("High acceleration mode");
    Matrix.Normalize3x3((float*)R1); //remove errors so dot product doesn't go complex

    Pitch = asin( Matrix.dot((float*)R1,(float*)J,3,3,3) ); //this gives the angle between the z axis and the horizontal plan (in which I and K reside) when no pitch is required this angle will be 0
    Yaw = asin ( Matrix.dot((float*)R1,(float*)J,3,3,1) );
    Heading = acos(Matrix.dot((float*)R1,(float*)J,3,3,2) );
  
    if (Pitch == 0) Pitch = 0.00001; // so we don't get divide by 0
  
    // find theta 
    //need to define pitch and yaw in relation to theta which is defined as 0 at servo one and increases clockwise when looking aft to fore,
    //so let pitch be positive in theta = 0° direction and yaw be positive in theta = 90° , note these may need to be altered
    theta=atan2(Yaw,Pitch);

    
    if (previous_r < r_max) Integral = Integral + Heading * time_for_loop; //stop integral causing windup      
    Der = (Heading - previous_Heading)/ time_for_loop; // might want a low pass filter on here
    previous_Heading = Heading;
    //PID control to find r, r is a scalar which can be found from the magntiude of the rotation, aka the heading
    r = Kp * Heading + Ki * Integral + Kd * Der;
    previous_r = r;
    if (r > r_max) r = r_max;
    //delay(1000);
   
    //convert into servo movements
    //do {
      s_a = sqrt(sq(r*cos(theta) + d - D) + sq(r*sin(theta))); //angle in radians 
      s_b = sqrt(sq(r*cos(theta) - d*0.5 + D*0.5) + sq(r*sin(theta) + d*0.866 - D*0.866)); //angle in radians 
      s_c = sqrt(sq(r*cos(theta) - d*0.5 + D*0.5) + sq(r*sin(theta) - d*0.866 + D*0.866)); //angle in radians 
      
      s_a = s_a-link;
      s_b = s_b-link;
      s_c = s_c-link;
      
      //convert these into servo angles
      temp1=(2*s_a + 2*link - 1.0/(2*horn) - sqrt(sq(1.0/(2*horn) - 2*link - 2*s_a) - 4*(sq(s_a) + 2*link*s_a)*(1 + 1/(8*pow(horn,3)))))/(2*(1 + 1/(8*pow(horn,3))));
      temp2=(2*s_b + 2*link - 1.0/(2*horn) - sqrt(sq(1.0/(2*horn) - 2*link - 2*s_b) - 4*(sq(s_b) + 2*link*s_b)*(1 + 1/(8*pow(horn,3)))))/(2*(1 + 1/(8*pow(horn,3))));
      temp3=(2*s_c + 2*link - 1.0/(2*horn) - sqrt(sq(1.0/(2*horn) - 2*link - 2*s_c) - 4*(sq(s_c) + 2*link*s_c)*(1 + 1/(8*pow(horn,3)))))/(2*(1 + 1/(8*pow(horn,3))));
      
      // convert into servo degrees 180/M_PI = 57
      s1 = 57*asin(temp1/horn) + pos1;
      s2 = 57*asin(temp2/horn) + pos2;
      s3 = 57*asin(temp3/horn) + pos3;
      
    //update servos with new positions
    servo_1.write(s1);              
    servo_2.write(s2);               
    servo_3.write(s3);   
  
    /* 
    //read magnetometer
    Mx=readMX();
    My=readMY();
    Mz=readMZ();
    
      
    temperature = bmp085GetTemperature(bmp085ReadUT()); //read temperature from the barometer which has a temp sesnors, and convert to degrees*10
    pressure = bmp085GetPressure(bmp085ReadUP()); // read pressure from barometer, and convert to Pa
    */
    
    
    //print data to file on SD card, using commas to seperate
    //myFile.print(time*0.000001);
    //myFile.print(",   ");
    //myFile.print(time_for_loop*1000);
    /*myFile.print(",   ");
    myFile.print(Accx);
    myFile.print(",    ");
    myFile.print(Accy);
    myFile.print(",    ");
    myFile.print(Accz);
    myFile.print(",    ");
    myFile.print(GxRate);
    myFile.print(",    ");
    myFile.print(GyRate);
    myFile.print(",    ");
    myFile.print(GzRate);
    myFile.print(",    ");
    myFile.print(Mx);
    myFile.print(",    ");
    myFile.print(My);
    myFile.print(",    ");
    myFile.print(Mz);
    myFile.print(",    ");
    myFile.print(temperature);
    myFile.print(",    ");
    myFile.print(pressure);
    myFile.print(",    ");
    myFile.print(Pitch);
    myFile.print(",    ");
    /*myFile.print(Yaw);
    myFile.print(",    ");
    //myFile.print(Mag_acc);
    //myFile.print(",    ");
    myFile.print(theta);
    myFile.print(",    ");
    /*myFile.print(r); 
    myFile.println(",    ");
    /*myFile.print(s1);
    myFile.print(",    ");
    myFile.print(s2);
    myFile.print(",    ");
    myFile.println(s3); 
    */
    
    //abridged version look at storing this in memory
    myFile.print(time_for_loop*1000);
    myFile.print(",   ");
    myFile.println(Heading);

    a=1;
  }
  
  //if statement sets start time when data starts to be recorded
  if (a==1 and b==0) { 
    start_time = micros();
    b=1;
  }
  
  record_time = micros() - start_time; //time spent recording data can be calculated
  
  //if statement to stop the loop after 60 minutes or after 60 seconds of recording
  if(micros()>2E9 or (record_time > 30E6 and a==1)) { //1E9 is 30 mins 3E7 is 30 seconds 5E6 is 5 seconds
    myFile.close(); //close and save SD file, otherwise precious data will be lost
    servo_1.write(pos1);              // tell servo to go to position in variable 'pos' 
    servo_2.write(pos2);              // tell servo to go to position in variable 'pos' 
    servo_3.write(pos3);              // tell servo to go to position in variable 'pos' 
    digitalWrite(led1Pin, LOW);    // turn LED off
    digitalWrite(led2Pin, LOW);    // turn LED off
    delay(1000); //wait a second
    digitalWrite(recoveryPin, HIGH); //deploy chute
    delay(3000); //wait 3 seconds
    digitalWrite(recoveryPin, LOW); //turn off mosfet
    delay(100000000); //is there a way to break out of the loop
    
  }

  time_for_loop=(micros()-time)*0.000001; //time taken from start of loop
}

//This function will write a value to a register on a sensors.
//Parameters:
//  device: The I2C address of the sensor. 
//  registerAddress: The address of the register on the sensor that should be written to.
//  data: The value to be written to the specified register.
void writeTo(int device, byte registerAddress, byte data)
{
  //Initiate a communication sequence with the desired i2c device
  Wire.beginTransmission(device);
  //Tell the I2C address which register we are writing to
  Wire.write(registerAddress);
  //Send the value to write to the specified register
  Wire.write(data);
  //End the communication sequence
  Wire.endTransmission();
}

//This function will read the data from a specified register and return the value.
//Parameters:
//  device: The I2C address of the sensor. 
//  registerAddress: The address of the register on the sensor that should be read
//Return:
//  unsigned char: The value currently residing in the specified register
unsigned char readFrom(int device, byte registerAddress)
{
  //This variable will hold the contents read from the i2c device.
  unsigned char data=0;
  
  //Send the register address to be read.
  Wire.beginTransmission(device);
  //Send the Register Address
  Wire.write(registerAddress);
  //End the communication sequence.
  Wire.endTransmission();
  
  //Ask the I2C device for data
  Wire.beginTransmission(device);
  Wire.requestFrom(device, 1);
  
  //Wait for a response from the I2C device
  if(Wire.available()){
    //Save the data sent from the I2C device
    data = Wire.read();
  }
  
  //End the communication sequence.
  Wire.endTransmission();
  
  //Return the data read during the operation
  return data;
}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int xRate = readX();
int readGX(void)
{
  int data=0;
  data = readFrom(Gaddress, GYRO_XOUT_H)<<8;
  data |= readFrom(Gaddress, GYRO_XOUT_L);  
  
  return data;
}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int yRate = readY();
int readGY(void)
{
  int data=0;
  data = readFrom(Gaddress, GYRO_YOUT_H)<<8;
  data |= readFrom(Gaddress, GYRO_YOUT_L);  
  
  return data;
}

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second. 
//Usage: int zRate = readZ();
int readGZ(void)
{
  int data=0;
  data = readFrom(Gaddress, GYRO_ZOUT_H)<<8;
  data |= readFrom(Gaddress, GYRO_ZOUT_L);  
  
  return data;
}

//This function is used to read the X-Axis value of the accelerometer. 
//NOTE: This value is NOT in SI units.
//Usage: 
int readAX(void)
{
  int data=0;
  data = readFrom(Aaddress, 0x33)<<8;
  data |= readFrom(Aaddress, 0x32); 
  
  return data;
}

//This function is used to read the Y-Axis value of the accelerometer. 
//NOTE: This value is NOT in SI units. 
//Usage:
int readAY(void)
{
  int data=0;
  data = readFrom(Aaddress, 0x35)<<8;
  data |= readFrom(Aaddress, 0x34);
  
  return data;
}

//This function is used to read the Z-Axis value of the accelerometer. 
//NOTE: This value is NOT in SI units. 
//Usage: 
int readAZ(void)
{
  int data=0;
  data = readFrom(Aaddress, 0x37)<<8;
  data |= readFrom(Aaddress, 0x36);  
  
  return data;
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char registeraddress)
{
  unsigned char data;
  
  Wire.beginTransmission(Baddress);
  Wire.write(registeraddress);
  Wire.endTransmission();
  
  Wire.requestFrom(Baddress, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char registeraddress)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(Baddress);
  Wire.write(registeraddress);
  Wire.endTransmission();
  
  Wire.requestFrom(Baddress, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  writeTo(Baddress, 0xF4, 0x2E);
 
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  writeTo(Baddress, 0xF4, (0x34 + (OSS<<6)));
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(Baddress);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(Baddress, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

//This function is used to read the magnetometer in the X direction. 
int readMX(void)
{
  int data=0;
  data = readFrom(Maddress, 0x03)<<8; //0x03 is MSB register
  data |= readFrom(Maddress, 0x04);  //LSB register
  
  return data;
}

//This function is used to read the magnetometer in the Y direction. 
int readMY(void)
{
  int data=0;
  data = readFrom(Maddress, 0x07)<<8; //0x07 is MSB register
  data |= readFrom(Maddress, 0x08);  //LSB register
  
  return data;
}

//This function is used to read the magnetometer in the Z direction. 
int readMZ(void)
{
  int data=0;
  data = readFrom(Maddress, 0x05)<<8; //0x03 is MSB register
  data |= readFrom(Maddress, 0x06);  //LSB register
  
  return data;
}

