/**
Visualize a cube which will assumes the orientation described
in a quaternion coming from the serial port.

INSTRUCTIONS: 
This program has to be run when you have the FreeIMU_serial
program running on your Arduino and the Arduino connected to your PC.
Remember to set the serialPort variable below to point to the name the
Arduino serial port has in your system. You can get the port using the
Arduino IDE from Tools->Serial Port: the selected entry is what you have
to use as serialPort variable.


Copyright (C) 2011-2012 Fabio Varesano - http://www.varesano.net/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

import processing.serial.*;
import processing.opengl.*;

Serial myPort;  // Create object from Serial class

final String serialPort = "COM5"; // replace this with your serial port. On windows you will need something like "COM1".

float [] M = new float [9];
float [] hM = null;
float [] Euler = new float [3]; // psi, theta, phi

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600;

final int burst = 32;
int count = 0;

void myDelay(int time) {
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}

void setup() 
{
  size(VIEW_SIZE_X, VIEW_SIZE_Y, OPENGL);
  myPort = new Serial(this, serialPort, 115200);
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw"); 
  
  println("Waiting IMU..");
  
  myPort.clear();
  
  while (myPort.available() == 0) {
   // myPort.write("v");
    myDelay(1000);
  }
  println(myPort.readStringUntil('\n'));
  myPort.write("q" + char(burst));
  myPort.bufferUntil('\n');
}


float decodeFloat(String inString) {
  byte [] inData = new byte[4];
  
  if(inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }
      
  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}

void serialEvent(Serial p) {
  if(p.available() >= 18) {
    String inputString = p.readStringUntil('\n');
    //print(inputString);
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      if(inputStringArr.length >= 10) { // q1,q2,q3,q4,\r\n so we have 5 elements
        M[0] = decodeFloat(inputStringArr[0]);
        M[1] = decodeFloat(inputStringArr[1]);
        M[2] = decodeFloat(inputStringArr[2]);
        M[3] = decodeFloat(inputStringArr[3]);
        M[4] = decodeFloat(inputStringArr[4]);
        M[5] = decodeFloat(inputStringArr[5]);
        M[6] = decodeFloat(inputStringArr[6]);
        M[7] = decodeFloat(inputStringArr[7]);
        M[8] = decodeFloat(inputStringArr[8]);
      }
    }
    count = count + 1;
    if(burst == count) { // ask more data when burst completed
      p.write("q" + char(burst));
      count = 0;
    }
  }
}



void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawCube() {  
  pushMatrix();
    translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50, 0);
    scale(5,5,5);
    
    // a demonstration of the following is at 
    // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
    rotateZ(-Euler[2]);
    rotateX(-Euler[1]);
    rotateY(-Euler[0]);
    
    buildBoxShape();
    
  popMatrix();
}


void draw() {
  background(#000000);
  fill(#ffffff);
  
  if(hM != null) { // use home quaternion
    MToEuler(MProd(hM, M), Euler);
    text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  }
  else {
    MToEuler(q, Euler);
    text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
  
  textFont(font, 20);
  textAlign(LEFT, TOP);
  text("Q:\n" + q[0] + "\n" + q[1] + "\n" + q[2] + "\n" + q[3], 20, 20);
  text("Euler Angles:\nYaw (psi)  : " + degrees(Euler[0]) + "\nPitch (theta): " + degrees(Euler[1]) + "\nRoll (phi)  : " + degrees(Euler[2]), 200, 20);
  
  drawCube();
  //myPort.write("q" + 1);
}


void keyPressed() {
  if(key == 'h') {
    println("pressed h");
    
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hM = Mtran(M);
    
  }
  else if(key == 'n') {
    println("pressed n");
    hM = null;
  }
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void MToEuler(float [] M, float [] euler) {
  //euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
 // euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
 // euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
  
  float theta_1, theta_2, psi_1, psi_2, phi_1, phi2;
  if(M[6]==1 || M[6] ==-1){
    phi_1=0;
    float delta;
    delta = atan2(M[1],M[2]);
    if(M[6]=-1){ 
      theta_1=M_PI/2;
      psi=phi+detla;
    }
    else{
      theta_1=-M_PI/2;
      psi=-phi+detla;
    }
  }
  else {
    theta_1 = -asin(M[6]);
    theta_1 = M_PI + asin(M[6]);
    psi_1 = atan2(M[7]/cos(theta_1),M[8]/cos(theta_1));
    psi_2 = atan2(M[7]/cos(theta_2),M[8]/cos(theta_2));
    phi_1=atan2(M[3]/cos(theta_1), M[0]/cos(theta_1));
    phi_2=atan2(M[3]/cos(theta_2), M[0]/cos(theta_2));
  }
  euler[0] = psi_1; //psi
  euler[1] = theta_1; //theta
  euler[3] = phi_1;//phi
}

void qToEuler(float [] M, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}
float [] MProd(float [] A, float [] B) {
  float [] M = new float[9];
  int m=3, p=3, m=3  
  int i, j, k;
      for (i=0;i<m;i++)
          for(j=0;j<n;j++)
          {
              M[n*i+j]=0;
              for (k=0;k<p;k++)
                  M[n*i+j]= M[n*i+j]+A[p*i+k]*B[n*k+j];
          }
  
  return M;
}


// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
  float [] q = new float[4];
  
  float halfAngle = angle / 2.0;
  float sinHalfAngle = sin(halfAngle);
  q[0] = cos(halfAngle);
  q[1] = -axis[0] * sinHalfAngle;
  q[2] = -axis[1] * sinHalfAngle;
  q[3] = -axis[2] * sinHalfAngle;
  
  return q;
}

// return the quaternion conjugate of quat
float [] Mtran(float [] M) {
  float [] tran = new float[9];
  int m=3, n=3;
  int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            tran[m*j+i]=M[n*i+j];
  
  return tran;
}



