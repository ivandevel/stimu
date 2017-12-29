/**
Visualize a cube which will assumes the orientation described
in a quaternion coming from the serial port. 

INSTRUCTIONS: 
This program has to be run when you have the FreeIMU_quaternion
program running on your Arduino and the Arduino connected to your PC.
Remember to set the serialPort variable below to point to the name the
Arduino serial port has in your system. You can get the port using the
Arduino IDE from Tools->Serial Port: the selected entry is what you have
to use as serialPort variable.


Copyright (C) 2011 Fabio Varesano - http://www.varesano.net/

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

Serial myPort;  // Create object from Serial class

final String serialPort = "COM8"; // replace this with your serial port. On windows you will need something like "COM1".
float [] accel = new float [3];
float [] gyro = new float [3];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi
int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)
PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600; 
PShape s;
boolean hasData = false;
// Data for compare
int maxSamples = 400;
int actualSample = 0;
int sampleStep = 1;
String[] AccelNames = { "PICH", "ROLL",  "YAW"};
String[] GyroNames = { "PICH", "ROLL",  "YAW"};
String[] PressureNames = { "ALTITUDE", "---",  "---"};
float[][] accelValues = new float[3][maxSamples];
float[][] gyroValues = new float[3][maxSamples];
float[][] pressureValues = new float [3][maxSamples];
PGraphics pgChart;
int[] colors = { #ff4444, #33ff99, #5588ff };

void setup() 
{
  size(1200, 900, P3D);
  myPort = new Serial(this, serialPort, 115200);  
  //s = loadShape("Drone.obj");
  delay(100);
  myPort.clear();
  myPort.write("1");
}

void serialEvent (Serial myPort) 
{
  if(myPort.available() >= 10) {
    String inputString = myPort.readStringUntil('\n');
    
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      //print(inputStringArr);
      
      if(inputStringArr.length > 8) { // q1,q2,q3,q4,\r\n so we have 5 elements
      //print(inputStringArr[0]+"\r\n");
      //print(inputStringArr[1]+"\r\n");
      //print(inputStringArr[2]+"\r\n");
      //print(inputStringArr[3]+"\r\n");
      
        Euler[2] = Float.parseFloat(inputStringArr[0]);
        Euler[1] = Float.parseFloat(inputStringArr[1]);
        Euler[0] = Float.parseFloat(inputStringArr[2]);
        
        accelValues[0][actualSample] = (Float.parseFloat(inputStringArr[3]))*100;
        accelValues[1][actualSample] = (Float.parseFloat(inputStringArr[4]))*100;
        accelValues[2][actualSample] = (Float.parseFloat(inputStringArr[5]))*100;
        
        gyroValues[0][actualSample] = (Float.parseFloat(inputStringArr[6]));
        gyroValues[1][actualSample] = (Float.parseFloat(inputStringArr[7]));
        gyroValues[2][actualSample] = (Float.parseFloat(inputStringArr[8]));
        //pressureValues[0][actualSample] = (Float.parseFloat(inputStringArr[9]));

        if (actualSample > 1)
    {
      hasData = true;
    }
        
        if (actualSample == (maxSamples-1))
    {
      //nextSample(pyrValues);
      //nextSample(pyrValuesFiltered);
      nextSample(accelValues);
      nextSample(gyroValues);
      nextSample(pressureValues);
      //nextSample(rollValues);
    } else
    {
      actualSample++;
    }
      }
      
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
    translate(800, 400, 0);
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
  
  //translate(800/2, 600/2);
  //shape(s, 90, 90);
  
  //readQ();
  /*
  if(hq != null) { // use home quaternion
    //quaternionToEuler(quatProd(hq, q), Euler);
    text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  }
  else {
    //quaternionToEuler(q, Euler);
    text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
  
  textAlign(LEFT, TOP);
  text("Q:\n" + q[0] + "\n" + q[1] + "\n" + q[2] + "\n" + q[3], 20, 20);
  text("Euler Angles:\nYaw (psi)  : " + degrees(Euler[0]) + "\nPitch (theta): " + degrees(Euler[1]) + "\nRoll (phi)  : " + degrees(Euler[2]), 200, 20);
  */
  drawCube(); 
  drawChart("Accelerometer [deg]", AccelNames, accelValues, 10, 10, 200, true, true, -125, 125, 25); 
  drawChart("Gyroscope [dps]", GyroNames, gyroValues, 10, 290, 200, true, true, -200, 200,50); 
  drawChart("Altitude [meters]", PressureNames, pressureValues, 10, 570, 200, true, true, 0, 200,50); 
}


void keyPressed() {
  if(key == 'h') {
    println("pressed h");
    
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    //hq = quatConjugate(q);
    
  }
  else if(key == 'n') {
    println("pressed n");
    hq = null;
  }
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];
  
  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  
  return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];
  
  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];
  
  return conj;
}

void drawChart(String title, String[] series, float[][] chart, int x, int y, int h, boolean symmetric, boolean fixed, int fixedMin, int fixedMax, int hlines) 
{
  int actualColor = 0;
  
  int maxA = 0;
  int maxB = 0;
  int maxAB = 0;
  
  int min = 0;
  int max = 0;
  int step = 0;
  int divide = 0;
 
  if (fixed)
  {
    min = fixedMin;
    max = fixedMax;
    step = hlines;
  } else
  {
    if (hlines > 2)
    {
      divide = (hlines - 2);
    } else
    {
      divide = 1;
    }
      
    if (symmetric)
    {
      maxA = (int)abs(getMin(chart));
      maxB = (int)abs(getMax(chart));
      maxAB = max(maxA, maxB);
      step = (maxAB * 2) / divide;
      min = -maxAB-step;
      max = maxAB+step;
    } else
    {
      min = (int)(getMin(chart));
      max = (int)(getMax(chart));
      
      if ((max >= 0) && (min <= 0)) step = (abs(min) + abs(max)) / divide; 
      if ((max < 0) && (min < 0)) step = abs(min - max) / divide; 
      if ((max > 0) && (min > 0)) step = (max - min) / divide; 
      
      if (divide > 1)
      {
        min -= step;
        max += step;
      }
    }
  }
  
  pgChart = createGraphics((maxSamples*sampleStep)+50, h+60);

  pgChart.beginDraw();

  // Draw chart area and title
  pgChart.background(0);
  pgChart.strokeWeight(1);
  pgChart.noFill();
  pgChart.stroke(50);
  pgChart.rect(0, 0, (maxSamples*sampleStep)+49, h+59);
  pgChart.text(title, ((maxSamples*sampleStep)/2)-(textWidth(title)/2)+40, 20);

  // Draw chart description
  String Description[] = new String[chart.length];
  int DescriptionWidth[] = new int[chart.length];
  int DesctiptionTotalWidth = 0;
  int DescriptionOffset = 0;

  for (int j = 0; j < chart.length; j++)
  {
     Description[j] = "  "+series[j]+" = ";
     DescriptionWidth[j] += textWidth(Description[j]+"+000.00");
     Description[j] += nf(chart[j][actualSample-1], 0, 2)+"  ";
     DesctiptionTotalWidth += DescriptionWidth[j];
  }

  actualColor = 0;

  for (int j = 0; j < chart.length; j++)
  {
    pgChart.fill(colors[actualColor]);
    pgChart.text(Description[j], ((maxSamples*sampleStep)/2)-(DesctiptionTotalWidth/2)+DescriptionOffset+40, h+50);
    DescriptionOffset += DescriptionWidth[j];
    actualColor++;
    if (actualColor >= colors.length) actualColor = 0;
  }

  // Draw H-Lines 
  pgChart.stroke(100);

  for (float t = min; t <= max; t=t+step)
  {
    float line = map(t, min, max, 0, h);
    pgChart.line(40, h-line+30, (maxSamples*sampleStep)+40, h-line+30);
    pgChart.fill(200, 200, 200);
    pgChart.textSize(12);
    pgChart.text(int(t), 5, h-line+34);
  }

  // Draw data series
  pgChart.strokeWeight(2);

  for (int i = 1; i < actualSample; i++)
  {
    actualColor = 0;

    for (int j = 0; j < chart.length; j++)
    {
      pgChart.stroke(colors[actualColor]);

      float d0 = chart[j][i-1];
      float d1 = chart[j][i];

      if (d0 < min) d0 = min;
      if (d0 > max) d0 = max;
      if (d1 < min) d1 = min;
      if (d1 > max) d1 = max;

      float v0 = map(d0, min, max, 0, h);
      float v1 = map(d1,   min, max, 0, h);

      pgChart.line(((i-1)*sampleStep)+40, h-v0+30, (i*sampleStep)+40, h-v1+30);

      actualColor++;

      if (actualColor >= colors.length) actualColor = 0;
    }
  }

  pgChart.endDraw();

  image(pgChart, x, y);
}
float getMin(float[][] chart)
{
  float minValue = 0;
  float[] testValues = new float[chart.length];
  float testMin = 0;

  for (int i = 0; i < actualSample; i++)
  {
    for (int j = 0; j < testValues.length; j++)
    {
      testValues[j] = chart[j][i];
    }
    
    testMin = min(testValues);
    
    if (i == 0)
    {
      minValue = testMin;
    } else
    {
      if (minValue > testMin) minValue = testMin;
    }
  }
 
  return ceil(minValue)-1; 
}

float getMax(float[][] chart)
{
  float maxValue = 0;
  float[] testValues = new float[chart.length];
  float testMax = 0;

  for (int i = 0; i < actualSample; i++)
  {
    for (int j = 0; j < testValues.length; j++)
    {
      testValues[j] = chart[j][i];
    }
    
    testMax = max(testValues);

    if (i == 0)
    {
      maxValue = testMax;
    } else
    {
      if (maxValue < testMax) maxValue = testMax;
    }
  }
 
  return ceil(maxValue); 
}

void nextSample(float[][] chart)
{
    for (int j = 0; j < chart.length; j++)
    {
      float last = chart[j][maxSamples-1];

      for (int i = 1; i < maxSamples; i++)
      {
        chart[j][i-1] = chart[j][i];
      }

      chart[j][(maxSamples-1)] = last;
    }
}