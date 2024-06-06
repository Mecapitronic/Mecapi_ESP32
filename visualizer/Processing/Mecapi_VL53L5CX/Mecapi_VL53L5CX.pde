/*
  Visualizing VL53L5CX Output As A 3D Mesh
  By: Nick Poole
  SparkFun Electronics
  Date: November 3, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example is a companion to the Arduino Library Example 4 "MaxOutput"
  Which you need to be running on an attached device. You'll need to 
  change "COM13" below to the name of your serial port. 
  
  Once the sketch is running, it may take a few seconds for the sensor to respond,
  during which you'll see a red square in the window. If it takes more than a few
  seconds for this to change, check your serial port and run the sketch again. 
  
  Once data is incoming, you can rotate the mesh by dragging within the display window.
  You can zoom the mesh in and out using the scroll wheel on your mouse.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

import processing.serial.*;

// Serial Port Variables
Serial port; // Initialize Serial object
String buff = ""; // Create a serial buffer
int[] depths = new int[64]; // Create a list to parse serial into 

// Mesh Generation Variables
int cols = 8; // Sensor is 8x8
int rows = 8;
//int scale = 100; // Scale value for drawing our mesh
float[][] terrain = new float[8][8]; // Create a table of distance values by pixel location

// Cursor Tracking Variables
float xPress = 0.0; // Store the cursor x position on mouse press 
float yPress = 0.0; // Store the cursor y position on mouse press

// Mesh Rotation Variables
float xRotOffset = 0; // Temporary x rotational offset (relevant during mouse drag)
float xRotPos = 0; // X rotational position
float yRotOffset = 0; // Temporary y rotational offset (relevant during mouse drag)
float yRotPos = 0; // Y rotational position
float zRotOffset = 0; // Temporary z rotational offset (relevant during mouse drag)
float zRotPos = 0; // Z rotational position
// Mesh Zoom Variables
float scaleOffset = 0.3; // Scale factor from mouse wheel

  // sensor variables
float horFOVdeg = 45; // horizontal FOV in degrees
float verFOVdeg = 45; // horizontal FOV in degrees
int horRes = 8; // horizontal resolution
int verRes = 8; // vertical resolution
float horOFFSETdeg = 0; // horizontal offset angle in degrees
float verOFFSETdeg = 45; // vertical offset angle in degrees
float zOFFSETmm = 320; // Z height fo sensor
float[] coefX = new float[8];
float[] coefY = new float[8];
float[] coefZ = new float[8];
//point3D[] cartesianConv = new point3D[8];
//point3D[] cloud = new point3D[64]; // Create a table of point cloud in 3D
float[] pointX = new float[64];
float[] pointY = new float[64];
float[] pointZ = new float[64];

//public class point3D {
//      public float x;
//      public float y;
//      public float z;
//      //public float radius;
//}

void setup(){
  String[] serials = Serial.list();
  size(1000,1000, P3D); // Make a 3D capable window //<>//
  printArray(serials);
  if(serials.length>0)
  {
    for (int i = 0; i < serials.length; i = i+1)
    {
      if(serials[i].equals("COM9"))
      {
        port = new Serial(this, "COM9", 115200); // CHANGE COM13 TO YOUR SERIAL PORT //<>//
        port.clear();
      }
    }
  }
  //port.bufferUntil(10); // ASCII LineFeed Character
  
  // Fill our list with 0s to avoid a null pointer exception
  // in case the sensor data is not immediately available
  for(int idx = 0; idx < 64; idx++){
    depths[idx] = 1850; 
  }
 
  // compute cartesian coefficients from sensor parameters
  float angle; // local variable
  // radians !
  float horAngRes = radians(horFOVdeg / (float)horRes);
  float verAngRes = radians(verFOVdeg / (float)verRes);
  float horAngZero = radians((horFOVdeg / 2) + horOFFSETdeg);
  float verAngZero = radians((verFOVdeg / 2) + verOFFSETdeg);

  // compute Horizontal => X and Y
  for (int i = 0; i < horRes; i++)
  {
      angle = (horAngRes * i) - horAngZero;
      coefX[i] = sin(angle);
      coefY[i] = cos(angle);
  }
  // compute Vertical => Z
  for (int i = 0; i < verRes; i++)
  {
      angle = verAngZero - (verAngRes * i);
      coefZ[i] = sin(angle);
  }
}

void draw() // main loop
{
  // data acquisition here is faster than interrupt
  if(port != null)
  {
    buff = (port.readStringUntil(10)); // read the whole line
    if(buff!=null)
    {
      buff = buff.substring(0, buff.length()-1); // remove the Carriage Return
      if (buff != "") {
        //println(buff);
        depths = int(split(buff, ',')); // split at commas into our array
      }
    }
  }
  
  colorMode(HSB); // HSB color space makes it easy to map hue
  lights(); // Add ambient light to scene
  noStroke(); // Draw without stroke
  smooth(); // Draw with anti-aliasing
  background(0,0,100); // Fill background //<>//
  
//size(400,400,P3D);
//translate(232, 192, 0); 
//rotateY(0.5);
//noFill();

  
textSize(32);
float rotx = 0-(xRotOffset+xRotPos)+PI/2;
float roty = 0+(yRotOffset+yRotPos)+PI/4;
float rotz = 0-zRotOffset-zRotPos-PI/2;
text("Rot X : " + degrees(rotx), 40, 40);
text("Rot Y : " + degrees(roty), 40, 80);
text("Rot Z : " + degrees(rotz), 40, 120);
text("Zoom : " + scaleOffset, 40, 160);

  // This stuff is all basically to scale and rotate the mesh
  // while keeping it in the center of the scene
  translate(width/4,height/4);
  rotateX(rotx);
  rotateY(roty);
  rotateZ(rotz);
  scale(scaleOffset);

  //noFill();
  box(100);
  
  stroke(255);
  line(0, 0, 0, 100, 100, 100);
  line(0, 0, 0, 100, -100, 100);
  line(0, 0, 0, -100, 100, 100);
  line(0, 0, 0, -100, -100, 100);
  
  line(100, 100, 100, 100, -100, 100);
  line(100, -100, 100, -100, -100, 100);
  line(-100, -100, 100, -100, 100, 100);
  line(-100, 100, 100, 100, 100, 100);
  noStroke();

  // for all points
  if(depths.length >= 64){
    for(int i=0; i<64; i++)
    {
      // conversion to cartesian coordinates
      int col = i % 8;  // plan horizontal => curseur
      int row = i / 8;  // plan vertical => numéro de ligne
      pointX[i] = depths[i]*coefX[col]; //<>//
      pointY[i] = depths[i]*coefY[col];
      pointZ[i] = depths[i]*coefZ[row] + zOFFSETmm;
      //cloud[i].radius = 10;
      
      // displaying
      pushMatrix();
      translate(pointX[i], pointY[i], pointZ[i]);
      
      sphere(10);
      popMatrix();
    }
  }
  
  // For each row, draw a triangle strip with the z-height of
  // each vertex corresponding to the distance reading at that 
  // location. While we're at it, set the fill color to a hue
  // corresponding to the data as well.
  //for(int y=0; y<rows-1; y++)
  //{
  //  beginShape(TRIANGLE_STRIP);
  //  for(int x=0; x<cols; x++)
  //  {
  //    fill(map(terrain[x][y],100,600,0,255),255,255);
  //    vertex(x*scale,y*scale, terrain[x][y]/1);
  //    vertex(x*scale, (y+1)*scale, terrain[x][y+1]/1);
  //  }
  //  endShape();
  //}

} //<>//

// Wehn the mouse is pressed, remember where the cursor was
// so we can calculate how "far" it gets dragged
void mousePressed() {
  xPress = mouseX;
  yPress = mouseY;
}

// Between mouse drag events, draw will still be running, so we need to 
// update this offset at every event.
void mouseDragged() {
  if(mouseButton == LEFT)
  {
    xRotOffset = (mouseY-yPress)/100;
    zRotOffset = (mouseX-xPress)/100;
  }
  if(mouseButton == RIGHT)
  {
    xRotOffset = (mouseY-yPress)/100;
    yRotOffset = (mouseX-xPress)/100;
  }
}

// To prevent the mesh position "snapping back" after releasing the mouse button
// We add the new temporary offset to the position and clear the temporary offset
// for the next mouse drag
void mouseReleased() {
  xRotPos += xRotOffset;
  xRotOffset = 0;
  yRotPos += yRotOffset;
  yRotOffset = 0;
  zRotPos += zRotOffset;
  zRotOffset = 0;
}

// Whenever the mousewheel is scrolled, we adjust the scale offset of the mesh
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scaleOffset += e/10;
}
