
import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
 
Serial myPort;
 
String data="";
float roll, pitch;
int count;
 
void setup() {
  size (960, 640, P3D);
  myPort = new Serial(this, "COM17", 115200); // starts the serial communication
  myPort.bufferUntil('\n');
}
 
//void draw() {
//  translate(width/2, height/2, 0);
//  background(33);
//  textSize(22);
//  text("Roll: " + int(roll) + "     Pitch: " + int(pitch), -100, 265);
 
//  // Rotate the object
//  rotateX(radians(roll));
//  rotateZ(radians(-pitch));
  
//  // 3D 0bject
//  textSize(30);  
//  fill(255, 0, 0);
//  box (200, 10, 300); // Draw box
//  textSize(45);
//  fill(0, 255, 255);
//}

void draw() {
  background(33);


  float squareSize = 300;
  float x = width / 2;  
  float y = height / 2;
  
  rectMode(CENTER);
  stroke(33);
  strokeWeight(2);

  textAlign(CENTER, CENTER);
  textSize(100);
  
  if (count == 1) {
    fill(0, 230, 0);
    rect(x, y, squareSize, squareSize);
    
    fill(255);
    text("LOW", width / 2, height / 2 - 10);
  } 
  else if(count > 1 && count < 5) {
    fill(255, 200, 0);
    rect(x, y, squareSize, squareSize);
    
    fill(255);
    text("MID", width / 2, height / 2 - 10);
  }
  else if (count >= 5) {
    fill(255, 0, 0);
    rect(x, y, squareSize, squareSize);
    
    fill(255);
    text("HIGH", width / 2, height / 2 - 10);
  }
}
 
// Read data from the Serial Port
void serialEvent (Serial myPort) { 
  // reads the data from the Serial Port up to the character '.' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
 
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    // split the string at "/"
    String items[] = split(data, '/');
    if (items.length > 1) {
 
      //--- Roll,Pitch in degrees
      //roll = float(items[0]);
      //pitch = float(items[1]);
      count = int(items[0]);
    }
  }
}
