/*
  Dual VL53L5CX 3D Visualization (Configurable)
  Supports both 8x8 (64 zones) and 4x4 (16 zones) modes.
  
  Make sure this matches your Arduino configuration!
*/

import processing.serial.*;

// --- CONFIGURATION ---
boolean IS_8x8 = false; // Set to 'false' for 4x4 mode
String SERIAL_PORT = "COM4"; // Change to your port

// --- GLOBAL VARIABLES ---
Serial port; 
String buff = ""; 

// Dynamic variables based on configuration
int matrixWidth;    // 8 or 4
int totalPixels;    // 64 or 16 per sensor
int packetSize;     // 128 or 32 total
int meshScale;      // Scale factor for drawing

int[] allDepths; 
float[][] terrainLeft; 
float[][] terrainRight; 

// Camera Control
float xPress = 0.0; 
float yPress = 0.0;
float xRotOffset = 0; 
float xRotPos = 0; 
float zRotOffset = 0; 
float zRotPos = 0;
float scaleOffset = 0.6; 

void setup(){
  size(1000, 700, P3D); 
  
  // 1. Configure Dimensions based on mode
  if (IS_8x8) {
    matrixWidth = 8;
    totalPixels = 64;
    packetSize = 128; // 64 left + 64 right
    meshScale = 60;   // Smaller scale for higher resolution
  } else {
    matrixWidth = 4;
    totalPixels = 16;
    packetSize = 32;  // 16 left + 16 right
    meshScale = 120;  // Larger scale so 4x4 doesn't look tiny
  }
  
  // 2. Initialize Arrays
  allDepths = new int[packetSize];
  terrainLeft = new float[matrixWidth][matrixWidth];
  terrainRight = new float[matrixWidth][matrixWidth];
  
  // 3. Start Serial
  try {
    port = new Serial(this, SERIAL_PORT, 921600); 
    port.bufferUntil(10); // LF (Line Feed)
    println("Connected to " + SERIAL_PORT + " in " + (IS_8x8 ? "8x8" : "4x4") + " mode.");
  } catch (Exception e) {
    println("ERR: Could not open Serial port. Check USB and COM number.");
  }
}

void draw(){
  colorMode(HSB); 
  lights(); 
  noStroke(); 
  smooth(); 
  background(0); 
  
  // --- CAMERA TRANSFORM ---
  translate(width/2, height/2);
  rotateX(PI/3 - (xRotOffset + xRotPos));
  rotateZ(0 - zRotOffset - zRotPos);
  scale(scaleOffset);
  
  // Center the view
  translate(- (matrixWidth * meshScale), - (matrixWidth * meshScale) / 2); 

  // --- PARSE DATA INTO GRIDS ---
  // Guard against array index out of bounds if serial is noisy
  if(allDepths.length >= packetSize) {
    for(int y=0; y<matrixWidth; y++){
      for(int x=0; x<matrixWidth; x++){
          
          // Map Left Sensor
          int indexL = x + y * matrixWidth;
          if (indexL < totalPixels) {
             terrainLeft[x][y] = allDepths[indexL] / 10.0; 
          }
          
          // Map Right Sensor (Offset by totalPixels)
          int indexR = indexL + totalPixels;
          if (indexR < packetSize) {
             terrainRight[x][y] = allDepths[indexR] / 10.0;
          }
      }
    }
  }
  
  // --- DRAW LEFT MESH ---
  pushMatrix();
  translate(-meshScale * (matrixWidth/2.0 + 0.5), 0, 0); // Shift Left
  drawMesh(terrainLeft);
  popMatrix();

  // --- DRAW RIGHT MESH ---
  pushMatrix();
  translate(meshScale * (matrixWidth/2.0 + 0.5), 0, 0); // Shift Right
  drawMesh(terrainRight);
  popMatrix();
}

// Helper function to draw a single mesh
void drawMesh(float[][] terrain) {
  for(int y=0; y<matrixWidth-1; y++){
    beginShape(TRIANGLE_STRIP);
    for(int x=0; x<matrixWidth; x++){
      
      float zVal = terrain[x][y];
      float zValNext = terrain[x][y+1];
      
      // Filter noise/outliers for cleaner view
      if (zVal > 300) zVal = 300; 
      if (zValNext > 300) zValNext = 300;
      
      // Color map: Near = Red (0), Far = Blue/Green (150)
      fill(map(zVal, 0, 200, 0, 150), 255, 255);
      
      vertex(x*meshScale, y*meshScale, zVal);
      vertex(x*meshScale, (y+1)*meshScale, zValNext);
    }
    endShape();
  }
}

// --- SERIAL EVENT ---
void serialEvent(Serial p){ 
  try {
    buff = p.readStringUntil('\n');
    if (buff != null) {
      buff = trim(buff); 
      int[] incoming = int(split(buff, ','));
      
      // Only update if we got the expected packet size
      if (incoming.length == packetSize) {
        allDepths = incoming;
      }
    }
  } catch (Exception e) {
    println("Serial parse error");
  }
}

// --- MOUSE CONTROLS ---
void mousePressed() { xPress = mouseX; yPress = mouseY; }
void mouseDragged() { xRotOffset = (mouseY-yPress)/100; zRotOffset = (mouseX-xPress)/100; }
void mouseReleased() { xRotPos += xRotOffset; xRotOffset = 0; zRotPos += zRotOffset; zRotOffset = 0; }
void mouseWheel(MouseEvent event) { scaleOffset += event.getCount()/10.0; }