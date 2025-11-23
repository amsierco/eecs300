#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> // TOF lib
#include <i2cdetect.h>

// Hardware Defines & Globals
const int fbfm = 1; // 1 = 4x4, 0 = 8x8
const int image_resolution = fbfm ? 16 : 64;  // TOF image resolution
const int image_width = fbfm ? 4 : 8;     // Row of TOF image resolution

#define tofl_rst 14      // Left TOF rst pin
#define tofr_rst 13      // Right TOF rst pin
#define tofl_addr 0x44    // Left TOF addr
#define tofr_addr 0x46    // Right TOF addr
#define tof_addr_default 0x29 // Default
#define I2C_freq 400000    // I2C bus freq (1 MHz)
#define ranging_freq 15    // Ranging freq
#define LED_PIN 2
#define repoll_attempts 3   // How many time it tries to re-poll data
#define repoll 1
#define dbg_distance 1

// Debugging
#define I2C_DEBUG false

// Left TOF Sensor
SparkFun_VL53L5CX tofl;
VL53L5CX_ResultsData datal;

// Right TOF Sensor
SparkFun_VL53L5CX tofr;
VL53L5CX_ResultsData datar;

void setup()
{

 Serial.begin(921600);

 pinMode(LED_PIN, OUTPUT);

 delay(1000);
 Serial.println("- - - - - - - - - - Initializing - - - - - - - - - -");

 Wire.begin(); // Init I2C Bus
 Wire.setClock(I2C_freq);

 pinMode(tofl_rst, OUTPUT);
 pinMode(tofr_rst, OUTPUT);
 digitalWrite(tofl_rst, HIGH);
 digitalWrite(tofr_rst, HIGH);
 delay(1000);
 digitalWrite(tofl_rst, LOW);
 delay(1000);

 if(I2C_DEBUG){i2cdetect();delay(2000);}

 // Initialize Left TOF
 Serial.println(("Initializing Left TOF"));
 if (!tofl.begin()) {
  Serial.print("Trying aux address: 0x");
  Serial.println(tofl_addr, HEX);
  if(!tofl.begin(tofl_addr)){
   Serial.println("Left TOF: Not Found");
   while(1);
  }
 }
 Serial.println("Left TOF Found!");

 // Change Left TOF Addr
 if (tofl.setAddress(tofl_addr) == false) {
  Serial.println(("Left TOF: Failed to change addr")); 
  while(1);
 } else if (tofl.getAddress() == tof_addr_default) {
  Serial.print(("Setting Left TOF address to: 0x"));
  Serial.println(tofl.getAddress(), HEX);
  delay(1000);
 }

 if(I2C_DEBUG){i2cdetect();delay(2000);}

 // Initialize Right TOF
 digitalWrite(tofr_rst, LOW); //Release right TOF from reset
 delay(1000);
 if(I2C_DEBUG){i2cdetect();delay(2000);}
 Serial.println("Initializing Right TOF");
 if (!tofr.begin()) {
  Serial.print("Trying aux address: 0x");
  Serial.println(tofr_addr, HEX);
  if(!tofr.begin(tofr_addr)){
   Serial.println("Right TOF: Not Found");
   while(1);
  }
 }
 Serial.println("Right TOF Found!");

 if(I2C_DEBUG){i2cdetect();delay(2000);}

 //Configure both sensors the same with 64 slots
 tofl.setResolution(image_resolution); 
 tofr.setResolution(image_resolution); 

 tofl.setRangingFrequency(ranging_freq);
 tofr.setRangingFrequency(ranging_freq);

 tofl.startRanging();
 tofr.startRanging();


 Serial.println("- - - - - - - - - - Initialization complete, starting counter - - - - - - - - - - ");
}

void loop() {
  // Poll both sensors
  if (tofl.isDataReady() && tofr.isDataReady()) {
    if (tofl.getRangingData(&datal) && tofr.getRangingData(&datar)) {
      
      // 1. PRINT LEFT SENSOR (0-63)
      for (int i = 0; i < image_resolution; i++) {
        Serial.print(datal.distance_mm[i]);
        Serial.print(",");
      }

      // 2. PRINT RIGHT SENSOR (0-63)
      for (int i = 0; i < image_resolution; i++) {
        Serial.print(datar.distance_mm[i]);
        // Only print comma if it's not the very last number
        if (i < image_resolution - 1) Serial.print(",");
      }

      // 3. NEWLINE (End of packet)
      Serial.println();
    }
  }
  delay(5);
}