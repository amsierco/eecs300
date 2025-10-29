#include <Wire.h> // I2C
#include <SparkFun_VL53L5CX_Library.h> // TOF lib
#include <i2cdetect.h>

// Hardware Defines & Globals
#define tofl_rst 14           // Left TOF rst pin
#define tofr_rst 13           // Right TOF rst pin
#define tofl_addr 0x44        // Left TOF addr
#define tofr_addr 0x46        // Right TOF addr
#define tof_addr_default 0x29 // Default
#define I2C_freq 400000       // I2C bus freq (1 MHz)
#define image_resolution 64   // TOF image resolution
#define image_width 8         // Row of TOF image resolution
#define ranging_freq 15       // Ranging freq

// Left TOF Sensor
SparkFun_VL53L5CX tofl;
VL53L5CX_ResultsData datal;

// Right TOF Sensor
SparkFun_VL53L5CX tofr;
VL53L5CX_ResultsData datar;

// Software Defines & Globals
int counter = 0;              // Primary In/Out counter
#define dist_threshold 20     // mm
#define active_threshold 9    // How many zones required to trigger a detection 

void setup()
{
  Serial.begin(115200);
  
  delay(1000);
  Serial.println("- - - - - - - - - - Initializing - - - - - - - - - -");

  Wire.begin(); // Init I2C Bus
  Wire.setClock(I2C_freq);

  pinMode(tofl_rst, OUTPUT);
  pinMode(tofr_rst, OUTPUT);
  digitalWrite(tofl_rst, HIGH);
  digitalWrite(tofr_rst, HIGH);
  //i2cdetect();delay(2000);
  delay(1000);
  digitalWrite(tofl_rst, LOW);
  delay(1000);

  //i2cdetect();delay(2000);

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
  if (tofl.getAddress() == tof_addr_default && tofl.setAddress(tofl_addr) == false) {
    Serial.println(("Left TOF: Failed to change addr")); 
    while(1);
  } else if (tofl.getAddress() == tof_addr_default) {
    Serial.print(("Setting Left TOF address to: 0x"));
    Serial.println(tofl.getAddress(), HEX);
    delay(1000);
  }

  //i2cdetect();delay(2000);

  // Initialize Right TOF
  digitalWrite(tofr_rst, LOW); //Release right TOF from reset
  delay(1000);
  //i2cdetect();delay(2000);
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
  
  // Change Right TOF Addr
  // if (tofr.setAddress(tofr_addr) == false) {Serial.println(("Right TOF: Failed to change addr")); while(1); }
  // Serial.print(("Setting Right TOF address to: 0x"));
  // Serial.println(tofr.getAddress(), HEX);
  // delay(1000);

  //i2cdetect();delay(2000);

  //Configure both sensors the same with 64 slots
  tofl.setResolution(image_resolution); 
  tofr.setResolution(image_resolution); 

  tofl.setRangingFrequency(ranging_freq);
  tofr.setRangingFrequency(ranging_freq);

  tofl.startRanging();
  tofr.startRanging();

  Serial.println("- - - - - - - - - - Initialization complete, starting counter - - - - - - - - - - ");
}

bool pollSensor(SparkFun_VL53L5CX &sens, VL53L5CX_ResultsData &buf)
{
  if(!sens.isDataReady()) return false;
  if(sens.getRangingData(&buf)) return true;
  return false;
}

void debug() {
  if (tofl.isDataReady() == true)
  {
    if (tofl.getRangingData(&datal))
    {
      for (int y = 0 ; y <= image_width * (image_width - 1) ; y += image_width)
      {
        for (int x = image_width - 1 ; x >= 0 ; x--)
        {
          Serial.print("\t");
          Serial.print(datal.distance_mm[x + y]);
        }
        Serial.println();
      }
      Serial.println();
    }
  }
}

void loop()
{
  
  // Debugs Left TOF
  //debug();delay(50);return;

  // Poll both sensors for new data
  bool bleft = pollSensor(tofl, datal);
  delay(1000);
  bool bright = pollSensor(tofr, datar);
  if(!bleft || !bright) return;
  
  // Temp counters
  volatile int left_count = 0;
  volatile int right_count = 0;

  // Left TOF: Read appropriate zones
  if(bleft){
    for(int i=0; i < image_resolution; i++){
      int col = i % image_width;
      if(col >=0 && col <= 3){
        if(datal.distance_mm[i] > 0 && datal.distance_mm[i] < dist_threshold){
          left_count++;
        }
      }
    }
  }

  // Right TOF: Read appropriate zones
  if(bright){
    for(int i=0; i < image_resolution; i++){
      int col = i % image_width;
      if(col >=4 && col <= 7){
        if(datar.distance_mm[i] > 0 && datar.distance_mm[i] < dist_threshold){
          right_count++;
        }
      }
    }
  }

  bool left_detect  = left_count  >= active_threshold;
  bool right_detect = right_count >= active_threshold;
  // Debug Msgs
  Serial.println("Data \t|\t Left TOF Readings \t|\t Right TOF Readings");
  Serial.printf("Count \t|\t %-15d \t|\t %-15d\n", int(left_count), int(right_count));
  Serial.printf("Detect \t|\t %-15d \t|\t %-15d\n", int(left_detect), int(right_detect));

  delay(50); //Small delay between polling
}
/*#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX tofl;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int image_width = 0; //Used to pretty print output

int peopleCount = 0;
bool currentlyDetected = false; // detection state

// State machine
enum CrossState { IDLE, TOP_DETECTED, BOTTOM_DETECTED };
CrossState state = IDLE;

const int DISTANCE_THRESHOLD = 500; // mm
const int MIN_ACTIVE_ZONES = 5;     // how many zones must trigger to count as detection

const int LED_PIN = 2;

void setup()
{
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("VL53L5CX TOF w/ Counter");

  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(1000000); //Sensor has max I2C freq of 1MHz
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (tofl.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  tofl.setResolution(8*8); //Enable all 64 pads
  
  imageResolution = tofl.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  image_width = sqrt(imageResolution); //Calculate printing width

  tofl.setRangingFrequency(15); // Using 8x8, min frequency is 1Hz and max is 15Hz

  tofl.startRanging();
}

void loop()
{
  //Poll sensor for new data
  if (tofl.isDataReady() == true)
  {
    if (tofl.getRangingData(&measurementData))  //Read distance data into array
    {
      int topActive = 0;
      int bottomActive = 0;

      // Count how many zones are within threshold
      for (int i = 0; i < imageResolution; i++)
      {
        if (measurementData.distance_mm[i] > 0 && measurementData.distance_mm[i] <= DISTANCE_THRESHOLD)
        {
          if (i < 32)
            bottomActive++;
          else
            topActive++;
        }
      }

  // State machine transitions
      switch (state)
      {
        case IDLE:
          if (topActive >= MIN_ACTIVE_ZONES)
          {
            peopleCount++;
            Serial.print("Count incremented! Total = ");
            Serial.println(peopleCount);
            digitalWrite(LED_PIN, HIGH);
            state = TOP_DETECTED;
          }
          else if (bottomActive >= MIN_ACTIVE_ZONES)
          {
            peopleCount++;
            Serial.print("Count incremented! Total = ");
            Serial.println(peopleCount);
            digitalWrite(LED_PIN, HIGH);
            state = BOTTOM_DETECTED;
          }
          else
          { digitalWrite(LED_PIN, LOW);
            state = IDLE;
          }
          break;

        case TOP_DETECTED:
          if (topActive >= MIN_ACTIVE_ZONES)
          {
            digitalWrite(LED_PIN, HIGH);
            state = TOP_DETECTED;
          }
          else if (bottomActive >= MIN_ACTIVE_ZONES)
          {
            Serial.println("Person EXITED");
            digitalWrite(LED_PIN, HIGH);
            state = BOTTOM_DETECTED;
          }
          else
          { digitalWrite(LED_PIN, LOW);
            state = IDLE;
          }
          break;

        case BOTTOM_DETECTED:
          if (bottomActive >= MIN_ACTIVE_ZONES)
          {
            digitalWrite(LED_PIN, HIGH);
            state = BOTTOM_DETECTED;
          }
          else if (topActive >= MIN_ACTIVE_ZONES)
          {
            Serial.println("Person ENTERED");
            digitalWrite(LED_PIN, HIGH);
            state = TOP_DETECTED;
          }
          else
          { digitalWrite(LED_PIN, LOW);
            state = IDLE;
          }
          break;
      }
      
    }
  }
  delay(50); // adjust sampling rate
}*/