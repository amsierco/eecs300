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
#define LED_PIN 2

// Debugging
#define I2C_DEBUG false
#define PUTTY_DEBUG false
#define CLEAR_SCREEN() Serial.write("\033[2J\033[H")

unsigned long state_start = 0;
const unsigned long TIMEOUT = 1000; // ms
unsigned long state_end = 0;
unsigned long state_end2 = 0;

// State Machine
enum CrossState {
  IDLE,
  ENTER_P,
  EXIT_P,
  CLEAR
};
CrossState state = IDLE;

enum CellType {
  L_OUTB,   // Large outer cell
  L_INB,    // Large inner cell
  L_OUTS,   // Small outer cell
  L_INS,    // Small inner cell
  
  R_OUTB,   // Large outer cell
  R_INB,    // Large inner cell
  R_OUTS,   // Small outer cell
  R_INS     // Small inner cell
};

int  cell_counts[8];
bool cell_active[8];

// Left TOF Sensor
SparkFun_VL53L5CX tofl;
VL53L5CX_ResultsData datal;

// Right TOF Sensor
SparkFun_VL53L5CX tofr;
VL53L5CX_ResultsData datar;

// Software Defines & Globals
int counter = 0;              // Primary In/Out counter
#define dist_threshold 500    // mm
#define active_threshold 3    // How many zones required to trigger a detection 

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

  state = IDLE;

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

bool meetsThresh(int i, VL53L5CX_ResultsData &buf)
{
  return buf.distance_mm[i] > 0 && buf.distance_mm[i] <= dist_threshold;
}

void countCells(bool bleft, bool bright)
{
  for (int i = 0; i < 8; i++) {
    cell_counts[i] = 0;
    cell_active[i] = false;
  }
  

  for(int i=0; i < image_resolution; i++){ // Loop all 64 cells
    int col = i % image_width; // Calculates current column
    int row = i / 8;           // Calculates curren row
    
    // Determine Left Cell Type
    CellType cell_l;
    CellType cell_r;
    if(col >= 0 && col <= 1 && row >= 0 && row <= 3) {
      cell_l = L_INS;
    } else if (col >= 0 && col <= 1 && row >= 3 && row <= 7){
      cell_l = L_OUTS;
    } else if (col >= 2 && col <= 7 && row >= 0 && row <= 3){
      cell_l = L_INB;
    } else if (col >= 2 && col <= 7 && row >= 3 && row <= 7){
      cell_l = L_OUTB;
    }
    
    // Determine Left Cell Type
    if(col >= 0 && col <= 5 && row >= 0 && row <= 3) {
      cell_r = R_INB;
    } else if (col >= 0 && col <= 5 && row >= 3 && row <= 7){
      cell_r = R_OUTB;
    } else if (col >= 5 && col <= 7 && row >= 0 && row <= 3){
      cell_r = R_INS;
    } else if (col >= 5 && col <= 7 && row >= 3 && row <= 7){
      cell_r = R_OUTS;
    }

    // Update respective counts
    if(meetsThresh(i, datal) && bleft) cell_counts[cell_l] += 1;
    if(meetsThresh(i, datar) && bright) cell_counts[cell_r] += 1;
  }

  // Update active cells
  for(int i=0; i<8; i++){
    cell_active[i] = cell_counts[i] >= active_threshold;
  }

  Serial.printf("Inner: %-4d %-4d %-4d %-4d\n\r", cell_active[L_INS], cell_active[L_INB], cell_active[R_INB], cell_active[R_INS]);
  Serial.printf("Outer: %-4d %-4d %-4d %-4d\n\r", cell_active[L_OUTS], cell_active[L_OUTB], cell_active[R_OUTB], cell_active[R_OUTS]);

  /*
  Serial.println("Data \t|\t Left TOF Readings \t|\t Right TOF Readings");
  Serial.printf("Count \t|\t %-15d \t|\t %-15d\n", int(left_count), int(right_count));
  Serial.printf("Detect \t|\t %-15d \t|\t %-15d\n", int(left_detect), int(right_detect));
  */
}

bool isTimeout(){
  return millis() - state_start > TIMEOUT;
}

void loop()
{
  if(PUTTY_DEBUG)CLEAR_SCREEN();
  
  // Poll both sensors for new data
  bool bleft = pollSensor(tofl, datal);
  delay(10);
  bool bright = pollSensor(tofr, datar);
  if(!bleft && !bright) return;

  // Count cells
  countCells(bleft, bright);
  
  // FSM state merging
  bool LOUT = cell_active[L_OUTB] || cell_active[L_OUTS];
  bool LIN  = cell_active[L_INB]  || cell_active[L_INS];
  bool ROUT = cell_active[R_OUTB] || cell_active[R_OUTS];
  bool RIN  = cell_active[R_INB]  || cell_active[R_INS];
  bool OUT  = LOUT || ROUT;
  bool IN   = LIN  || RIN;

  const unsigned long debounce_thresh = 100; //ms
  const unsigned long clear_thresh = 1500;   //ms

  switch(state){
    case IDLE:
      digitalWrite(LED_PIN, LOW);
      //state = OUT ? ENTER_P : IN ? EXIT_P : IDLE;
      if(OUT) {
        state = ENTER_P;
      } else if (IN) {
        state = EXIT_P;
      } else {
        state = IDLE;
      }
      state_start = millis();
      break;

    case ENTER_P:
      digitalWrite(LED_PIN, HIGH);
      if(IN && (millis() - state_start >= debounce_thresh) ) {
        ++counter;
        state = CLEAR;
        state_end = millis();
      }
      if(isTimeout()) state = IDLE;
      break;

    case EXIT_P:
      if(OUT && (millis() - state_start >= debounce_thresh) ) {
        --counter;
        state = CLEAR;
        state_end2 = millis();
      } 
      if(isTimeout()) state = IDLE;
      break;

    case CLEAR:
      if(!IN && !OUT && millis() - state_start >= clear_thresh || isTimeout()){
        state = IDLE;
        digitalWrite(LED_PIN, LOW);
      }
      break;
  }
  Serial.printf("Count: %-4d \t|\t State: %04d\n\n\r", counter, state);
}
