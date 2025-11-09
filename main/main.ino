#include <Wire.h>                       // I2C
#include <esp_task_wdt.h>               // Watch Dog Timer
#include <SparkFun_VL53L5CX_Library.h>  // TOF lib
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
#define repoll_attempts 3     // How many time it tries to re-poll data

// Debugging
#define I2C_DEBUG false
#define PUTTY_DEBUG false
#define CLEAR_SCREEN() Serial.write("\033[2J\033[H")


/*****************************|
      Physical Parameter
|*****************************/
unsigned long state_start           = 0;      // ms
const unsigned long TIMEOUT         = 2000;   // ms
const unsigned long debounce_thresh = 5;      // ms
const unsigned long clear_thresh    = 250;    // ms
const unsigned long measure_thresh  = 10;     // ms
#define dist_threshold 600                    // mm
#define active_threshold 12                   // How many zones required to trigger a detection 
#define sbs_active_threshold 2                // Side-By-Side zone threshold

bool ptr = true;

// Detection booleans
bool LOUT, LIN, ROUT, RIN, LOUTS, LINS, ROUTS, RINS, OUT, IN = false;
bool _LOUT, _LIN, _ROUT, _RIN, _LOUTS, _LINS, _ROUTS, _RINS, _OUT, _IN = false;


// State Machine
enum CrossState {
  IDLE,     // 0
  ENTER_P,  // 1
  EXIT_P,   // 2
  CLEAR     // 3
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

int counter = 0;  // Primary In/Out counter

//16.5cm
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

bool meetsThresh(int i, VL53L5CX_ResultsData &buf)
{
  return buf.distance_mm[i] > 0 && buf.distance_mm[i] <= dist_threshold;
}

void countCells()
{
  for (int i = 0; i < 8; i++) {
    cell_counts[i] = 0;
    cell_active[i] = false;
  }
  

  for(int i=0; i < image_resolution; i++){  // Loop all 64 cells
    int col = i % image_width;              // Calculates current column
    int row = i / 8;                        // Calculates curren row
    
    // Determine Left Cell Type
    CellType cell_l;
    CellType cell_r;
    /*if(col >= 0 && col <= 0 && row >= 0 && row <= 3) {
      cell_l = L_INS;
    } else if (col >= 0 && col <= 0 && row >= 3 && row <= 7){
      cell_l = L_OUTS;
    } else if (col >= 1 && col <= 7 && row >= 0 && row <= 3){
      cell_l = L_INB;
    } else if (col >= 1 && col <= 7 && row >= 3 && row <= 7){
      cell_l = L_OUTB;
    }
    
    // Determine Left Cell Type
    if(col >= 0 && col <= 6 && row >= 0 && row <= 3) {
      cell_r = R_INB;
    } else if (col >= 0 && col <= 6 && row >= 3 && row <= 7){
      cell_r = R_OUTB;
    } else if (col >= 6 && col <= 7 && row >= 0 && row <= 3){
      cell_r = R_INS;
    } else if (col >= 6 && col <= 7 && row >= 3 && row <= 7){
      cell_r = R_OUTS;
    }*/

  
    if(col >= 0 && col <= 1 && row >= 0 && row <= 3) {
      cell_l = L_INS;
    } else if (col >= 0 && col <= 1 && row >= 3 && row <= 7){
      cell_l = L_OUTS;
    } else if (col >= 2 && col <= 7 && row >= 0 && row <= 3){
      cell_l = L_INB;
    } else if (col >= 2 && col <= 7 && row >= 3 && row <= 7){
      cell_l = L_OUTB;
    }
    
    // Determine Right Cell Type
    if(col >= 0 && col <= 5 && row >= 0 && row <= 3) {
      cell_r = R_INB;
    } else if (col >= 0 && col <= 5 && row >= 3 && row <= 7){
      cell_r = R_OUTB;
    } else if (col >= 5 && col <= 7 && row >= 0 && row <= 3){
      cell_r = R_INS;
    } else if (col >= 5 && col <= 7 && row >= 3 && row <= 7){
      cell_r = R_OUTS;
    }
    
    /*
    // Determine Left 4x4 Cell Type
    if(col >= 0 && col <= 1 && row >= 0 && row <= 3) {
      cell_l = L_INS;
    } else if (col >= 0 && col <= 1 && row >= 3 && row <= 7){
      cell_l = L_OUTS;
    } else if (col >= 2 && col <= 7 && row >= 0 && row <= 3){
      cell_l = L_INB;
    } else if (col >= 2 && col <= 7 && row >= 3 && row <= 7){
      cell_l = L_OUTB;
    }
    
    // Determine Right 4x4 Cell Type
    if(col >= 0 && col <= 5 && row >= 0 && row <= 3) {
      cell_r = R_INB;
    } else if (col >= 0 && col <= 5 && row >= 3 && row <= 7){
      cell_r = R_OUTB;
    } else if (col >= 5 && col <= 7 && row >= 0 && row <= 3){
      cell_r = R_INS;
    } else if (col >= 5 && col <= 7 && row >= 3 && row <= 7){
      cell_r = R_OUTS;
    }*/
  

    // Update respective counts
    if(meetsThresh(i, datal)) cell_counts[cell_l] += 1;
    if(meetsThresh(i, datar)) cell_counts[cell_r] += 1;
  }

  // Update active cells
  for(int i=0; i<8; i++){
    cell_active[i] = cell_counts[i] >= active_threshold;
    if(i == L_INS || i == L_OUTS || i == R_INS || i == R_OUTS) {
      cell_active[i] = cell_counts[i] >= sbs_active_threshold;
    }
  }

  // Boolean update
  LOUTS= cell_active[L_OUTS];
  LINS = cell_active[L_INS];
  ROUTS= cell_active[R_OUTS];
  RINS = cell_active[R_INS];
  LOUT = cell_active[L_OUTB] || cell_active[L_OUTS];
  LIN  = cell_active[L_INB]  || cell_active[L_INS];
  ROUT = cell_active[R_OUTB] || cell_active[R_OUTS];
  RIN  = cell_active[R_INB]  || cell_active[R_INS];
  OUT  = LOUT || ROUT;
  IN   = LIN  || RIN;

  if(ptr) {
  Serial.printf("Inner: %-4d %-4d %-4d %-4d\n\r", cell_active[L_INS], cell_active[L_INB], cell_active[R_INB], cell_active[R_INS]);
  Serial.printf("Outer: %-4d %-4d %-4d %-4d\n\r", cell_active[L_OUTS], cell_active[L_OUTB], cell_active[R_OUTB], cell_active[R_OUTS]);
  ptr = false;
  }
}

bool isTimeout(){
  return millis() - state_start > TIMEOUT;
}

bool isDebounce(){
  return true;//millis() - state_start > debounce_thresh;
}

bool isClear(){
  return millis() - state_start > clear_thresh;
}


bool pollBothSensors() {
  for (int i = 0; i < repoll_attempts; ++i) {
    if (pollSensor(tofl, datal) && pollSensor(tofr, datar))
      return true;
    delay(2);
  }
  return false;
}


bool delayedMeasure() {

  /*****************************|
        First Poll
  |*****************************/
  // bool bleft  = pollSensor(tofl, datal);
  // bool bright = pollSensor(tofr, datar);
  // if(!bleft || !bright) {return false;}
  pollBothSensors();

  unsigned long mes_start = millis();
  countCells();
  _LOUT=LOUT;
  _LIN=LIN;
  _ROUT=ROUT;
  _RIN=RIN;
  _LINS=LINS;
  _LOUTS=LOUTS;
  _RINS=RINS;
  _ROUTS=ROUTS;
  _OUT=OUT;
  _IN=IN;
  //digitalWrite(LED_PIN, 1);
  while(!metTimeThresh(mes_start, measure_thresh));

  /*****************************|
        Second Poll
  |*****************************/
  // bleft  = pollSensor(tofl, datal);
  // bright = pollSensor(tofr, datar);
  //digitalWrite(LED_PIN, 0);
  // if(!bleft || !bright) {return false;}
  pollBothSensors();
  
  countCells();
  LOUT  |= _LOUT;
  LIN   |= _LIN;
  ROUT  |= _ROUT;
  RIN   |= _RIN;
  OUT   |= _OUT;
  IN    |= _IN;
  LOUTS |= _LOUTS;
  ROUTS |= _ROUTS;
  LINS  |= _LINS;
  RINS  |= _RINS;

  ptr = true;
  return true;
}

bool metTimeThresh(unsigned long ref, unsigned long thresh) {return millis() - ref > thresh;}

int dbl = 0;
int dblp = 0;

void loop()
{ 
  //delay(50);
  if(PUTTY_DEBUG)CLEAR_SCREEN();
  /// Delayed Measurements ///
  if(!delayedMeasure()) return;
  
  switch(state){
    /*****************************|
          Default IDLE state
    |*****************************/
    case IDLE:
      state = IN && OUT ? IDLE: OUT ? ENTER_P : IN ? EXIT_P : IDLE;
      dblp = (LINS && RINS) || (LOUTS && ROUTS);
      if(dblp){
        digitalWrite(LED_PIN, 1);
      } else {
        digitalWrite(LED_PIN, 0);
      }
      state_start = millis();
      break;

    /*****************************|
            Enter Pending
    |*****************************/
    case ENTER_P:
      if(IN){// && isDebounce()) {
        ++counter;
        state = CLEAR;
        
        if(dblp && (LINS || RINS)){
          ++counter;
          dbl = 1;
        }
        state_start = millis();
      }
      if(isTimeout()) state = IDLE;
      break;

    /*****************************|
            Exit Pending
    |*****************************/
    case EXIT_P:
      if(OUT){// && isDebounce()) {
        --counter;
        state = CLEAR;
        
        if(dblp && (LOUTS || ROUTS)){
          --counter;
          dbl = 1;
        }
        state_start = millis();
      } 
      
      if(isTimeout()) state = IDLE;
      break;

    /*****************************|
          Clear Waiting Area
    |*****************************/
    case CLEAR:
      if(isClear() || isTimeout()){
        state = IDLE;
        dbl = 0;
        dblp = 0;
      }
      break;
  }

  Serial.printf("Count: %-4d \t|\t State: %4d \t|\t Double: %4d \t|\t Pending: %4d\n\r", counter, state, dbl, dblp);
}
