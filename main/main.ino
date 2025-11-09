#include <Wire.h>                // I2C 
#include <esp_task_wdt.h>        // Watch Dog Timer 
#include <SparkFun_VL53L5CX_Library.h> // TOF lib 
#include <i2cdetect.h> 

// Hardware Defines & Globals 
#define tofl_rst 14             // Left TOF rst pin 
#define tofr_rst 13             // Right TOF rst pin 
#define tofl_addr 0x44          // Left TOF addr 
#define tofr_addr 0x46          // Right TOF addr 
#define tof_addr_default 0x29   // Default 
#define I2C_freq 400000         // I2C bus freq (400kHz)
#define image_resolution 64     // TOF image resolution 
#define image_width 8           // Row of TOF image resolution 
#define ranging_freq 15         // Ranging freq 
#define LED_PIN 2 

// Debugging 
#define I2C_DEBUG false 
#define PUTTY_DEBUG false 
#define CLEAR_SCREEN() Serial.write("\033[2J\033[H") 

// --- MODIFIED --- (Added individual timers for each FSM)
unsigned long state_start_L = 0;       // ms 
unsigned long state_start_R = 0;       // ms 

const unsigned long TIMEOUT = 2000;         // ms 
const unsigned long debounce_thresh = 5;    // ms 
const unsigned long clear_thresh = 1500;    // ms 

// --- NEW --- (Add these to your global variables)
const unsigned long COOLDOWN_PERIOD = 500; // 500ms refractory period
unsigned long last_count_time = 0;         // Timer to track the last event

// --- NEW --- (Event type returned by each FSM)
enum EventType {
  NONE,
  ENTER,
  EXIT
};

// --- MODIFIED --- (Renamed from CrossState for clarity)
enum FsmState { 
  FSM_IDLE, 
  FSM_PENDING_ENTER, 
  FSM_PENDING_EXIT, 
  FSM_CLEAR 
}; 

// --- MODIFIED --- (One state for each FSM)
FsmState state_L = FSM_IDLE;
FsmState state_R = FSM_IDLE;

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
int counter = 0;                // Primary In/Out counter 
#define dist_threshold 900      // mm 

// --- MODIFIED --- (Replaced single thresholds with Hysteresis)
#define ON_THRESHOLD 4          // Hysteresis ON (Big Zones)
#define OFF_THRESHOLD 2         // Hysteresis OFF (Big Zones)
#define SBS_ON_THRESHOLD 3      // Hysteresis ON (Small Zones)
#define SBS_OFF_THRESHOLD 1     // Hysteresis OFF (Small Zones)

void setup() 
{ 
  Serial.begin(921600); 
     
  //watchdog timer with 5s period 
  //esp_task_wdt_init(5, true); // Enable watchdog (which will restart ESP32 if it hangs) 
  //esp_task_wdt_add(NULL);     // Add current thread to WDT watch 
   
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

  state_L = FSM_IDLE; // --- MODIFIED ---
  state_R = FSM_IDLE; // --- MODIFIED ---

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
    // cell_active[i] is NOT reset here, to support hysteresis
  } 
   

  for(int i=0; i < image_resolution; i++){ // Loop all 64 cells 
    int col = i % image_width; // Calculates current column 
    int row = i / 8;           // Calculates curren row 
     
    // Determine Left Cell Type 
    CellType cell_l; 
    CellType cell_r; 
    if(col >= 0 && col <= 0 && row >= 0 && row <= 3) { 
      cell_l = L_INS; 
    } else if (col >= 0 && col <= 0 && row >= 3 && row <= 7){ 
      cell_l = L_OUTS; 
    } else if (col >= 1 && col <= 7 && row >= 0 && row <= 3){ 
      cell_l = L_INB; 
    } else if (col >= 1 && col <= 7 && row >= 3 && row <= 7){ 
      cell_l = L_OUTB; 
    } 
     
    // Determine Right Cell Type
    if(col >= 0 && col <= 6 && row >= 0 && row <= 3) {
      cell_r = R_INB; 
    } else if (col >= 0 && col <= 6 && row >= 3 && row <= 7){ 
      cell_r = R_OUTB; 
    } else if (col >= 7 && col <= 7 && row >= 0 && row <= 3){ // --- MODIFIED --- (col >= 6 became col >= 7)
      cell_r = R_INS; 
    } else if (col >= 7 && col <= 7 && row >= 3 && row <= 7){ // --- MODIFIED --- (col >= 6 became col >= 7)
      cell_r = R_OUTS; 
    } 

    // Update respective counts 
    if(meetsThresh(i, datal) && bleft) cell_counts[cell_l] += 1; 
    if(meetsThresh(i, datar) && bright) cell_counts[cell_r] += 1; 
  } 

  // --- MODIFIED --- (This entire block is the new Hysteresis logic)
  // Update active cells
  for(int i=0; i<8; i++){
    // Determine thresholds based on zone type
    int on_thresh = (i == L_INS || i == L_OUTS || i == R_INS || i == R_OUTS) ? SBS_ON_THRESHOLD : ON_THRESHOLD;
    int off_thresh = (i == L_INS || i == L_OUTS || i == R_INS || i == R_OUTS) ? SBS_OFF_THRESHOLD : OFF_THRESHOLD;

    // Apply hysteresis logic
    if (cell_counts[i] >= on_thresh) {
      cell_active[i] = true;
    } else if (cell_counts[i] <= off_thresh) {
      cell_active[i] = false;
    }
    // else: cell_active[i] remains unchanged, preventing flicker
  } 
 
  Serial.printf("Inner: %-4d %-4d %-4d %-4d\n\r", cell_active[L_INS], cell_active[L_INB], cell_active[R_INB], cell_active[R_INS]); 
  Serial.printf("Outer: %-4d %-4d %-4d %-4d\n\r", cell_active[L_OUTS], cell_active[L_OUTB], cell_active[R_OUTB], cell_active[R_OUTS]); 
} 

// --- MODIFIED --- (Timer functions now take a parameter)
bool isTimeout(unsigned long start_time){ 
  return millis() - start_time > TIMEOUT; 
} 

bool isDebounce(unsigned long start_time){ 
  return millis() - start_time > debounce_thresh; 
} 

bool isClear(unsigned long start_time){ 
  return millis() - start_time > clear_thresh; 
} 

// --- NEW --- (The FSM logic is now its own re-usable function)
EventType update_fsm(FsmState &state, unsigned long &state_start, bool zone_OUT, bool zone_IN)
{
  EventType event = NONE; // Return value

  switch(state){
    case FSM_IDLE:
      if (zone_IN && zone_OUT) {
        state = FSM_IDLE; // Bi-dir or lingering case, stay idle
      } else if (zone_OUT) {
        state = FSM_PENDING_ENTER;
      } else if (zone_IN) {
        state = FSM_PENDING_EXIT;
      }
      
      if(state != FSM_IDLE) state_start = millis(); // Start timer on state change
      break;

    case FSM_PENDING_ENTER:
      if (zone_IN && isDebounce(state_start)) {
        event = ENTER; // Fire ENTER event!
        state = FSM_CLEAR;
        state_start = millis();
      }
      if (isTimeout(state_start)) state = FSM_IDLE; // Timeout
      break;

    case FSM_PENDING_EXIT:
      if (zone_OUT && isDebounce(state_start)) {
        event = EXIT; // Fire EXIT event!
        state = FSM_CLEAR;
        state_start = millis();
      }
      if (isTimeout(state_start)) state = FSM_IDLE; // Timeout
      break;

    case FSM_CLEAR:
      if ((!zone_IN && !zone_OUT && isClear(state_start)) || isTimeout(state_start)) {
        state = FSM_IDLE;
      }
      break;
  }
  return event;
}


void loop() 
{ 
  if(PUTTY_DEBUG)CLEAR_SCREEN(); 
   
  // Poll both sensors for new data 
  bool bleft = pollSensor(tofl, datal); 
  bool bright = pollSensor(tofr, datar); 
  if(!bleft || !bright) return; 

  // Count cells (with hysteresis) 
  countCells(bleft, bright); 
   
  // --- DUAL FSM INPUTS ---
  bool LOUT = cell_active[L_OUTB] || cell_active[L_OUTS];
  bool LIN  = cell_active[L_INB]  || cell_active[L_INS];
  bool ROUT = cell_active[R_OUTB] || cell_active[R_OUTS];
  bool RIN  = cell_active[R_INB]  || cell_active[R_INS];

  // --- RUN FSMs ---
  EventType event_L = update_fsm(state_L, state_start_L, LOUT, LIN);
  EventType event_R = update_fsm(state_R, state_start_R, ROUT, RIN);

  // --- COORDINATOR LOGIC (v3 - Fixes your diagnosed bugs) ---
  
  bool event_happened = false;

  // --- Refractory Period Check ---
  // If we are in the cooldown window, ignore all new events
  if (millis() - last_count_time < COOLDOWN_PERIOD) {
    // Do nothing, wait for cooldown to finish
  } 
  
  // --- CASE 1: BI-DIRECTIONAL (Net 0) ---
  else if (event_L == ENTER && event_R == EXIT) {
    counter++; 
    counter--; 
    event_happened = true;
  } 
  else if (event_L == EXIT && event_R == ENTER) {
    counter--; 
    counter++; 
    event_happened = true;
  }

  // Case 2: Same Direction (Enter)
  else if (event_L == ENTER && event_R == ENTER) {
    bool sbs_enter_confirmed = cell_active[L_INS] && cell_active[R_INS];
    if (sbs_enter_confirmed) {
      counter += 2; // Two people confirmed
    } else {
      counter++; // One person in the middle
    }
    event_happened = true;
  }

  // Case 3: Same Direction (Exit)
  else if (event_L == EXIT && event_R == EXIT) {
      bool sbs_exit_confirmed = cell_active[L_OUTS] && cell_active[R_OUTS];
      if (sbs_exit_confirmed) {
      counter -= 2; // Two people confirmed
    } else {
      counter--; // One person in the middle
    }
    event_happened = true;
  }
  
  // Case 4: Single Person (one side only)
  else if (event_L == ENTER || event_R == ENTER) {
    counter++;
    event_happened = true;
  }
  else if (event_L == EXIT || event_R == EXIT) {
    counter--;
    event_happened = true;
  }
  
  // --- End of Coordinator ---

  // --- Update Timer and LED ---
  if (event_happened) {
      last_count_time = millis(); // Start the refractory period
      digitalWrite(LED_PIN, HIGH); 
  } else if (state_L == FSM_IDLE && state_R == FSM_IDLE) {
      digitalWrite(LED_PIN, LOW); 
  }
  
  // Serial Print
  Serial.printf("Count: %-4d | L-State: %d | R-State: %d \n\r", counter, state_L, state_R);
  Serial.printf("L-Event: %-4d | R-Event: %-4d \n\n\r", event_L, event_R);
}