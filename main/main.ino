#include <Wire.h>                       // I2C
#include <esp_task_wdt.h>               // Watch Dog Timer
#include <SparkFun_VL53L5CX_Library.h>  // TOF lib
#include <i2cdetect.h>                  // I2C Debug
#include <WiFi.h>                       // WIFI
#include <WiFiClient.h>                 // WIFI
#include <WiFiAP.h>                     // WIFI

#define BIDIR_LED 19
#define CTR_INC 18
#define CTR_DEC 5
#define IDLE_LED 17

// Hardware Defines & Globals
const int  fbfm  = 1;
#define tofl_rst 14           // Left TOF rst pin
#define tofr_rst 13           // Right TOF rst pin
#define tofl_addr 0x44        // Left TOF addr
#define tofr_addr 0x46        // Right TOF addr
#define tof_addr_default 0x29 // Default
#define I2C_freq 400000       // I2C bus freq (1 MHz)
const int image_resolution = fbfm ? 16 : 64;   // TOF image resolution
const int image_width = fbfm ? 4 : 8;         // Row of TOF image resolution
#define ranging_freq 15       // Ranging freq
#define LED_PIN 2
#define repoll_attempts 3     // How many time it tries to re-poll data
#define repoll 1
#define dbg_distance 1

// Debugging
#define I2C_DEBUG false

/*****************************|
      Physical Parameter
|*****************************/
unsigned long state_start           = 0;      // ms
const unsigned long TIMEOUT         = 750;    // ms
const unsigned long clear_thresh    = 200;    // ms
const unsigned long measure_thresh  = 15;    // ms (20ms is promising)
unsigned long print_timer           = 0;
#define dist_threshold 1500                    // mm 
const int active_threshold = fbfm ? 2 : 4;                    // How many zones required to trigger a detection
const int sbs_active_threshold = fbfm ? 2 : 4;                // Side-By-Side zone threshold

bool ptr = true;

// Detection booleans
bool LOUT, LIN, ROUT, RIN, LOUTS, LINS, ROUTS, RINS, OUT, IN = false;
bool _LOUT, _LIN, _ROUT, _RIN, _LOUTS, _LINS, _ROUTS, _RINS, _OUT, _IN = false;
int cell_acc[8];


// State Machine
enum CrossState {
  IDLE,     // 0
  ENTER_P,  // 1
  EXIT_P,   // 2
  CLEAR,    // 3
  BIDIR_P   // 4
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
int dbl = 0;
int dblp = 0;

// Left TOF Sensor
SparkFun_VL53L5CX tofl;
VL53L5CX_ResultsData datal;

// Right TOF Sensor
SparkFun_VL53L5CX tofr;
VL53L5CX_ResultsData datar;

int counter = 0;  // Primary In/Out counter

////////////////////////////////

// Left Inner Baseline
const int l_in_b_c_min = fbfm ? 2 : 4;
const int l_in_b_c_max = fbfm ? 3 : 7;
const int l_in_b_r_min = fbfm ? 0 : 0;
const int l_in_b_r_max = fbfm ? 0 : 0;

// Left Outer Baseline
const int l_out_b_c_min = fbfm ? 2 : 4;
const int l_out_b_c_max = fbfm ? 3 : 7;
const int l_out_b_r_min = fbfm ? 3 : 7;
const int l_out_b_r_max = fbfm ? 3 : 7;

// Left Inner Edge Case
const int l_in_e_c_min = fbfm ? 0 : 0;
const int l_in_e_c_max = fbfm ? 1 : 1;
const int l_in_e_r_min = fbfm ? 0 : 0;
const int l_in_e_r_max = fbfm ? 0 : 0;

// Left Outer Edge Case
const int l_out_e_c_min = fbfm ? 0 : 0;
const int l_out_e_c_max = fbfm ? 1 : 1;
const int l_out_e_r_min = fbfm ? 3 : 7;
const int l_out_e_r_max = fbfm ? 3 : 7;

////////////////////////////////

// Right Inner Baseline
const int r_in_b_c_min = fbfm ? 0 : 4;
const int r_in_b_c_max = fbfm ? 1 : 7;
const int r_in_b_r_min = fbfm ? 0 : 7;
const int r_in_b_r_max = fbfm ? 0 : 7;

// Right Outer Baseline
const int r_out_b_c_min = fbfm ? 0 : 4;
const int r_out_b_c_max = fbfm ? 1 : 7;
const int r_out_b_r_min = fbfm ? 3 : 0;
const int r_out_b_r_max = fbfm ? 3 : 0;

// Right Inner Edge Case
const int r_in_e_c_min = fbfm ? 2 : 0;
const int r_in_e_c_max = fbfm ? 3 : 1;
const int r_in_e_r_min = fbfm ? 0 : 7;
const int r_in_e_r_max = fbfm ? 0 : 7;

// Right Outer Edge Case
const int r_out_e_c_min = fbfm ? 2 : 0;
const int r_out_e_c_max = fbfm ? 3 : 1;
const int r_out_e_r_min = fbfm ? 3 : 0;
const int r_out_e_r_max = fbfm ? 3 : 0;
////////////////////////////////

// WIFI
const char *ssid = "ESP32_GROUP2";
WiFiServer tcpServer(1234);

void wifi_config(){
  if (!WiFi.softAP(ssid)) {
    Serial.println("Failed to create wifi AP");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  tcpServer.begin();

  Serial.println("Server started");
}

void setup()
{
  for(int i=0; i<8; i++){ cell_acc[i] = 0; }

  Serial.begin(921600);

  wifi_config();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BIDIR_LED, OUTPUT);
  pinMode(CTR_INC, OUTPUT);
  pinMode(CTR_DEC, OUTPUT);
  pinMode(IDLE_LED, OUTPUT);

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

inline bool pollSensor(SparkFun_VL53L5CX &sens, VL53L5CX_ResultsData &buf)
{
  if(!sens.isDataReady()) return false;
  if(sens.getRangingData(&buf)) return true;
  return false;
}

inline bool meetsThresh(int i, VL53L5CX_ResultsData &buf)
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
    int row = i / image_width;                        // Calculates curren row
    
    CellType cell_l;
    CellType cell_r;
    // Determine Left 8x8 Cell Type w/o middle
    if        (col >= l_in_e_c_min && col <= l_in_e_c_max && row >= l_in_e_r_min && row <= l_in_e_r_max) {
      cell_l = L_INS;
    } else if (col >= l_out_e_c_min && col <= l_out_e_c_max && row >= l_out_e_r_min && row <= l_out_e_r_max){
      cell_l = L_OUTS;
    } else if (col >= l_in_b_c_min && col <= l_in_b_c_max && row >= l_in_b_r_min && row <= l_in_b_r_max){
      cell_l = L_INB;
    } else if (col >= l_out_b_c_min && col <= l_out_b_c_max && row >= l_out_b_r_min && row <= l_out_b_r_max){
      cell_l = L_OUTB;
    }
    
    // Determine Right 8x8 Cell  w/o middle
    if        (col >= r_in_b_c_min && col <= r_in_b_c_max && row >= r_in_b_r_min && row <= r_in_b_r_max) {
      cell_r = R_INB;
    } else if (col >= r_out_b_c_min && col <= r_out_b_c_max && row >= r_out_b_r_min && row <= r_out_b_r_max){
      cell_r = R_OUTB;
    } else if (col >= r_in_e_c_min && col <= r_in_e_c_max && row >= r_in_e_r_min && row <= r_in_e_r_max){
      cell_r = R_INS;
    } else if (col >= r_out_e_c_min && col <= r_out_e_c_max && row >= r_out_e_r_min && row <= r_out_e_r_max){
      cell_r = R_OUTS;
    }

    // Update respective counts
    if(meetsThresh(i, datal)) cell_counts[cell_l] += 1;
    if(meetsThresh(i, datar)) cell_counts[cell_r] += 1;

  }

  
  if(dbg_distance){
    if(millis() - print_timer > 500) {
    for (int i=0; i < image_width; ++i){
      
      printf("%-5d %-5d %-5d %-5d \t|\t  %-5d %-5d %-5d %-5d\n\r", 
        (int)(datal.distance_mm[(i*image_width)+0]/10),
        (int)(datal.distance_mm[(i*image_width)+1]/10),
        (int)(datal.distance_mm[(i*image_width)+2]/10),
        (int)(datal.distance_mm[(i*image_width)+3]/10),

        (int)(datar.distance_mm[(12-(i*image_width))+3]/10),
        (int)(datar.distance_mm[(12-(i*image_width))+2]/10),
        (int)(datar.distance_mm[(12-(i*image_width))+1]/10),
        (int)(datar.distance_mm[(12-(i*image_width))+0]/10));
    }
      printf("\n\n");
      print_timer = millis();
    }
  }

  /*
  if(millis() - print_timer > 500) {
  for (int i=0; i < 8; ++i){
    
    printf("%-5d %-5d %-5d %-5d %-5d %-5d %-5d %-5d \t|\t %-5d %-5d %-5d %-5d %-5d %-5d %-5d %-5d\n\r", 
      (int)(datal.distance_mm[(i*8)+0]/10),
      (int)(datal.distance_mm[(i*8)+1]/10),
      (int)(datal.distance_mm[(i*8)+2]/10),
      (int)(datal.distance_mm[(i*8)+3]/10),
      (int)(datal.distance_mm[(i*8)+4]/10),
      (int)(datal.distance_mm[(i*8)+5]/10),
      (int)(datal.distance_mm[(i*8)+6]/10),
      (int)(datal.distance_mm[(i*8)+7]/10),
      
      (int)(datar.distance_mm[(56-(i*8))+7]/10),
      (int)(datar.distance_mm[(56-(i*8))+6]/10),
      (int)(datar.distance_mm[(56-(i*8))+5]/10),
      (int)(datar.distance_mm[(56-(i*8))+4]/10),
      (int)(datar.distance_mm[(56-(i*8))+3]/10),
      (int)(datar.distance_mm[(56-(i*8))+2]/10),
      (int)(datar.distance_mm[(56-(i*8))+1]/10),
      (int)(datar.distance_mm[(56-(i*8))+0]/10));
  }
    printf("\n\n");
    print_timer = millis();
  }*/

  // Update active cells
  for(int i=0; i<8; i++){
    cell_active[i] = cell_counts[i] >= active_threshold;
    if(i == L_INS || i == L_OUTS || i == R_INS || i == R_OUTS) {
      cell_active[i] = cell_counts[i] >= sbs_active_threshold;
    }
  }

  //cell_active[R_INS] = false;

  // Boolean update
  LOUTS= cell_active[L_OUTS];
  LINS = cell_active[L_INS];
  ROUTS= cell_active[R_OUTS];
  RINS = cell_active[R_INS];
  LOUT = cell_active[L_OUTB];// || cell_active[L_OUTS];
  LIN  = cell_active[L_INB];//  || cell_active[L_INS];
  ROUT = cell_active[R_OUTB];// || cell_active[R_OUTS];
  RIN  = cell_active[R_INB];//  || cell_active[R_INS];
  OUT  = LOUT || ROUT;
  IN   = LIN  || RIN;

  // if(ptr) {
  // Serial.printf("Inner: %-4d %-4d %-4d %-4d\n\r", cell_active[L_INS], cell_active[L_INB], cell_active[R_INB], cell_active[R_INS]);
  // Serial.printf("Outer: %-4d %-4d %-4d %-4d\n\r", cell_active[L_OUTS], cell_active[L_OUTB], cell_active[R_OUTB], cell_active[R_OUTS]);
  // ptr = false;
  // }
}

inline bool isTimeout(){
  return millis() - state_start > TIMEOUT;
}

inline bool isClear(){
  return millis() - state_start > clear_thresh;
}

inline bool pollBothSensors() {
  pollSensor(tofl, datal);
  pollSensor(tofr, datar);
  return true;
  
  for (int i = 0; i < repoll_attempts; ++i) {
    if (pollSensor(tofl, datal) && pollSensor(tofr, datar))
      return true;
    delay(2);
  }
  return false;
}

bool pollSensors( WiFiClient client) {
/*****************************|
        TOF Connection Check
  |*****************************/
  if(!tofl.isConnected()) {
    printf("Left TOF Disconnected\n\r");
  }
  if(!tofr.isConnected()) {
    printf("Right TOF Disconnected\n\r");
  }
  if(!tofl.isConnected() || !tofr.isConnected()) {
    return false;
  }
  
  /*****************************|
          TOF Measurement
  |*****************************/
  for(int i=0; i<8; i++){ cell_acc[i] = 0; }
  for(int i=0; i<repoll; i++){
    pollBothSensors();
    countCells();

    cell_acc[L_OUTB] |= LOUT  ? 1 : 0;
    cell_acc[L_OUTS] |= LOUTS ? 1 : 0;
    cell_acc[L_INB]  |= LIN   ? 1 : 0;
    cell_acc[L_INS]  |= LINS  ? 1 : 0;
    cell_acc[R_OUTB] |= ROUT  ? 1 : 0;
    cell_acc[R_OUTS] |= ROUTS ? 1 : 0;
    cell_acc[R_INB]  |= RIN   ? 1 : 0;
    cell_acc[R_INS]  |= RINS  ? 1 : 0;

    delay(measure_thresh);
  }

  LOUT  = cell_acc[L_OUTB]  ? true : false;
  LIN   = cell_acc[L_INB]   ? true : false;
  ROUT  = cell_acc[R_OUTB]  ? true : false;
  RIN   = cell_acc[R_INB]   ? true : false;

  LOUTS = cell_acc[L_OUTS]  ? true : false;
  ROUTS = cell_acc[R_OUTS]  ? true : false;
  LINS  = cell_acc[L_INS]   ? true : false;
  RINS  = cell_acc[R_INS]   ? true : false;

  OUT   = LOUT || ROUT || LOUTS || ROUTS;
  IN    = LIN  || RIN  || LINS  || RINS;

  bool same_in, same_out = false;
  same_in = LINS && RINS;
  same_out = LOUTS && ROUTS;

  Serial.printf("Inner: %-4d %-4d %-4d %-4d | Dual Edges: %-4d \n\r", LINS, LIN, RIN, RINS, same_in);
  Serial.printf("Outer: %-4d %-4d %-4d %-4d | Dual Edges: %-4d \n\r", LOUTS, LOUT, ROUT, ROUTS, same_out);
  //client.println();
  
  return true;
}

void loop()
{ 
  WiFiClient client = tcpServer.available();
  
  if(client){
    while(client.connected()){
      // Read TOF Data
      pollSensors(client);
      // FSM 
      switch(state){
        /*****************************|
              Default IDLE state
        |*****************************/
        case IDLE:
          digitalWrite(IDLE_LED, HIGH);
          //if ((LINS && ROUTS) || (LOUTS && RINS)){  state = BIDIR_P;}
          if (IN && OUT){                      state = IDLE;}
          else if (OUT){                            state = ENTER_P;}
          else if (IN){                             state = EXIT_P;}
          else {                                    state = IDLE;}

          dblp = (LINS && RINS) || (LOUTS && ROUTS);
          state_start = millis();
          break;

        /*****************************|
            Bi-Directional Movement
        |*****************************/
        case BIDIR_P:
          digitalWrite(BIDIR_LED, HIGH);
          if((LIN && ROUT) || (LOUT && RIN)){
            state = CLEAR;
            state_start = millis();
          }
          if(isTimeout()) state = IDLE;
          break;

        /*****************************|
                Enter Pending
        |*****************************/
        case ENTER_P:
          if(IN ){//&& millis()-state_start>500){
            ++counter;
            state = CLEAR;
            digitalWrite(CTR_INC, HIGH);
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
          if(OUT){//&& millis()-state_start>500){
            --counter;
            state = CLEAR;
            digitalWrite(CTR_DEC, HIGH);
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
          if(isClear()){
            state = IDLE;
            dbl = 0;
            dblp = 0;
            digitalWrite(CTR_INC, LOW);
            digitalWrite(CTR_DEC, LOW);
            digitalWrite(IDLE_LED, LOW);
            digitalWrite(BIDIR_LED, LOW);
          }
          break;
      }

      client.printf("Count: %-4d | State: %4d | Double: %4d | Pending: %4d<br>",
                    counter, state, dbl, dblp);
    }
    
    client.stop();
    Serial.println("Client disconnected");
  }
  return; // Skip legacy serial logic

  /*****************************|
        Alt USB/Serial Logic
  |*****************************/

  // Read TOF Data
  //if(!pollSensors(client)) return;
  
  // switch(state){
  //   /*****************************|
  //         Default IDLE state
  //   |*****************************/
  //   case IDLE:
  //     digitalWrite(IDLE_LED, HIGH);
  //     //if ((LINS && ROUTS) || (LOUTS && RINS)){  state = BIDIR_P;}
  //     if (IN && OUT){                      state = IDLE;}
  //     else if (OUT){                            state = ENTER_P;}
  //     else if (IN){                             state = EXIT_P;}
  //     else {                                    state = IDLE;}

  //     dblp = (LINS && RINS) || (LOUTS && ROUTS);
  //     state_start = millis();
  //     break;

  //   /*****************************|
  //        Bi-Directional Movement
  //   |*****************************/
  //   case BIDIR_P:
  //     digitalWrite(BIDIR_LED, HIGH);
  //     if((LIN && ROUT) || (LOUT && RIN)){
  //       state = CLEAR;
  //       state_start = millis();
  //     }
  //     if(isTimeout()) state = IDLE;
  //     break;

  //   /*****************************|
  //           Enter Pending
  //   |*****************************/
  //   case ENTER_P:
  //     if(IN ){//&& millis()-state_start>500){
  //       ++counter;
  //       state = CLEAR;
  //       digitalWrite(CTR_INC, HIGH);
  //       if(dblp && (LINS || RINS)){
  //         ++counter;
  //         dbl = 1;
  //       }
  //       state_start = millis();
  //     }
  //     if(isTimeout()) state = IDLE;
  //     break;

  //   /*****************************|
  //           Exit Pending
  //   |*****************************/
  //   case EXIT_P:
  //     if(OUT){//&& millis()-state_start>500){
  //       --counter;
  //       state = CLEAR;
  //       digitalWrite(CTR_DEC, HIGH);
  //       if(dblp && (LOUTS || ROUTS)){
  //         --counter;
  //         dbl = 1;
  //       }
  //       state_start = millis();
  //     } 
      
  //     if(isTimeout()) state = IDLE;
  //     break;

  //   /*****************************|
  //         Clear Waiting Area
  //   |*****************************/
  //   case CLEAR:
  //     if(isClear()){
  //       state = IDLE;
  //       dbl = 0;
  //       dblp = 0;
  //       digitalWrite(CTR_INC, LOW);
  //       digitalWrite(CTR_DEC, LOW);
  //       digitalWrite(IDLE_LED, LOW);
  //       digitalWrite(BIDIR_LED, LOW);
  //     }
  //     break;
  // }

  /*****************************|
             Debugging
  |*****************************/
  // if(dbl){
  //   digitalWrite(LED_PIN, 1);
  //   delay(1000);
  //   digitalWrite(LED_PIN, 0);
  // }
  // Serial.printf("Count: %-4d \t|\t State: %4d \t|\t Double: %4d \t|\t Pending: %4d\n\r", counter, state, dbl, dblp); 

}
