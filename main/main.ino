#include <Wire.h>                       // I2C
#include <esp_task_wdt.h>               // Watch Dog Timer
#include <SparkFun_VL53L5CX_Library.h>  // TOF lib
#include <i2cdetect.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <utility>

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

unsigned long state_start = 0;              // ms
const unsigned long TIMEOUT = 2000;         // ms
const unsigned long debounce_thresh = 5;  //ms
const unsigned long clear_thresh = 1500;    //ms

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
#define dist_threshold 900    // mm
#define active_threshold 4    // How many zones required to trigger a detection 
#define sbs_active_threshold 2 // Side-By-Side zone threshold

// Blob structure to represent connected components
struct Blob {
    std::vector<std::pair<int, int>> pixels; // (row, col) coordinates
    int min_row, max_row, min_col, max_col;
    int size;
    
    Blob() : min_row(8), max_row(0), min_col(8), max_col(0), size(0) {}
    
    void addPixel(int row, int col) {
        pixels.push_back(std::make_pair(row, col));
        min_row = std::min(min_row, row);
        max_row = std::max(max_row, row);
        min_col = std::min(min_col, col);
        max_col = std::max(max_col, col);
        size++;
    }
};

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

// Blob detection function
std::vector<Blob> findBlobs(bool binaryImage[8][8], int minBlobSize = 2) {
    std::vector<Blob> blobs;
    bool visited[8][8] = {false};
    
    // 8-connected neighborhood
    int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            if (binaryImage[r][c] && !visited[r][c]) {
                Blob blob;
                std::queue<std::pair<int, int>> q;
                
                q.push(std::make_pair(r, c));
                visited[r][c] = true;
                
                while (!q.empty()) {
                    std::pair<int, int> current = q.front();
                    q.pop();
                    int current_r = current.first;
                    int current_c = current.second;
                    
                    blob.addPixel(current_r, current_c);
                    
                    // Check all 8 neighbors
                    for (int i = 0; i < 8; i++) {
                        int nr = current_r + dr[i];
                        int nc = current_c + dc[i];
                        
                        if (nr >= 0 && nr < 8 && nc >= 0 && nc < 8 && 
                            binaryImage[nr][nc] && !visited[nr][nc]) {
                            visited[nr][nc] = true;
                            q.push(std::make_pair(nr, nc));
                        }
                    }
                }
                
                if (blob.size >= minBlobSize) {
                    blobs.push_back(blob);
                }
            }
        }
    }
    
    return blobs;
}

// Helper function to handle side-by-side cases
void handleSideBySideDetection(const std::vector<Blob>& leftBlobs, const std::vector<Blob>& rightBlobs) {
    // Detect multiple blobs in the same sensor (side-by-side people)
    if (leftBlobs.size() >= 2) {
        Serial.println("Multiple blobs detected on LEFT sensor - side by side");
        // Mark both inner small regions as active if blobs are in appropriate positions
        for (size_t i = 0; i < leftBlobs.size(); i++) {
            const Blob& blob = leftBlobs[i];
            float centroid_col = 0;
            for (size_t j = 0; j < blob.pixels.size(); j++) {
                centroid_col += blob.pixels[j].second;
            }
            centroid_col /= blob.size;
            
            if (centroid_col <= 1) { // Very left side
                cell_active[L_INS] = true;
                cell_active[L_OUTS] = true;
            }
        }
    }
    
    if (rightBlobs.size() >= 2) {
        Serial.println("Multiple blobs detected on RIGHT sensor - side by side");
        // Mark both inner small regions as active if blobs are in appropriate positions
        for (size_t i = 0; i < rightBlobs.size(); i++) {
            const Blob& blob = rightBlobs[i];
            float centroid_col = 0;
            for (size_t j = 0; j < blob.pixels.size(); j++) {
                centroid_col += blob.pixels[j].second;
            }
            centroid_col /= blob.size;
            
            if (centroid_col >= 6) { // Very right side
                cell_active[R_INS] = true;
                cell_active[R_OUTS] = true;
            }
        }
    }
    
    // Detect simultaneous entry from both sides
    if (!leftBlobs.empty() && !rightBlobs.empty()) {
        bool leftInner = cell_active[L_INS] || cell_active[L_INB];
        bool rightInner = cell_active[R_INS] || cell_active[R_INB];
        bool leftOuter = cell_active[L_OUTS] || cell_active[L_OUTB];
        bool rightOuter = cell_active[R_OUTS] || cell_active[R_OUTB];
        
        if ((leftInner && rightInner) || (leftOuter && rightOuter)) {
            Serial.println("Simultaneous detection on both sides");
        }
    }
}

// Updated countCells function with blob detection
void countCells(bool bleft, bool bright)
{
    for (int i = 0; i < 8; i++) {
        cell_counts[i] = 0;
        cell_active[i] = false;
    }
    
    // Create binary images for each sensor
    bool leftBinary[8][8] = {false};
    bool rightBinary[8][8] = {false};
    
    // Fill binary images
    for(int i = 0; i < image_resolution; i++) {
        int row = i / image_width;
        int col = i % image_width;
        
        if(bleft && meetsThresh(i, datal)) {
            leftBinary[row][col] = true;
        }
        if(bright && meetsThresh(i, datar)) {
            rightBinary[row][col] = true;
        }
    }
    
    // Find blobs for left sensor
    std::vector<Blob> leftBlobs = findBlobs(leftBinary, 2); // min 2 pixels for blob
    std::vector<Blob> rightBlobs = findBlobs(rightBinary, 2);
    
    // Process left sensor blobs
    for (size_t i = 0; i < leftBlobs.size(); i++) {
        const Blob& blob = leftBlobs[i];
        // Calculate blob centroid
        float centroid_row = 0, centroid_col = 0;
        for (size_t j = 0; j < blob.pixels.size(); j++) {
            centroid_row += blob.pixels[j].first;
            centroid_col += blob.pixels[j].second;
        }
        centroid_row /= blob.size;
        centroid_col /= blob.size;
        
        // Classify blob based on position
        CellType cell_type;
        
        if(centroid_col >= 0 && centroid_col <= 0 && centroid_row >= 0 && centroid_row <= 3) {
            cell_type = L_INS;
        } else if (centroid_col >= 0 && centroid_col <= 0 && centroid_row >= 3 && centroid_row <= 7){
            cell_type = L_OUTS;
        } else if (centroid_col >= 1 && centroid_col <= 7 && centroid_row >= 0 && centroid_row <= 3){
            cell_type = L_INB;
        } else { // centroid_col >= 1 && centroid_col <= 7 && centroid_row >= 3 && centroid_row <= 7
            cell_type = L_OUTB;
        }
        
        cell_counts[cell_type] += blob.size; // Use blob size as weight
        cell_active[cell_type] = cell_active[cell_type] || (blob.size >= 2);
    }
    
    // Process right sensor blobs  
    for (size_t i = 0; i < rightBlobs.size(); i++) {
        const Blob& blob = rightBlobs[i];
        // Calculate blob centroid
        float centroid_row = 0, centroid_col = 0;
        for (size_t j = 0; j < blob.pixels.size(); j++) {
            centroid_row += blob.pixels[j].first;
            centroid_col += blob.pixels[j].second;
        }
        centroid_row /= blob.size;
        centroid_col /= blob.size;
        
        // Classify blob based on position
        CellType cell_type;
        
        if(centroid_col >= 0 && centroid_col <= 6 && centroid_row >= 0 && centroid_row <= 3) {
            cell_type = R_INB;
        } else if (centroid_col >= 0 && centroid_col <= 6 && centroid_row >= 3 && centroid_row <= 7){
            cell_type = R_OUTB;
        } else if (centroid_col >= 6 && centroid_col <= 7 && centroid_row >= 0 && centroid_row <= 3){
            cell_type = R_INS;
        } else { // centroid_col >= 6 && centroid_col <= 7 && centroid_row >= 3 && centroid_row <= 7
            cell_type = R_OUTS;
        }
        
        cell_counts[cell_type] += blob.size; // Use blob size as weight
        cell_active[cell_type] = cell_active[cell_type] || (blob.size >= 2);
    }
    
    // Additional logic for side-by-side detection
    handleSideBySideDetection(leftBlobs, rightBlobs);
    
    Serial.printf("Inner: %-4d %-4d %-4d %-4d\n\r", cell_active[L_INS], cell_active[L_INB], cell_active[R_INB], cell_active[R_INS]);
    Serial.printf("Outer: %-4d %-4d %-4d %-4d\n\r", cell_active[L_OUTS], cell_active[L_OUTB], cell_active[R_OUTB], cell_active[R_OUTS]);
}

bool isTimeout(){
  return millis() - state_start > TIMEOUT;
}

bool isDebounce(){
  return millis() - state_start > debounce_thresh;
}

bool isClear(){
  return millis() - state_start > clear_thresh;
}

int dbl = 0;
int dblp = 0;

void loop()
{
  if(PUTTY_DEBUG)CLEAR_SCREEN();
  
  // Poll both sensors for new data
  bool bleft = pollSensor(tofl, datal);
  bool bright = pollSensor(tofr, datar);
  if(!bleft || !bright) return;

  // Count cells using blob detection
  countCells(bleft, bright);
  
  // FSM state merging
  bool LOUT = cell_active[L_OUTB] || cell_active[L_OUTS];
  bool LIN  = cell_active[L_INB]  || cell_active[L_INS];
  bool ROUT = cell_active[R_OUTB] || cell_active[R_OUTS];
  bool RIN  = cell_active[R_INB]  || cell_active[R_INS];
  bool OUT  = LOUT || ROUT;
  bool IN   = LIN  || RIN;

  bool tl = false;
  bool tr = false;
  switch(state){
    // Default IDLE state
    case IDLE:
      digitalWrite(LED_PIN, LOW);
      state = IN && OUT ? IDLE: OUT ? ENTER_P : IN ? EXIT_P : IDLE;
      dblp = (cell_active[L_INS] || cell_active[R_INS]) || (cell_active[L_OUTS] || cell_active[R_OUTS]); 
      state_start = millis();
      break;

    // Door Enter Pending
    case ENTER_P:
      digitalWrite(LED_PIN, HIGH);
      if(IN && isDebounce()) {
        ++counter;
        state = CLEAR;
        
        if(dblp && (cell_active[L_INS] || cell_active[R_INS])){
          ++counter;
          dbl = 1;
        }
      }
      if(isTimeout()) state = IDLE;
      break;

    // Door Exit Pending
    case EXIT_P:
      if(OUT && isDebounce()) {
        --counter;
        state = CLEAR;
        
        if(dblp){
          --counter;
          dbl = 1;
        }
      } 
      
      if(isTimeout()) state = IDLE;
      break;

    // Door Trigger Clear
    case CLEAR:
      if(!IN && !OUT && isClear() || isTimeout()){
        state = IDLE;
        dbl = 0;
        dblp = 0;
        digitalWrite(LED_PIN, LOW);
      }
      break;
  }
  Serial.printf("Count: %-4d \t|\t State: %4d \t|\t Double: %4d \t|\t Pending: %4d\n\n\r", counter, state, dbl, dblp);
}