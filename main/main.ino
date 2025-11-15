#include <Wire.h>
#include <esp_task_wdt.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <i2cdetect.h>

// --- Hardware Defines, Debug, & Timers (Unchanged) ---
#define tofl_rst 14
#define tofr_rst 13
#define tofl_addr 0x44
#define tofr_addr 0x46
#define tof_addr_default 0x29
#define I2C_freq 400000
#define image_resolution 64
#define image_width 8
#define ranging_freq 15
#define LED_PIN 2

#define I2C_DEBUG false
#define PUTTY_DEBUG false
#define CLEAR_SCREEN() Serial.write("\033[2J\033[H")

unsigned long state_start = 0;
const unsigned long TIMEOUT = 2000;
const unsigned long debounce_thresh = 5;
const unsigned long clear_thresh = 1500;

// --- State Machine (Unchanged) ---
enum CrossState {
  IDLE,
  ENTER_P,
  EXIT_P,
  CLEAR
};
CrossState state = IDLE;

// --- REMOVED ---
// enum CellType { ... };
// int  cell_counts[8];
// bool cell_active[8];
// #define active_threshold 3
// #define sbs_active_threshold 3

// --- NEW --- (Blob Detection Data Structures)
struct Pixel { int row; int col; };
struct Blob {
  int id;
  int size;
  int sumRow;
  int sumCol;
  float centroidRow;
  float centroidCol;
};

// --- Left & Right TOF Sensor Objects (Unchanged) ---
SparkFun_VL53L5CX tofl;
VL53L5CX_ResultsData datal;
SparkFun_VL53L5CX tofr;
VL53L5CX_ResultsData datar;

// --- Software Globals ---
int counter = 0;
#define dist_threshold 900
#define MIN_BLOB_SIZE 3 // Our new noise/flicker filter

// --- NEW --- (Blob Detection Globals)
int sensorBitmap[64];
Blob blobList[8];        // Max 8 blobs
int blobMap[64];
Pixel pixelQueue[64];
int queueHead = 0;
int queueTail = 0;

// --- setup() function is UNCHANGED ---
// (It correctly initializes both sensors)
void setup()
{
  Serial.begin(921600);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  Serial.println("- - - - - - - - - - Initializing - - - - - - - - - -");
  Wire.begin();
  Wire.setClock(I2C_freq);
  pinMode(tofl_rst, OUTPUT);
  pinMode(tofr_rst, OUTPUT);
  digitalWrite(tofl_rst, HIGH);
  digitalWrite(tofr_rst, HIGH);
  delay(1000);
  digitalWrite(tofl_rst, LOW);
  delay(1000);
  if (I2C_DEBUG) {i2cdetect(); delay(2000);}
  Serial.println(("Initializing Left TOF"));
  if (!tofl.begin()) {
    Serial.print("Trying aux address: 0x");
    Serial.println(tofl_addr, HEX);
    if (!tofl.begin(tofl_addr)) {
      Serial.println("Left TOF: Not Found");
      while (1);
    }
  }
  Serial.println("Left TOF Found!");
  if (tofl.setAddress(tofl_addr) == false) {
    Serial.println(("Left TOF: Failed to change addr"));
    while (1);
  } else if (tofl.getAddress() == tof_addr_default) {
    Serial.print(("Setting Left TOF address to: 0x"));
    Serial.println(tofl.getAddress(), HEX);
    delay(1000);
  }
  if (I2C_DEBUG) {i2cdetect(); delay(2000);}
  digitalWrite(tofr_rst, LOW);
  delay(1000);
  if (I2C_DEBUG) {i2cdetect(); delay(2000);}
  Serial.println("Initializing Right TOF");
  if (!tofr.begin()) {
    Serial.print("Trying aux address: 0x");
    Serial.println(tofr_addr, HEX);
    if (!tofr.begin(tofr_addr)) {
      Serial.println("Right TOF: Not Found");
      while (1);
    }
  }
  Serial.println("Right TOF Found!");
  if (I2C_DEBUG) {i2cdetect(); delay(2000);}
  tofl.setResolution(image_resolution);
  tofr.setResolution(image_resolution);
  tofl.setRangingFrequency(ranging_freq);
  tofr.setRangingFrequency(ranging_freq);
  tofl.startRanging();
  tofr.startRanging();
  state = IDLE;
  Serial.println("- - - - - - - - - - Initialization complete, starting counter - - - - - - - - - - ");
}

// --- pollSensor() and meetsThresh() are UNCHANGED ---
bool pollSensor(SparkFun_VL53L5CX &sens, VL53L5CX_ResultsData &buf)
{
  if (!sens.isDataReady()) return false;
  if (sens.getRangingData(&buf)) return true;
  return false;
}
bool meetsThresh(int i, VL53L5CX_ResultsData &buf)
{
  return buf.distance_mm[i] > 0 && buf.distance_mm[i] <= dist_threshold;
}

// --- REMOVED ---
// void debug() { ... }
// void countCells(...) { ... }

// --- NEW --- (Blob Detection Queue Functions)
void queuePush(Pixel p) {
  pixelQueue[queueTail] = p;
  queueTail = (queueTail + 1) % 64;
}
Pixel queuePop() {
  Pixel p = pixelQueue[queueHead];
  queueHead = (queueHead + 1) % 64;
  return p;
}
bool isQueueEmpty() { return queueHead == queueTail; }
void queueClear() { queueHead = 0; queueTail = 0; }

// --- NEW --- (The Main Blob Detection Function)
int findBlobs(int* bitmap, Blob* blobs, int maxBlobs, int* blobID_map)
{
  int currentBlobID = 1;
  memset(blobID_map, 0, 64 * sizeof(int));
  queueClear();

  for (int i = 0; i < 64; i++) {
    if (bitmap[i] == 1 && blobID_map[i] == 0) {
      if (currentBlobID > maxBlobs) break;
      Blob& currentBlob = blobs[currentBlobID - 1];
      currentBlob.id = currentBlobID;
      currentBlob.size = 0;
      currentBlob.sumRow = 0;
      currentBlob.sumCol = 0;

      Pixel startPixel = { i / 8, i % 8 };
      queuePush(startPixel);
      blobID_map[i] = currentBlobID;

      while (!isQueueEmpty()) {
        Pixel p = queuePop();
        currentBlob.size++;
        currentBlob.sumRow += p.row;
        currentBlob.sumCol += p.col;

        int neighbors_row[] = { p.row - 1, p.row + 1, p.row, p.row };
        int neighbors_col[] = { p.col, p.col, p.col - 1, p.col + 1 };

        for (int n = 0; n < 4; n++) {
          int nr = neighbors_row[n];
          int nc = neighbors_col[n];
          if (nr >= 0 && nr < 8 && nc >= 0 && nc < 8) {
            int neighborIndex = nr * 8 + nc;
            if (bitmap[neighborIndex] == 1 && blobID_map[neighborIndex] == 0) {
              blobID_map[neighborIndex] = currentBlobID;
              Pixel neighborPixel = { nr, nc };
              queuePush(neighborPixel);
            }
          }
        }
      }
      if (currentBlob.size > 0) {
        currentBlob.centroidRow = (float)currentBlob.sumRow / currentBlob.size;
        currentBlob.centroidCol = (float)currentBlob.sumCol / currentBlob.size;
      }
      currentBlobID++;
    }
  }
  return currentBlobID - 1;
}

// --- Timer functions are UNCHANGED ---
bool isTimeout() {
  return millis() - state_start > TIMEOUT;
}
bool isDebounce() {
  return millis() - state_start > debounce_thresh;
}
bool isClear() {
  return millis() - state_start > clear_thresh;
}

// --- REMOVED ---
// int dbl = 0;
// int dblp = 0;

void loop()
{
  if (PUTTY_DEBUG)CLEAR_SCREEN();

  // Poll both sensors for new data
  bool bleft = pollSensor(tofl, datal);
  bool bright = pollSensor(tofr, datar);
  if (!bleft || !bright) return;

  // --- NEW --- (Create the 64-pixel bitmap)
  // This combines both sensors into one map
  for (int i = 0; i < 64; i++) {
    // Check if EITHER sensor sees something
    if (meetsThresh(i, datal) || meetsThresh(i, datar)) {
      sensorBitmap[i] = 1;
    } else {
      sensorBitmap[i] = 0;
    }
  }

  // --- NEW --- (Find all blobs)
  int numBlobs = findBlobs(sensorBitmap, blobList, 8, blobMap);

  // --- NEW --- (Generate FSM Inputs)
  // This replaces all the LOUT, LIN, ROUT, RIN logic
  bool IN = false;
  bool OUT = false;

  for (int i = 0; i < numBlobs; i++) {
    Blob& b = blobList[i];

    // --- Noise Filter (replaces hysteresis) ---
    if (b.size >= MIN_BLOB_SIZE) {
      
      // Check which "zone" the blob's CENTER is in
      // Rows 0-3 are "Inner", Rows 4-7 are "Outer"
      // (You can swap this by changing < 4.0)
      if (b.centroidRow < 4.0) {
        IN = true;
      } else {
        OUT = true;
      }
    }
  }

  // --- Simplified FSM (dblp and dbl logic removed) ---
  switch (state) {
    // Default IDLE state
    case IDLE:
      digitalWrite(LED_PIN, LOW);
      // This logic is now robust. A person in the middle is 1 blob
      // with one centroid, so it's impossible to trigger IN and OUT at once.
      // The (IN && OUT) check only catches two separate people (bi-dir).
      state = IN && OUT ? IDLE : OUT ? ENTER_P : IN ? EXIT_P : IDLE;
      state_start = millis();
      break;

    // Door Enter Pending
    case ENTER_P:
      digitalWrite(LED_PIN, HIGH);
      if (IN && isDebounce()) {
        ++counter;
        state = CLEAR;
      }
      if (isTimeout()) state = IDLE;
      break;

    // Door Exit Pending
    case EXIT_P:
      if (OUT && isDebounce()) {
        --counter;
        state = CLEAR;
      }
      if (isTimeout()) state = IDLE;
      break;

    // Door Trigger Clear
    case CLEAR:
      if (!IN && !OUT && isClear() || isTimeout()) {
        state = IDLE;
        digitalWrite(LED_PIN, LOW);
      }
      break;
  }
  
  // Updated Serial Print
  Serial.printf("Count: %-4d | State: %d | Blobs: %d | IN: %d | OUT: %d\n\n\r", counter, state, numBlobs, IN, OUT);
}