#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

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
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  myImager.setResolution(8*8); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager.setRangingFrequency(15); // Using 8x8, min frequency is 1Hz and max is 15Hz

  myImager.startRanging();
}

void loop()
{
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData))  //Read distance data into array
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
}