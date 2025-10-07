#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

int peopleCount = 0;
bool currentlyDetected = false; // detection state

const int DISTANCE_THRESHOLD = 500; // mm
const int MIN_ACTIVE_ZONES = 5;     // how many zones must trigger to count as detection

void setup()
{
  pinMode(2,OUTPUT);
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

  myImager.setRangingFrequency(15);

  myImager.startRanging();
}

void loop()
{
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData))  //Read distance data into array
    {
      int activeZones = 0;

      // Count how many zones are within threshold
      for (int i = 0; i < imageResolution; i++)
      {
        if (measurementData.distance_mm[i] > 0 && measurementData.distance_mm[i] <= DISTANCE_THRESHOLD)
        {
          activeZones++;
        }
      }

      // Detection logic
      if (activeZones >= MIN_ACTIVE_ZONES)
      {
        if (!currentlyDetected)
        {
          digitalWrite(2,1);
          peopleCount++;
          currentlyDetected = true; // object detected
          Serial.print("Count incremented! Total = ");
          Serial.println(peopleCount);
        }
      }
      else
      {
        currentlyDetected = false; // reset when object leaves
        digitalWrite(2,0);
      }
    }
  }

  delay(50); // adjust sampling rate
}
