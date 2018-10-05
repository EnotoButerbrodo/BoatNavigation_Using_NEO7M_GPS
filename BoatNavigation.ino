//Pins for SoftwareSerial
#define RXpin 2 //RX 
#define TXpin 3 //TX

#define SoftwareserialSpeed 38600 //SoftwareSerial speed
#define HardwareserialSpeed 38600 //HardwareSerial speed

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h> //Compass

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXpin, TXpin);

// The compass object
HMC5883L compass;


class NavigationObject
{
  public:
    int latitude, longitude, g_day, g_month, g_year, g_seconds;
    float Degrees;

    void Get_GPS_Location()
    {
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        Serial.println(latitude);

        longitude = gps.location.lng();
        Serial.println(longitude);
      }

    }

    void Get_GPS_Date()
    {
      while (ss.available() > 0)
        if (gps.encode(ss.read())) {


          if (gps.date.isValid())
          {
            g_month = gps.date.month();
            g_day   = gps.date.day();
            g_year  = gps.date.year();
          }
          else
          {
            Serial.println("No");
          }
        }
    }

    void Get_GPS_Time()
    {
      while (ss.available() > 0)
        if (gps.encode(ss.read()))
        {
          if (gps.time.isValid())
          {
            g_seconds = gps.time.second();
          }
        }
    }

    void Get_Compas_Data()
    {
      Vector norm = compass.readNormalize();

      // Calculate heading
      float heading = atan2(norm.YAxis, norm.XAxis);
      float declinationAngle = (-11.0 + (-56.0 / 60.0)) / (180 / M_PI);
      heading += declinationAngle;

      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0)  heading += 2 * PI;
      if (heading > 2 * PI)  heading -= 2 * PI;

      // Convert to degrees
      float headingDegrees = ((heading * 180 / M_PI) + 130);
      if (headingDegrees >= 360) headingDegrees -= 360;
      Degrees = headingDegrees;
    }
};

void setup() {
  // === Serial initialize ===
  Serial.begin(HardwareserialSpeed);
  ss.begin(SoftwareserialSpeed);

  // === Compass initialize ===
  Serial.print("Initialize HMC5883L ... ");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sendor, check wiring!");
    delay(500);
  }
  Serial.println(" Done");

  // === Compas calibration ===
  compass.setRange(HMC5883L_RANGE_1_3GA);  // Set measurement range
  compass.setMeasurementMode(HMC5883L_CONTINOUS);  // Set measurement mode
  compass.setDataRate(HMC5883L_DATARATE_30HZ); // Set data rate
  compass.setSamples(HMC5883L_SAMPLES_8); // Set number of samples averaged
  compass.setOffset(-127, -118);  // Set calibration offset. See HMC5883L_calibration.ino

}



void loop() {
  // put your main code here, to run repeatedly:




}










