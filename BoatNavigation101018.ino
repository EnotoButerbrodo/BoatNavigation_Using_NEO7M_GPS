//Pins for SoftwareSerial
#define RXpin 2 //RX 
#define TXpin 3 //TX

#define SoftwareserialSpeed 38600 //SoftwareSerial speed
#define HardwareserialSpeed 38600 //HardwareSerial speed



#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <HMC5883L.h> //Compass
#include <math.h>

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXpin, TXpin);

// The compass object
HMC5883L compass;


//Структура для точек
struct waypoint
{
  float point_latitude,   //Широта
        point_longitude;  //Долгота

  byte point_type;  //0-начальный 1-обычный 2-конечный
};



//Основной класс для работы с объектом навигации
class NavigationObject
{
  public:
    int latitude,   //Широта
        longitude,  //Долгота
        g_day, g_month, g_year, g_seconds;
    float compas_degrees;  //Азимут в градусах c компаса
    bool FINISH_FLAG = false; //Флаг завершения миссии
    int actual_point = 0;//Номер обрабатываемой точки
    float to_point_distance; //Расстояние до точки
    float to_point_azimuth; //Азимут точки

    //Маршрут
    waypoint route[2] = {{48.529998, 135.055007, 0},
                         {48.530651, 135.054473, 2}
    };

    void get_GPS_location()//Считывание долготы и широты
    {
      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        Serial.println(latitude);

        longitude = gps.location.lng();
        Serial.println(longitude);
      }

    }

    void get_GPS_date() //Считывание даты с GPS
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

    void get_GPS_time() //Считывание времени с GPS
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
    
    void Get_Compas_Data() //Считывание и расчёт азимута с компаса
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
      compas_degrees = headingDegrees;
    }
    
    bool isFinish() //Проверка достижения последней точки
    {
      if (route[actual_point].point_type == 2)
      {
        FINISH_FLAG = true;
        return true;
      }
      else
      {
        return false;
      }
    }
    
    void math() //Расчёт расстояние между точками и азимут
    {
      //pi - число pi, rad - радиус сферы (Земли)
      float rad = 6372795.0;

      //координаты двух точек
      float llat1 = route[actual_point].point_latitude;
      float llong1 = route[actual_point].point_longitude;
      float llat2 = route[actual_point+1].point_latitude; 
      float llong2 = route[actual_point+1].point_longitude;
     

      //в радианах
      float lat1 = (llat1 * M_PI) / 180.0;
      float lat2 = (llat2 * M_PI) / 180.0;
      float long1 = (llong1 * M_PI) / 180.0;
      float long2 = llong2 * M_PI / 180.0;
      float cl1 = cos(lat1);
      float cl2 = cos(lat2);
      float sl1 = sin(lat1);
      float sl2 = sin(lat2);
      float delta = long2 - long1;
      float cdelta = cos(delta);
      float sdelta = sin(delta);
      float y = sqrt(pow((cl2 * sdelta), 2) + pow((cl1 * sl2 - sl1 * cl2 * cdelta), 2));
      float x = (sl1 * sl2) + (cl1 * cl2 * cdelta);
      float ad = atan2(y, x);
      float dist = ad * rad;
      x = (cl1 * sl2) - (sl1 * cl2 * cdelta);
      y = sdelta * cl2;
      float z = (atan(-y / x) * 180) / M_PI;

      if (x < 0)
      {
        z = z + 180.0;
      }
      
      float z2 = (int)(z + 180.0) % 360 - 180.0;
      z2 = -(z2 * (M_PI / 180.0));
      float anglerad2 = z2 - ((2 * M_PI) * floor((z2 / (2 * M_PI))));
      float angledeg = anglerad2 * (180.0 / M_PI);
      Serial.print("Distance ");Serial.println(dist);
      to_point_distance = dist;
      Serial.print("Azimuth ");Serial.println(angledeg);
      to_point_azimuth = angledeg;

      
    

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


   NavigationObject Boat; //Создание объекта для лодки
   Boat.math();//Расчитать 
}



void loop() {
  // put your main code here, to run repeatedly:

  
}










