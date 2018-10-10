//Пины для SoftwareSerial
#define RXpin 2 //RX 
#define TXpin 3 //TX

#define SoftwareserialSpeed 38600 //Скорость для SoftwareSerial подключения
#define HardwareserialSpeed 38600 //Скорость для Serial подключения

#include <TinyGPS++.h> //Для расшифровки пакетом NMEA с GPS для дальнейшей работы с ними
#include <SoftwareSerial.h> //Для подключения GPS к Arduino
#include <Wire.h> //Для подключение HMC5883L по протоколу I2C
#include <HMC5883L.h> //Для работы с компасом
#include <math.h> //Для математичкских расчётов

//Создание объекта для работы с GPS
TinyGPSPlus gps;

//Создание подключения с GPS через програмный Serial порт.
SoftwareSerial ss(RXpin, TXpin);

//Создание объекта для работы с компасом
HMC5883L compass;

//Структура для работы с waypoint
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
        g_day, g_month, g_year, g_seconds; //День, месяц, год, секунды
    float compas_degrees;  //Азимут с компаса в градусах
    bool FINISH_FLAG = false; //Флаг завершения миссии
    int actual_point = 0;//Номер обрабатываемой целевой точки
    float to_point_distance; //Расстояние до целевой точки
    float to_point_azimuth; //Азимут целевой точки

    //Количество waypoint'ов
    #define waypoint_count 2
    
    //Маршрут
    waypoint route[waypoint_count] = {{48.529998, 135.055007, 0},
                                      {48.530651, 135.054473, 2}
        
                                     };
    //В констукторе класса помещена принудительная классификация начальной и финальной точки
    NavigationObject() 
    {
      route[0].point_type = 0;  //Классификация начальной точки
      route[waypoint_count - 1].point_type = 2; //Классификация финальной точки
    }
    
    
    void get_GPS_location() //Считывание долготы и широты
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

    bool isFinish() //Проверка на достижения последней точки
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

    void math() //Расчёт расстояние между точками и азимута
    {
      //pi - число pi, rad - радиус сферы (Земли)
      float rad = 6372795.0; //Это число должно быть float. Обязательно

      //координаты двух точек
      float llat1 = route[actual_point].point_latitude;
      float llong1 = route[actual_point].point_longitude;
      float llat2 = route[actual_point + 1].point_latitude;
      float llong2 = route[actual_point + 1].point_longitude;

      //в радианах
      float lat1 = (llat1 * M_PI) / 180.0;
      float lat2 = (llat2 * M_PI) / 180.0;
      float long1 = (llong1 * M_PI) / 180.0;
      float long2 = llong2 * M_PI / 180.0;

      //Косинусы и синусы широт, разницы долгот
      float cl1 = cos(lat1);
      float cl2 = cos(lat2);
      float sl1 = sin(lat1);
      float sl2 = sin(lat2);
      float delta = long2 - long1;
      float cdelta = cos(delta);
      float sdelta = sin(delta);

      //Вычисление длины большого круга
      float y = sqrt(pow((cl2 * sdelta), 2) + pow((cl1 * sl2 - sl1 * cl2 * cdelta), 2));
      float x = (sl1 * sl2) + (cl1 * cl2 * cdelta);
      float ad = atan2(y, x);
      float dist = ad * rad;

      //Вычисление начального азимута
      x = (cl1 * sl2) - (sl1 * cl2 * cdelta);
      y = sdelta * cl2;
      float z = (atan(-y / x) * 180) / M_PI;

      if (x < 0) z = z + 180.0;

      float z2 = (int)(z + 180.0) % 360 - 180.0;
      z2 = -(z2 * (M_PI / 180.0));
      float anglerad2 = z2 - ((2 * M_PI) * floor((z2 / (2 * M_PI))));
      float angledeg = anglerad2 * (180.0 / M_PI);

      //Внесение ответов в базу
      to_point_distance = dist;
      to_point_azimuth = angledeg;
    }

};

void setup() {
  // Создание Serial подключения
  Serial.begin(HardwareserialSpeed);
  ss.begin(SoftwareserialSpeed);

  // Подключение компаса
  Serial.print("Initialize HMC5883L ... ");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sendor, check wiring!");
    delay(500);
  }
  Serial.println(" Done"); //Компас готов к работе

  // Колибровка и настройка компаса
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










