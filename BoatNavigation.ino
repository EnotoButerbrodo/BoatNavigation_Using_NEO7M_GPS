//Пины для SoftwareSerial
#define RXpin 2 //RX пин
#define TXpin 3 //TX пин

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

  bool point_type;  //0-обычный 1-конечный
};

//Основной класс для работы с объектом навигации
class NavigationObject
{
  public:
    float latitude,   //Широта у объекта
          longitude;  //Долгота у объекта

    int actual_point = 0; //Номер обрабатываемой целевой точки.

    bool FINISH_FLAG = false; //Флаг завершения миссии

    float distance_to_waypoint, //Расстояние до целевой точки
          azimuth,     //Азимут
          bearing,     //Пеленг судна
          cross_track_error; //Ошибка курса

    //Количество waypoint'ов
#define waypoint_count 1

    //Маршрут
    waypoint route[waypoint_count] = {{48.529998, 135.055007, 1}};



    void get_location() //Считывание долготы и широты
    {

      while (ss.available() > 0)
        if (gps.encode(ss.read())) { //Пиздец какая важная строка. Без неё всё виснет

          if (gps.location.isValid())
          {
            latitude = gps.location.lat();
            Serial.println(latitude);

            longitude = gps.location.lng();
            Serial.println(longitude);
          }
        }
    }

    void get_bearing() //Определение пеленга
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
      bearing = headingDegrees;
    }

    bool isFinish() //Проверка на достижения последней точки
    {
      if (route[actual_point].point_type == 1)
      {
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
      float llat1 = latitude;
      float llong1 = longitude;
      float llat2 = route[actual_point].point_latitude;
      float llong2 = route[actual_point].point_longitude;

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
      distance_to_waypoint = dist;
      azimuth = angledeg;
    }

    void get_cross_track_error() //Вычисление ошибки курса
    {
      cross_track_error = azimuth - bearing;
    }

    bool resive_point() //Проверка на достижение окрестностей целевой точки
    {
      if ((fabs(latitude - route[actual_point].point_latitude) <= 0.00001) &&
          fabs((longitude - route[actual_point].point_longitude) <= 0.00001))
      {
        return true;
        
        if(!Boat.ifFinish()) //Если не достигли конечной точки
        {          
        actual_point ++; // Начинаем работать с другой        
        }
        else
        {
          FINISH_FLAG = true; //Иначе отмечаем завершение миссии
          Serial.println("Finish mission");
        }
       
      }
      else
      {
        return false;
      }
    }
};


void setup() {
  // Создание Serial подключения
  Serial.begin(HardwareserialSpeed);
  ss.begin(SoftwareserialSpeed); //Для GPS

  // Инициализация компаса
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


}

//Объект для работы с лодочкой
NavigationObject Boat;

void loop() {
  // put your main code here, to run repeatedly:

  while (!Boat.FINISH_FLAG)//Пока не достигли точки
  {
    Boat.get_location();//Получаем данные с GPS
    Boat.get_bearing(); //Получаем пеленг
    Boat.math(); //Считаем азимут и расстояние до точки
    Boat.get_cross_track_error(); //Расчитываем ошибку курса
    Boat.resive_point(); //Проверка на достижение точки и на завершение миссии

    //Выводим
    Serial.print("Position : "); Serial.print(Boat.latitude, 7); Serial.print("  "); Serial.println(Boat.longitude, 7);

    Serial.print("Distance to point "); Serial.println(Boat.distance_to_waypoint);
    Serial.print("Azimuth "); Serial.println(Boat.azimuth, 7);
    Serial.print("Bearing "); Serial.println(Boat.bearing);
    Serial.print("Cross-track eroor "); Serial.println(Boat.cross_track_error);
    
    delay(200);

  }

}










