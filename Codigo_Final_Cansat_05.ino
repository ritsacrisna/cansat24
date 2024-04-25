//------------------------------------------------------
//Importação de biliotecas
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "AK09918.h"
#include "ICM20600.h"
#include <TinyGPSPlus.h>
#include <SD.h>

//Secção de declaração de constantes e variáveis globais
#define pinBUZZER 4
#define SEALEVELPRESSURE_HPA (1013.25)
#define rad 1/57.3

//------------------------------------------------------
//Secção de instanciação de objetos
const long interval = 1000;
unsigned long previousTime = 0;
unsigned long actualTime = 0;

// BMP
Adafruit_BMP3XX bmp;
const float zz = 0.0065, R = 287.06, g = 9.81;
float T0 = 0;
float P0 = 0;
float temperature = 0;
float pressure = 0;
float altitude = 0;

//giroscopio
AK09918 ak09918;
ICM20600 icm20600(true);
AK09918_err_type_t err;
int32_t x, y, z;
int16_t acc_x, acc_y, acc_z;
int16_t gg_x, gg_y, gg_z;
double heading;
int32_t offset_x, offset_y, offset_z;
double roll, pitch;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_santamaria = -6.933; // -6° 56'

float X;
float A;
float XC;
float YC;
float r;

//APC
#define pinAPCTX 1
#define pinAPCRX 0
//Serial1(pinAPCRX, pinAPCTX); // Define os pinos de comunicação RX e TX para o APC
int cmd;

//GPS
#define pinGPSTX 2
#define pinGPSRX 7
//Serial2(pinGPSRX, pinGPSTX); // the serial interface to the GPS module
TinyGPSPlus gps; // the TinyGPS+ object

//cartao sd
#define pinSDCS 8
String filename = "cansat24.csv";
File logfile;

//buzzer
int activateBuzzer = 0;

//fotodiodos
const int analogRedPin = A0; 
const int analogNirPin = A1;
int RED = 0;  
int NIR = 0;
float NDVI;

//formatar dados para enviar
String data = "";
String gpsdata = "";
String bmpdata = "";
String gyrodata = "";
String diodedata = "";



//------------------------------------------------------
void setup(){
  //void setup geral
  pinMode(pinBUZZER, OUTPUT);

  //inicialização de comunicações, do sensor e do módulo emissor
  Serial.begin(9600);  // the Serial port of Arduino baud rate.
  Serial1.begin(9600); // Inicializa a comunicação serial com o módulo APC-220
  Serial2.begin(9600);  // Default baud of Air530z GPS module is 9600
  delay(1000);

  //void setup do GPS
  Serial2.println("$PCAS01,4*18");//alterar o baud rate para 57600
  delay(1000);
  Serial2.flush();
  delay(1000);
  Serial2.end();
  delay(1000);
  Serial2.begin(57600);
  delay(1000);
  Serial2.println("$PCAS10,0*1C"); //warm start
  delay(250);
  //Serial2.println("$PCAS03,1,0,0,0,1,0,0,0,0,0,0,0,0*1E"); //envia apenas o GGA e o RMC 

  //void setup do BMP
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial1.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //void setup do giroscopio
  Wire.begin();
  err = ak09918.initialize();
  icm20600.initialize();
  ak09918.switchMode(AK09918_POWER_DOWN);
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

  err = ak09918.isDataReady();
  while (err != AK09918_ERR_OK) {
    Serial1.println("Waiting Sensor");
    delay(100);
    err = ak09918.isDataReady();
  }

  //Serial.println("Start figure-8 calibration after 2 seconds.");
  Serial1.println("Start figure-8 calibration after 2 seconds.");
  delay(2000);
  calibrate(10000, &offset_x, &offset_y, &offset_z);
  //Serial.println();
  Serial1.println();
  delay(2000);
   //lê os valores da pressão e temperatura e guarda nas variáveis P0 e T0 respetivamente
  P0 = bmp.readPressure() / 100; //converte para hPa
  T0 = bmp.readTemperature();

  //void setup do ficheiro
  if (!SD.begin(pinSDCS)) {
    Serial1.println("initialization failed!");
    while (1);
  }
  Serial1.println("SD initialization done.");
  logfile = SD.open(filename, FILE_WRITE);    
  if (logfile) {
    Serial1.print("Writing to cansat24.csv  ...");
    logfile.println("DATE,TIME,AGE,LAT,LATM,LNG,LNGM,HDOP,COURSE,SPEED,SPEEDM,ALTITUDE,SAT");
  }
  // close the file:
  logfile.close();
  Serial1.println("done.");
}

//------------------------------------------------------
void loop() {

  getValues(); //chamar a função que lê os dados

  actualTime = millis(); 
  //se passaram 1000ms desde o último envio de dados, enviar dados
  if (actualTime - previousTime >= interval) {
    //atualiza o tempo atual
    previousTime = actualTime;
    //chamar a função que envia os dados com aviso
    if (activateBuzzer)
      tone(pinBUZZER,1500);

    sendData();  
  }  
  noTone(pinBUZZER);

  receiveCmds();

}

//------------------------------------------------------
void receiveCmds() {

 if (Serial1.available()) {
    char c = Serial1.read();
    if (c == '1') {
      Serial1.println("buzzer");
      activateBuzzer = 1;
    }
    if (c == '0') {
      Serial1.println("no buzzer");
      activateBuzzer = 0;
    }
  }
}

//------------------------------------------------------
void sendData() {
  //compor string para enviar
  data = gpsdata + bmpdata + gyrodata + diodedata;
  
  //enviar os dados para o módulo apc220
  Serial1.println();
  if (data != "")  
    Serial1.println(data);

  saveData();
}

//------------------------------------------------------
void saveData() {
  if (data != "") {  
    if(SD.exists(filename)) { // check the card is still there
      // now append new data file
      logfile = SD.open(filename, FILE_WRITE );
      if (logfile) {
        logfile.println(data);
        logfile.close(); // close the file
        data = "";
      }
    }
    else {
      logfile = SD.open(filename, FILE_WRITE );
      if (logfile) {
        logfile.println("DATE,TIME,AGE,LAT,LATM,LNG,LNGM,HDOP,COURSE,SPEED,SPEEDM,ALTITUDE,SAT");
        logfile.println(data);
        logfile.close(); // close the file
        data = "";
      }
      Serial1.println("Error writing to file !");
    }
  }
}

//------------------------------------------------------
void getValues() {

  //GPS
  if (Serial2.available() > 0) {
    while(Serial2.available())               // reading data into char array
    {
      int ch = Serial2.read();
      gps.encode(ch);
      Serial.write(ch);
    }
  }

  if (gps.location.isValid()) {
    gpsdata = String(gps.date.value()) + "," + String(gps.time.value()) + "," + String(gps.location.age()) + "," + \
              String(gps.location.lat(),6) + "," + String(gps.location.lng(),6) + "," + \
              String(gps.hdop.hdop()) + "," + String(gps.course.deg()) + "," + \
              String(gps.speed.kmph()) + "," + String(gps.altitude.meters()) + "," + String(gps.satellites.value());
  }

  //BMP
  if (! bmp.performReading()) {
    Serial1.println("Failed to perform BMP reading :(");
    return;
  }
  
  //ler valores do sensor e guardar no bmpdata
  bmpdata = String(bmp.temperature) + "," + String(bmp.pressure / 100.0) + "," + String(bmp.readAltitude(SEALEVELPRESSURE_HPA));

/*
  temperature = bmp.temperature;
  pressure = bmp.pressure / 100.0;
  //calcular a altitude
  //altitude = ((T0 + 273.15) / z) * (1 - (pow(pressure / P0, (z * R) / g)));
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
*/

  //giroscópio
  acc_x = icm20600.getAccelerationX();
  acc_y = icm20600.getAccelerationY();
  acc_z = icm20600.getAccelerationZ();

  gg_x = icm20600.getGyroscopeX();
  gg_y = icm20600.getGyroscopeY();
  gg_z = icm20600.getGyroscopeZ();

  ak09918.getData(&x, &y, &z);
  x = x - offset_x;
  y = y - offset_y;
  z = z - offset_z;

  // roll/pitch/heading
  roll = (atan2((float)acc_y, (float)acc_z)) * 57.3;
  pitch = (atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z))) * 57.3;

  double Xheading = x * cos(pitch) + y * sin(roll) * sin(pitch) + z * cos(roll) * sin(pitch);
  double Yheading = y * cos(roll) - z * sin(pitch);

  heading = (180 + 57.3 * atan2(Yheading, Xheading) + declination_santamaria) * 57.3;

  if (pitch < 0) {
    pitch = pitch + 90;
  } 

  X = 10 * tan((pitch - 20) * rad);
  A = (10 * (tan((pitch + 20) * rad) - tan((pitch - 20) * rad))) / 2;
  r = X + A;
  XC = r * cos(heading * rad) - 100;
  YC = r * sin(heading * rad) - 100;

  if (pitch > 70) 
    A = 0.0; //se aparecer 0.0 na string, então é porque A não é valido
  

  gyrodata = String(roll) + "," + String(pitch) + "," + String(heading) + "," + String(X) + "," + \
             String(A) + "," + String(r) + "," + String(XC) + "," + String(YC);

//fotodiodos
  RED = analogRead(analogRedPin);
  NIR = analogRead(analogNirPin);
  NDVI = ((float)NIR - (float)RED)/((float)NIR + (float)RED);
 
  diodedata = String(RED) + "," + String(NIR) + "," + String(NDVI);

}
 
//------------------------------------------------------
void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
  int32_t value_x_min = 0;
  int32_t value_x_max = 0;
  int32_t value_y_min = 0;
  int32_t value_y_max = 0;
  int32_t value_z_min = 0;
  int32_t value_z_max = 0;
  uint32_t timeStart = 0;

  ak09918.getData(&x, &y, &z);

  value_x_min = x;
  value_x_max = x;
  value_y_min = y;
  value_y_max = y;
  value_z_min = z;
  value_z_max = z;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout) {
    ak09918.getData(&x, &y, &z);

    /* Update x-Axis max/min value */
    if (value_x_min > x) {
      value_x_min = x;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    } else if (value_x_max < x) {
        value_x_max = x;
        // Serial.print("update value_x_max: ");
        // Serial.println(value_x_max);
      }

    /* Update y-Axis max/min value */
    if (value_y_min > y) {
      value_y_min = y;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    } else if (value_y_max < y) {
        value_y_max = y;
        // Serial.print("update value_y_max: ");
        // Serial.println(value_y_max);
      }

    /* Update z-Axis max/min value */
    if (value_z_min > z) {
      value_z_min = z;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    } else if (value_z_max < z) {
        value_z_max = z;
        // Serial.print("update value_z_max: ");
        // Serial.println(value_z_max);
      }
      
    Serial1.print(".");        
    delay(100);

  }

  *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
  *offsety = value_y_min + (value_y_max - value_y_min) / 2;
  *offsetz = value_z_min + (value_z_max - value_z_min) / 2;

}




