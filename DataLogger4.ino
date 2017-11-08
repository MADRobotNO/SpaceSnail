//DataLogger for Arduino MEGA used to High Altitude Balloon in Globall Space Balloon Challenge 2017.
//Created by Martin Agnar Dahl for MAD Robot

//========================

//Datalogger v.4 
//Added SMS function and millis counter
//=======
//Datalogger v.3
//Changed SIM808 Library and few new functions
//=======
  
  //SIM808+GPS
#include <gps.h>
#include <SIM900.h>
#include <sms.h>

  // BME280
#include <BME280I2C.h> //BME280 via I2C
#include <BME280.h> //BME280

  //DS18B20
#include <OneWire.h> //DS18B20 uses One Wire
#include <DallasTemperature.h>  // DS18B20

  //SD
#include <SPI.h> //SD Card uses SPI
#include <SD.h>  //SD Card

  //DHT22
#include <DHT.h> //DHT22

  //I2C
#include <Wire.h>  //I2C

//DS3231 (clock)
#include "Sodaq_DS3231.h"  // clock DS3231

// One wire bus for ds18b20 - Temp.Outside
#define ONE_WIRE_BUS 3
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//DHT22 variables (temp.inside)
#define DHTPIN 11
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

  float h;
  float t;
  char weekDay[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
  const int chipSelect = 53;
  
  int d;
  int mon;
  int y;

  int hr;
  int m;
  int s;

//ds18b20 variables (temp.outside)
  float tempOut;

//BME280 variables
  BME280I2C bme; 
  bool metric = true;
  float temp(NAN), hum(NAN), pres(NAN);
  uint8_t pressureUnit(1);
  float altitude;


  //GPS variables
GPSGSM gps;
char lon[15];
char lat[15];
char alt[15];
char time[20];
char speed[15];

char stat;

float conv_lon;
float conv_lat;

  //SIM variables
SMSGSM sms;
int numdata;
char smsbuffer[160];
char n[20];

//Time counter
unsigned long timer;
unsigned long prevTimer;
int interval = 45; //in sec

// =============================SETUP===================================

void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
  //RED diode lights up
  digitalWrite(4, HIGH);

  //Serial starts
  Serial.begin(9600); 
  Serial1.begin(9600);
  
  dht.begin();
  Wire.begin();
  rtc.begin();
  sensors.begin();
  
  separator();
  DateTime now = rtc.now();
  Serial.print("System starts at: ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
  Serial.println(' ');
  String startTime ="";
  startTime = String(now.date()) + "/" + String(now.month()) + "/" + String(now.year()) + "," + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  separator();
  
   // see if the card is present and can be initialized: 
  Serial.println("Initializing SD card... ");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    Serial.println(' ');  
  }
  else{
  Serial.println("SD Card initialized successfully!");  
  Serial.println(' ');
  
  // write headers
  String dataStringHead ="";
  dataStringHead = String ("Date") + "," + String ("Time") + "," + String ("Humid.") + "," + String ("Temp.Inside") + "," + String ("Temp.Outside") + "," + String ("Pressure") + "," + String ("Altitude") + "," + String ("Longitude") + "," + String ("Latitude") + "," + String ("Altitude from GPS") + "," + String ("Speed from GPS");
  File dataFile = SD.open("logg.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("=============================================================================================================");
    dataFile.print("System starts at: ");
    dataFile.println(startTime);
    dataFile.println("=============================================================================================================");
    dataFile.println("");
    dataFile.println(dataStringHead);
    dataFile.close();
  }
  else{
    Serial.println("Error opening data file...");
  }
  }
  
  separator(); 
  
  testDHT();
  
  separator();
  
  bme280test();
  
  separator();
  
  testGPS();

  separator();
  
  sateliteTest();
  
  separator(); 
  
  Serial.println("Logging starts");
  Serial.println(' ');  
  
  separator();  
  
  allTestsOk(); //blinks
}

// =============================LOOP===================================

void loop() {
  timer = millis();
  gpsDataInfo();
  nowtime(); 
  readDHT();
  readTempOut();
  BME280read();
  logg();
  //sendSMS();
  Serial.println("");
  separator();
  delay(2000);
}

// =============================OTHERS===================================

// GPS test
void testGPS(){
  Serial.println("SIM and GPS module (SIM808) test starts now.");
  Serial.println("");
  Serial.print("Connecting to SIM808.");
    gsm.forceON();
    while(!gsm.begin(9600)) { 
      delay(1000);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("SIM808 Module connected.");
  Serial.println("");
  delay(200);
  Serial.print("Powering GPS.");
  while(!gps.attachGPS())
  {
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("GPS is now ON.");
  Serial.println("");
  Serial.println("SIM and GPS module (SIM808) test passed!");
  Serial.println("");
  delay(500);
}

// Satelite test

void sateliteTest(){
  Serial.println("Getting GPS signal");
  digitalWrite(6, HIGH);
  Serial.print("Please wait...");
  stat=gps.getStat();
  while(stat!=3){
    stat=gps.getStat();
    Serial.print(".");
    Serial.println(stat, DEC);
    delay(1000);
    if(stat==3){
      break;
      }
    }
  digitalWrite(6, LOW);
  Serial.println("");
  Serial.println("GPS signal fixed (3D)!");
  Serial.println("");
  delay(500);
}


//GPS+SIM data
void gpsDataInfo(){
  digitalWrite(5, HIGH);  
  
  gps.getPar(lon,lat,alt,time,speed);
  Serial.println("GPS Data: ");
  Serial.print("Altitude: ");
  Serial.println(alt);
  Serial.print("Ttime: ");
  Serial.println(time);
  Serial.print("speed: ");
  Serial.println(speed);
  Serial.print("Conv. lon, lat: ");
  conv_lon = atof(lon);
  conv_lat = atof(lat);
  Serial.print(conv_coords(conv_lon), DEC);
  Serial.print(", ");
  Serial.println(conv_coords(conv_lat), DEC);
  Serial.println("");
  
  conv_lon = conv_coords(conv_lon);
  conv_lat = conv_coords(conv_lat);
  digitalWrite(5, LOW);
}

void allTestsOk() {
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  delay(600);
  digitalWrite(5, LOW);
  delay(400);
  digitalWrite(5, HIGH);
  delay(600);
  digitalWrite(5, LOW);
  delay(400);
  digitalWrite(5, HIGH);
  delay(600);
  digitalWrite(5, LOW);
  delay(400);
  digitalWrite(5, HIGH);
  delay(600);
  digitalWrite(5, LOW);
  delay(400);
  digitalWrite(5, HIGH);
  delay(600);
  digitalWrite(5, LOW);
  delay(400);
  digitalWrite(5, HIGH);
  delay(600);
  digitalWrite(5, LOW);
  delay(400);
}

// only graphic separator for Serial monitor
void separator() {
  Serial.println("=============================================================================================================");
  Serial.println(' ');
}

// Reading DHT
void readDHT() {
  Serial.println("Temperature in data:");
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from temperature (DHT22) sensor!");
    return;
    }
  else {
  Serial.print("Humidity: "); 
  Serial.print(h);
  Serial.print("%, ");
  Serial.print("Temperature: "); 
  Serial.print(t);
  Serial.println("*C ");
  Serial.println("");
  }
}

// Readint Temperature Out
void readTempOut() {
  Serial.println("Temperature out data:");
    // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
//  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
//  Serial.print("DONE, ");
  tempOut = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.println(tempOut);
  Serial.println("");
    // You can have more than one IC on the same bus. 
    // 0 refers to the first IC on the wire
}

// Testing if DHT22 works properly
void testDHT() {
  Serial.println("Temperature sensor (DHT22) test starts now...");
  h = dht.readHumidity();
  t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
  Serial.println("Failed to read from temperature (DHT22) sensor!");
  Serial.println(' ');
  return;
  }
  else {
  Serial.println("Temperature sensor (DHT22) test passed!");
  Serial.println("");
  }
}

// Getting time
void nowtime() {
  Serial.println("Clock time:");
  DateTime now = rtc.now();

   d = now.date();
   mon = now.month();
   y = now.year();

   hr = now.hour();
   m = now.minute();
   s = now.second();
  
  Serial.print(y);
  Serial.print('/');
  Serial.print(mon);
  Serial.print('/');
  Serial.print(d);
  Serial.print(' ');
  Serial.print(hr);
  Serial.print(':');
  Serial.print(m);
  Serial.print(':');
  Serial.print(s);
  Serial.print(' ');
  Serial.println(weekDay[now.dayOfWeek()]);
  Serial.println("");
}

// Logging data
void logg() {

String dataString = "";
String dateOnClock = "";
dateOnClock += String(d) + "/" + String(mon) + "/" + String (y) + ",";
dataString += dateOnClock;
//clock
dataString += String(hr); 
dataString += ":";
dataString += String(m);
dataString += ":";
dataString += String(s);
dataString += ",";
//temp&humid from DHT22
dataString += String(h);
dataString += ",";
dataString += String(t);
dataString += ",";
//Temp outside
dataString += String(tempOut);
dataString += ",";
//Data from BME280
dataString += String(pres);
dataString += ",";
dataString += String(altitude);
dataString += ",";
//Data from GPS
dataString += String(conv_lon, 8);
dataString += ",";
dataString += String(conv_lat, 8);
dataString += ",";
dataString += String(alt);
dataString += ",";
dataString += String(speed);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("logg.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.print("Logged data: ");
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening data file...");
  }
}

void bme280test(){
  Serial.println("Altitude sensor (BME280) test starts now...");
  if(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");
    Serial.println(' ');
    return;
  }
  else {
    Serial.println("Altitude sensor (BME280) test passed!");
    Serial.println(' ');
  }
}

void BME280read() {
  Serial.println("Barometric Data:");
  bme.read(pres, temp, hum, metric, pressureUnit);
  altitude = bme.alt(metric);
  Serial.print("Pressure: ");
  Serial.println(pres);
  Serial.print("Altitude: ");
  Serial.println(altitude);
  Serial.println("");
}

float conv_coords(float in_coords)
 {
 //Initialize the location.
 float f = in_coords;
 // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
 // firsttowdigits should be 77 at this point.
 int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
 float nexttwodigits = f - (float)(firsttwodigits*100);
 float theFinalAnswer = (float)(firsttwodigits + nexttwodigits/60.0);
 return theFinalAnswer;
 }

void sendSMS(){
  double seconds=timer/1000;
  double minutes=seconds/60;
  double hours=minutes/60;
  char buf[160];
  Serial.println("Time since start:");
  Serial.print("Millis:");
  Serial.println(timer);
  Serial.print("Sec: ");
  Serial.println(seconds);
  Serial.print("Min: ");
  Serial.println(minutes);
  Serial.print("Hrs: ");
  Serial.println(hours);  
  String smsText = "";
  String timeNow ="";
  String link ="";
  link += String("http://www.google.com/maps/place/")+String(conv_lon, 8)+","+String(conv_lat, 8);
  timeNow += String(hr); 
  timeNow += ":";
  timeNow += String(m);
  timeNow += ":";
  timeNow += String(s);
  Serial.println(timeNow);
  smsText += String(conv_lon, 8) + ", " + String(conv_lat, 8) + ", " + String(alt) + ", " + timeNow + ", " + link;
  Serial.println(smsText);
  if(seconds - prevTimer > interval) {
    smsText.toCharArray(buf, 160);
    sms.SendSMS("46385343", buf);
    prevTimer = seconds; 
  }
}

