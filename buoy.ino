#include <Adafruit_PWMServoDriver.h>
#include <cmath>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>

// Define the RX and TX pins for Serial 2
#define RXD2 25
#define TXD2 26
#define GPS_BAUD 9600

TinyGPSPlus gps;
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();
TFT_eSPI tft;

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

//imu object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//PID
const double kP               = 0.5;
const double BASE_SPEED       = 60.0;
const double MAX_SPEED        = 100.0;
const double ARRIVAL_RADIUS_M = 2.0;

double dist = 0;

struct coordinate {
  double lat;
  double lon;
};

double wrapAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}
/*
void setMotorSpeed(int channel, float speed) {
  speed = constrain(speed, 0.0f, 100.0f);
  int pulse = (int)map((long)speed, 0, 100, 205, 410);
  pca.setPWM(channel, 0, pulse);
}
*/

void setMotorSpeed(int channel, float speed) {

  speed = constrain(speed, 0.0f, 100.0f);

  const int MIN_RUN = 350;
  const int MAX_RUN = 410;

  int pulse = MIN_RUN + (speed / 100.0f) * (MAX_RUN - MIN_RUN);

  pca.setPWM(channel, 0, pulse);
}

void tftStatus(const char* line1, const char* line2 = "", uint16_t color = TFT_GREEN) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(4, 20); tft.println(line1);  
  tft.setTextSize(1);
  tft.setCursor(4, 60); tft.println(line2);  
}

coordinate target = {38.267382, -122.382084};

void setup(){
  // Serial Monitor
  tft.init();
  tft.setRotation(1);
  Serial.begin(115200);
  tftStatus("Buoy Boot", "Starting...", TFT_YELLOW);
  Serial.println("START");
  delay(400);
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");

pca.begin();
pca.setPWMFreq(50);
  // I2C setup for ESP32
  Wire.begin(21, 22);   // SDA = 21, SCL = 22

  // Initialize IMU sensors
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
     tftStatus("IMU FAIL", "Check wiring", TFT_RED);
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  tftStatus("IMU", "OK", TFT_GREEN);  
  delay(400);

//setMotorSpeed(14, 0);

//setMotorSpeed(15, 0);

pca.setPWM(14, 0, 0);
pca.setPWM(15, 0,0);

delay(5000);
tftStatus("Motors", "OK", TFT_GREEN);  
}

double targetAngle = 0;

void loop(){
  while (gpsSerial.available() > 0){
    // get the byte data from the GPS
    //char gpsData = gpsSerial.read();
   // Serial.print(gpsData);
   gps.encode(gpsSerial.read());
   if (gps.location.isUpdated()) {
     Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: "); 
      Serial.println(gps.location.lng(), 6);
      Serial.print("SPEED (km/h) = "); 
      Serial.println(gps.speed.kmph()); 
      Serial.print("ALT (min)= "); 
      Serial.println(gps.altitude.meters());
      Serial.print("HDOP = "); 
      Serial.println(gps.hdop.value() / 100.0); 
      Serial.print("Satellites = "); 
      Serial.println(gps.satellites.value()); 
      Serial.print("Time in PST: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + (String((gps.time.hour() + 24 - 8) % 24)) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      Serial.print("Distance from Target = "); 
      Serial.println(TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        target.lat,
        target.lon
    ));
      double dLon = target.lon - gps.location.lng();
      double dLat = target.lat - gps.location.lat();
      double targetAngle = atan2(dLon, dLat) * 180.0 / PI;
      dist = TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        target.lat,
        target.lon
    );


      if (targetAngle < 0)
        targetAngle += 360; 
      //0 is north
      Serial.print("Target Angle = "); 
      Serial.println(targetAngle); 
      Serial.println("");
    }
    }

  sensors_event_t event; 
  bno.getEvent(&event);
  
   Serial.print("Distance = "); 
    Serial.println(dist); 
    
  /* Display the floating point data */
  Serial.print("Heading: ");
  Serial.println(event.orientation.x, 4);
  double currentHeading = event.orientation.x;
  double angleError = wrapAngle(targetAngle - event.orientation.x);
  Serial.print("Angle off from Target = "); 
  Serial.println(angleError); 

   float leftSpeed = 0, rightSpeed = 0;

  //if (dist > ARRIVAL_RADIUS_M) {
    float correction = kP * (float)angleError;
    correction  = constrain(correction, -BASE_SPEED, BASE_SPEED);
    leftSpeed   = constrain(BASE_SPEED - correction, 0, MAX_SPEED);
    rightSpeed  = constrain(BASE_SPEED + correction, 0, MAX_SPEED);
 // }

  setMotorSpeed(14,  50);
  Serial.print("left speed = "); 
  Serial.println(leftSpeed); 
  setMotorSpeed(15, 50);
  Serial.print("right speed = "); 
  Serial.println(rightSpeed); 
 tftStatus("fifty", "bith", TFT_GREEN);
 delay(5000);
 setMotorSpeed(14,  leftSpeed);
  Serial.print("left speed = "); 
  Serial.println(leftSpeed); 
  setMotorSpeed(15, rightSpeed);
  Serial.print("right speed = "); 
  Serial.println(rightSpeed); 
 tftStatus("normal", "bith", TFT_GREEN);
 delay(5000);
 /*
  setMotorSpeed(14,  100);
  Serial.print("left speed = "); 
  Serial.println(leftSpeed); 
  setMotorSpeed(15, 100);
  Serial.print("right speed = "); 
  Serial.println(rightSpeed); 
 tftStatus("hund", "bith", TFT_GREEN);
 delay(5000);
 */
 

//test!:

/*
setServoAngle(14, 90);
delay(1000);

setServoAngle(14, 180);
delay(1000);
*/
  delay(1000);
  Serial.println("-------------------------------");
}
