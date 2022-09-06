/***************************************************************
 *  author: Mohamed Maoui                                      *
 *  date: September 5, 2022                                    *
 ***************************************************************/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPSPlus.h>
//--------------------------------------------------------------//
// personal phone number to receive sms alerts.
// phone number must start with country code.
const String PHONE = "+213xxxxxxxxx";
//--------------------------------------------------------------//
/*
/* SIM800L
 * Typical pin layout used:
 * ----------------------------
 * Signal     Arduino Nano Pin
 * ----------------------------
 * TX          D2
 * RX          D3
 */
#define txPin 2
#define rxPin 3
SoftwareSerial sim800(rxPin, txPin);
//--------------------------------------------------------------
/*
 * GY-GPS6MV2
 * Typical pin layout used:
 * ----------------------------
 * Signal     Arduino Nano Pin
 * ----------------------------
 * TX          D8
 * RX          D9
 */
AltSoftSerial neogps;
TinyGPSPlus gps;
//--------------------------------------------------------------
#define BUZZER 4
// alarm
int buzzer_timer = 0;
bool alarm = false;
boolean send_alert_once = true;
//--------------------------------------------------------------
// size of the geofencing (in meters)
const float maxDistance = 30;
//--------------------------------------------------------------
float initialLatitude = 36.69747360782452;
float initialLongitude = 4.055741066619446;

float latitude, longitude;
//--------------------------------------------------------------

void getGps(float &latitude, float &longitude);

/***************************************************************
 * setup() function                                            *
 ***************************************************************/
void setup()
{
  // put your setup code here, to run once:
  //-------------------------------------------------------------
  // Arduino serial initialize
  Serial.begin(9600);
  Serial.println("Welcome.");
  Serial.println("......................");
  Serial.println();
  delay(100);
  //--------------------------------------------------------------
  // SIM800L serial initialize
  sim800.begin(9600);
  //--------------------------------------------------------------
  // GY-GPS6MV2 serial initialize
  neogps.begin(9600);
  //--------------------------------------------------------------
  pinMode(BUZZER, OUTPUT);
  //--------------------------------------------------------------
  sim800.println("AT"); // check SIM800L module
  delay(500);
  sim800.println("ATE1"); // echo ON
  delay(500);
  sim800.println("AT+CPIN?"); // check SIM ready
  delay(500);
  sim800.println("AT+CMGF=1"); // SMS text mode
  delay(500);
  sim800.println("AT+CNMI=1,1,0,0,0"); // decides how newly arrived SMS should be handled
  delay(500);
  //--------------------------------------------------------------
  delay(2000);
  buzzer_timer = millis();
}

/*****************************************************************
 * loop() function                                               *
 *****************************************************************/
void loop()
{
  // put your main code here, to run repeatedly:
  //--------------------------------------------------------------
  getGps(latitude, longitude);
  //--------------------------------------------------------------
  // float distance = 30;
  float distance = 31;
  //--------------------------------------------------------------
  Serial.print("initialLatitude= ");
  Serial.println(initialLatitude, 6);
  Serial.print("initialLngitude= ");
  Serial.println(initialLongitude, 6);
  Serial.print("current Distance= ");
  Serial.println(distance);
  //--------------------------------------------------------------
  delay(1000);
  // set alarm on
  if (distance > maxDistance)
  {
    //------------------------------------------
    if (send_alert_once == true)
    {
      digitalWrite(BUZZER, HIGH);
      Serial.println("1");
      sendAlert();
      Serial.println("2");
      alarm = true;
      send_alert_once = false;
      buzzer_timer = millis();
      Serial.println("3");
    }
    //------------------------------------------
  }
  else
  {
    send_alert_once = true;
  }
  //--------------------------------------------------------------

  // handle alarm
  if (alarm == true)
  {
    if (millis() - buzzer_timer > 5000)
    {
      digitalWrite(BUZZER, LOW);
      alarm = false;
      buzzer_timer = 0;
    }
  }
  //--------------------------------------------------------------
  while (sim800.available())
  {
    Serial.println(sim800.readString());
  }
  //--------------------------------------------------------------
  while (Serial.available())
  {
    sim800.println(Serial.readString());
  }
  //--------------------------------------------------------------
}

/*****************************************************************
 * getGps() Function                                             *
 *****************************************************************/
void getGps(float &latitude, float &longitude)
{
  // can take up to 60 seconds
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 2000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
        break;
      }
    }
  }

  if (newData) // if newData is true
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    newData = false;
  }
  else
  {
    Serial.println("No GPS data is available!");
    latitude = 0;
    longitude = 0;
  }
}

/*****************************************************************
 * sendAlert() function                                          *
 *****************************************************************/
void sendAlert()
{
  // return;
  String sms_data;
  sms_data = "Alert! The object is outside the fense.\r";
  sms_data += "http://maps.google.com/maps?q=loc:";
  sms_data += String(latitude) + "," + String(longitude);

  // return;
  sim800.print("AT+CMGF=1\r");
  delay(1000);
  sim800.print("AT+CMGS=\"" + PHONE + "\"\r");
  delay(1000);
  sim800.print(sms_data);
  delay(100);
  sim800.write(0x1A); // ascii code for ctrl-26 // sim800.println((char)26); // ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully!");
}