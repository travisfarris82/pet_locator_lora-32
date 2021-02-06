/****************************************
 * Include Libraries
 ****************************************/
#include "heltec.h"
#include "images.h"
<<<<<<< Updated upstream

=======
#include "DHTesp.h"
#include "ESP32Ticker.h"
#include <TinyGPS++.h>
#include <SPI.h>
#include <SoftwareSerial.h>

/****************************************
 * Define Constants
 ****************************************/
float hum;
float temp;

/****************************************
 * Configure DHT22 
 ****************************************/

#ifndef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP32 ONLY!)
#error Select ESP32 board.
#endif
DHTesp dht;
/** Pin number for DHT22 data pin */
int dhtPin = 22;

/****************************************
 * Configure GPS Module
 ****************************************/
static const int RXPin = 17, TXPin = 2; // GPS transmisson lines
static const uint32_t GPSBaud = 9600; // Default baud rate for NEO-7M GPS module

TinyGPSPlus tinyGPS;  // Create a tinyGPS object
SoftwareSerial ssGPS(RXPin, TXPin); // Assigns pins to GPS module

/****************************************
 * Configure LoRa ESP32
 ****************************************/
>>>>>>> Stashed changes
#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;

<<<<<<< Updated upstream
=======

void setup()
{
// Serial Debug Settings
  Serial.begin(115200);
  ssGPS.begin(GPSBaud);
  Serial.println(F("Ublox NEO-7M GPS Module Test\n"));
  delay(1000);
  //Serial.println(F("\nDHT22 ESP32 Debug!\n"));
// DHT22 Setup
  dht.setup(dhtPin, DHTesp::DHT22);
  Serial.println("DHT initiated");
//WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
// Display images called from the "image.h" header file
  logo();
  delay(1500);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "LoRa32 GPS Pet Locator");
  Heltec.display->display();
  delay(1000);
  
}

void loop(){
   float converted = 0.00;
   //Read data and store it to variables hum and temp
   hum = dht.getHumidity();
   temp= dht.getTemperature();

  //Fahrenheit
  //T(°F) = T(°C) × 9/5 + 32
  converted = ( temp * 1.8 ) + 32;
  Serial.println("Fahrenheit = " + String(converted) + "°F");
  Serial.println("Humidity = " + String(hum));
  String t_data = String("Temp: " + String(converted)+ " °F");
  sendData(t_data);
  String h_data = String("Humidity: " + String(hum));
  sendData(h_data);

  // Send GPS data to OLED display
  gps_OLED();
  
  // print position, altitude, speed, time/date, and satellites:
  printGPSInfo();

  // Send GPS LAT data to LoRa Receiver
  sendData("LAT: " + String(tinyGPS.location.lat(), 6));

  // Send GPS LONG data to LoRa Receiver
  sendData("LONG: " + String(tinyGPS.location.lng(), 6));

  // Send GPS Date to LoRa Receiver
  sendData("DATE: " + getGPSDate());

  // Send GPS Time to LoRa Receiver
  sendData("TIME: " + getGPSTime()); 
  
  //2000mS delay between reads
  delay(2000);
}

/****************************************
 * Auxiliary Functions
 ****************************************/


// Send data packet
void sendData(String tx_data) {
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "Sending packet: ");
  Heltec.display->drawString(90, 0, String(counter));
  Heltec.display->display(); 
  // send packet
  LoRa.beginPacket();
  
/*
 * LoRa.setTxPower(txPower,RFOUT_pin);
 * txPower -- 0 ~ 20
 * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
 *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
*/
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(tx_data);
  //LoRa.print(value);
  LoRa.endPacket();

  counter++;
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
}

>>>>>>> Stashed changes
void logo()
{
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
}

// Used to send GPS data to OLED display
void gps_OLED()
{
  Heltec.display->clear();
  Heltec.display->drawString(0, 0,"Lat: " +(String(tinyGPS.location.lat(), 6)));
  Heltec.display->drawString(0, 10,"Lng: " +String(tinyGPS.location.lng(), 6));
  Heltec.display->drawString(0, 20,"Date: " +String(tinyGPS.date.month()) +("/")
  +String(tinyGPS.date.day()) +("/") +String(tinyGPS.date.year()));
  Heltec.display->drawString(0, 30,"Time: " +String(tinyGPS.time.hour()-6) +(":")
  +String(tinyGPS.time.minute()) +(":") +String(tinyGPS.time.second()));
  Heltec.display->display();
<<<<<<< Updated upstream
  delay(1000);
}

void loop()
{
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  
  Heltec.display->drawString(0, 0, "Sending packet: ");
  Heltec.display->drawString(90, 0, String(counter));
  Heltec.display->display();

  // send packet
  LoRa.beginPacket();
  
/*
 * LoRa.setTxPower(txPower,RFOUT_pin);
 * txPower -- 0 ~ 20
 * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
 *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
*/
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
=======
}

void printGPSInfo()
{
  // Print latitude, longitude, date, time
  Serial.print("Lat: "); 
  Serial.println(tinyGPS.location.lat(), 6);
  Serial.print("Long: "); 
  Serial.println(tinyGPS.location.lng(), 6);
  Serial.println();
}

// printDate() formats the date into mm/dd/yy.
String getGPSDate()
{
  Serial.print("Date: ");
  Serial.print(tinyGPS.date.month());
  Serial.print("/");
  Serial.print(tinyGPS.date.day());
  Serial.print("/");
  Serial.println(tinyGPS.date.year());
  return String(String(tinyGPS.date.month()) + "/" + String(tinyGPS.date.day()) + "/" + String(tinyGPS.date.year()));
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
String getGPSTime()
{
  String s_minute;
  String s_second;
  Serial.print("Time: ");
  Serial.print(tinyGPS.time.hour());
  Serial.print(":");
  if (tinyGPS.time.minute() < 10) {
    Serial.print('0');
    s_minute = "0";
  } else {
    Serial.print(tinyGPS.time.minute());
    s_minute = String(tinyGPS.time.minute());
  }
  Serial.print(":");
  if (tinyGPS.time.second() < 10) {
    Serial.println('0');
    s_second = "0";
  } else {
    Serial.println(tinyGPS.time.second());
    s_second = String(tinyGPS.time.second());
  }
  return String(String(tinyGPS.time.hour()) + ":" + s_minute + ":" + s_second);
>>>>>>> Stashed changes
}
