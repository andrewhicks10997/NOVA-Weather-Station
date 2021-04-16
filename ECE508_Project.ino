/**************************************************************************
 ECE508 Project
 Group 8 - Weather Station
 Modules Used - BMP180, Raindrop Sensor, DHT22, BN-220(GPS), OLED
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EMailSender.h>
#include <WiFiNINA.h>
#include "NETWORK_INFO.h"

#include <Adafruit_BMP085.h> //Library for BMP180 (Need to download zip file from here: https://github.com/adafruit/Adafruit-BMP085-Library)


// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4


/*********Setting up the Barometer***************/
Adafruit_BMP085 bmp; //Using the BMP180, but this will still work

/*********Setting up the Temp Sensor (DHT22)***************/
#include <SimpleDHT.h> // headers for DHT_22
int pinDHT22 = 2; //setting pin D2 for sensor
SimpleDHT22 dht22(pinDHT22);
float temperature = 0;
float humidity = 0;
int errDHT22 = SimpleDHTErrSuccess;


/*********Setting up the Email Functions***************/
EMailSender emailSend("NOVAWeatherStation@gmail.com", "ECE508IoTWeatherStation123!");     //signing into client email (address and password)
uint8_t connection_state = 0;
uint16_t reconnect_interval = 10000;
#define EmailRecipients {"EnterEmailAddressHere@gmail.com", "CanContinueWithMoreAddressesOnList@hotmail.com"}
#define RecipientsSize 2          //length of EmailRecipient Array

/*********Setting up the Raindrop Sensor***************/
int inputPinRain = 3;
int valRain = 0;                    // variable for reading the pin status

/*********Setting up the UDFsr***************/
uint8_t WiFiConnect(const char* nSSID, const char* nPassword);
void Awaits();
void SendEmailAlert(char subject[], char contents[]);

  
void setup() {
  Serial.begin(9600);
  SendEmailAlert("testing", "testing123");
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  }
}



void loop() {
  get_BMP180_Values(); //Function call for Barometer
  delay(500);
  get_DHT22_Values(); //Function call for DHT22
  get_rain_status();
}

   

void get_BMP180_Values(){

    Serial.println("BMP180 INFO: ");
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();
    delay(500);
}

void get_DHT22_Values(){
  errDHT22 = dht22.read2(&temperature, &humidity, NULL);
  if (errDHT22 != 0) {
    temperature = (-1 - 32)/1.8;
    humidity = -1;
  }
  Serial.println("DHT22 INFO: ");
  Serial.print("Temperature in C: ");
  Serial.println(temperature);
  Serial.print("Humidity in RH%: ");
  Serial.println(humidity);
  Serial.println();
  delay(500);
  
  
  }
  
void get_rain_status(){
  valRain = digitalRead(inputPinRain);
  Serial.println("RAINDROP SENSOR INFO");
  if (valRain == HIGH) {            // check if the input is HIGH

  Serial.println("NO RAIN");   
  } 
    else{

    Serial.println("It is Raining");    
  }
  Serial.println();
  }  

  void SendEmailAlert(char subject[], char contents[])
{
    const char* ssid = NETWORK_SSID;
    const char* password = NETWORK_PSA;

    connection_state = WiFiConnect(ssid, password);
    if(!connection_state)  // if not connected to WIFI
        Awaits();          // constantly trying to connect

    EMailSender::EMailMessage message;
    message.subject = String(subject);
    message.message = String(contents);

    // Send to email list
    const char* arrayOfEmail[] = EmailRecipients;
    EMailSender::Response resp = emailSend.send(arrayOfEmail, RecipientsSize, message);

//    // Send to 3 different email, 2 in C and 1 in CC
//    const char* arrayOfEmail[] = {"<FIRST>@gmail.com", "<SECOND>@yahoo.com", "<THIRD>@hotmail.com"};
//    EMailSender::Response resp = emailSend.send(arrayOfEmail, 2, 1, message);
//
//    // Send to 3 different email first to C second to CC and third to CCn
//    const char* arrayOfEmail[] = {"<FIRST>@gmail.com", "<SECOND>@yahoo.com", "<THIRD>@hotmail.com"};
//    EMailSender::Response resp = emailSend.send(arrayOfEmail, 1,1,1, message);


    Serial.println("Sending status: ");

    Serial.println(resp.status);
    Serial.println(resp.code);
    Serial.println(resp.desc);
}

uint8_t WiFiConnect(const char* nSSID = nullptr, const char* nPassword = nullptr)
{
    static uint16_t attempt = 0;
    Serial.print("Connecting to ");
    if(nSSID) {
        WiFi.begin(nSSID, nPassword);
        Serial.println(nSSID);
    }

    uint8_t i = 0;
    while(WiFi.status()!= WL_CONNECTED && i++ < 50)
    {
        delay(200);
        Serial.print(".");
    }
    ++attempt;
    Serial.println("");
    if(i == 51) {
        Serial.print("Connection: TIMEOUT on attempt: ");
        Serial.println(attempt);
        if(attempt % 2 == 0)
            Serial.println("Check if access point available or SSID and Password\r\n");
        return false;
    }
    Serial.println("Connection: ESTABLISHED");
    Serial.print("Got IP address: ");
    Serial.println(WiFi.localIP());
    return true;
}

void Awaits()
{
    uint32_t ts = millis();
    while(!connection_state)
    {
        delay(50);
        if(millis() > (ts + reconnect_interval) && !connection_state){
            connection_state = WiFiConnect();
            ts = millis();
        }
    }
}
