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

/*********Setting up the Raindrop Sensor***************/
int inputPinRain = 3;
int valRain = 0;                    // variable for reading the pin status

/*********************Setting up calculated values*****************************/
float Tdew = 0;
int fog = 0;    //bool for fog status (0 = no fog, 1 = fog)
float HeatIndex = 0;

/*********************Setting up status flags for calculated values*****************************/
int HeatFlag = 0; 
int FreezeFlag = 0;
int ExtremeColdFlag = 0;
int FogFlag = 0;
int RainFlag = 0;
int RainStat[] = {0,0,0,0,0};
int FogStat[] = {0,0,0,0,0};
int samples = 0;

/*********Setting up the UDFs***************/
void get_BMP180_Values(void);
void get_DHT22_Values(void);
void get_rain_status(void);
uint8_t WiFiConnect(const char* nSSID, const char* nPassword);
void Awaits();
void SendEmailAlert(char subject[], char contents[]);
void Calc_and_Analysis(void);
void FogStatus(void);
void CalcHeatIndex(void);
void WeatherAlert(void);

/*---------------------------------------------------------------SETUP FUNCTION-----------------------------------------------------*/  
void setup() {
  Serial.begin(9600);

  //begin using the bmp module
  if (!bmp.begin()) 
  {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  }
}


/*--------------------------------------------------------LOOP FUNCTION------------------------------------------------------------------*/  
void loop() 
{
  get_BMP180_Values(); //Function call for Barometer
  delay(500);
  get_DHT22_Values(); //Function call for DHT22
  delay(500);
  get_rain_status();
  delay(500);
  Calc_and_Analysis();
  delay(500);
}

   
/*-----------------------------------------------------------BMP180 UDF----------------------------------------------------------*/
void get_BMP180_Values(void)
{

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

/*----------------------------------------------------------DHT22 UDF----------------------------------------------------*/  
void get_DHT22_Values(void)
{
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

/*----------------------------------------------------Raindrop Sensor UDF---------------------------------------------------------*/  
void get_rain_status(void)
{
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

/*-----------------------------------------------------------Email Tx UDF----------------------------------------------------------*/  
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

    Serial.println("Sending status: ");

    Serial.println(resp.status);
    Serial.println(resp.code);
    Serial.println(resp.desc);
}

/*------------------------------------------------------WiFi Connect UDF------------------------------------------------*/  
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

/*-----------------------------------WiFi Connection sub UDF-----------------------------*/  
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

/*-----------------------------------Analysis of Calculations UDF-----------------------------*/  

void Calc_and_Analysis(void)
{
  //get fog and dew point temperature 
  FogStatus();

  //calculate heat index
  CalcHeatIndex();

  //output line for spacing 
  Serial.println(" ");

  //alert user if there are any changes
  WeatherAlert();
}

/*-----------------------------------Calculate Fog and Dew Point UDF-----------------------------*/  
void FogStatus(void)
{
  Tdew = temperature-((100-humidity)/5); 
  Serial.println("Dew Point Temperature: " + String(Tdew)+ " C");
  if ((temperature <= Tdew+2.5) && (temperature >= Tdew-2.5))
  {
    Serial.println("Fog Detected");
  }
  else
  {
    Serial.println("No Fog Detected");
  }
}

/*-----------------------------------Send Weather Updates UDF-----------------------------*/  
void CalcHeatIndex(void)
{
  //get initial values
  float tempInF = (temperature*1.8)+32;
  float c[] = {-42.379, 2.04901523, 10.14333127,-0.22475541, -0.00683783, -0.05481717, 0.00122874, 0.00085282, -0.00000199};
  float tempSquared = tempInF*tempInF;
  float humSquared = humidity*humidity;
  float HeatIndexInF = 0;

  //calculate Head Index in F
  HeatIndexInF = c[0] + (c[1]*tempInF) + (c[2]*humidity) + (c[3]*tempInF*humidity) + (c[4]*tempSquared) + (c[5]*humSquared) + (c[6]*tempSquared*humidity) + (c[7]*tempInF*humSquared) + (c[8]*tempSquared*humSquared);

  //convert F to C
  HeatIndex = ((HeatIndexInF-32)*5)/9;

  Serial.println("Heat Index: "+String(HeatIndex)+" C");
}


/*-----------------------------------Send Weather Updates UDF-----------------------------*/  
void WeatherAlert(void)
{
  //increment samples (to verify if we have enough samples to average or not)
  if (samples <=5)
  {
    samples++;
  }
  
  //add to rain and fog averages
  int i = 0; 

  //append new value to array
  for (i = 4; i >= 0; i--)
  {
    //if this is the first value, add newly recorded value
    if (i == 0)
    {
      FogStat[i] = fog;
      RainStat[i] = valRain;
    }

    //otherwise, shift current position back
    else 
    {
      //shift one element up
       FogStat[i] = FogStat[i-1];
       RainStat[i] = RainStat[i-1];
    }
  }

  //initialize variables
  int FogAvg = 0, RainAvg = 0, FogCount = 0, RainCount = 0;

  //if we have taken at least 5 samples
  if (samples >= 5)
  {
    //sum the values
    for (i = 0; i < 4; i++)
    {
      if (FogStat[i] == 1)
      {
        FogCount++;
      }
      if (RainStat[i] == 1)
      {
        RainCount++;
      }
    }
    
    //determine output value via majority wins
    if (RainCount >=3)          //rain value
    {
      RainAvg = 1;
    }
    else
    {
      RainAvg = 0;
    }
    if (FogCount >= 3)        //fog value
    {
      FogAvg = 1;
    }
    else
    {
      FogAvg = 0;
    }
  }

  //if 5 samples have not been taken yet, just assign the newly recorded value
  else
  {
    RainAvg = valRain; 
    FogAvg = fog;
  }

  /*--flags implemented to avoid sending the same email every iteration --*/
  
  //check for fog
  if ((FogAvg == 1) && (FogFlag == 0))
  {
    SendEmailAlert("WEATHER ALERT: FOG", "Fog has been detected within your area. Due to thsi fog, there is limited visibility. Drive with caution!"); 
    FogFlag = 1;
    delay(10000);
  }

  //check for rain 
  if ((RainAvg == 0) && (RainFlag == 0))
  { 
    SendEmailAlert("WEATHER ALERT: RAIN", "Rain has been detected. Bring an umbrella!");
    RainFlag = 1;
    delay(10000);
  }
  
  //check for extreme heat 
  if (((temperature >= 37.78) || (HeatIndex >= 40)) && (HeatFlag == 0))
  {
    char emailTxString[200];
    sprintf(emailTxString, "Dangerously hot temperature of %0.2f C with a heat index of %0.2f C. Stay indoors and drink plenty of water.", temperature, HeatIndex);
    SendEmailAlert("WEATHER ALERT: EXTREME HEAT", emailTxString);
    HeatFlag = 1;
    delay(10000);
  }

  //check to see if freezing 
  if ((temperature <= 0) && (FreezeFlag == 0))
  {
    SendEmailAlert("WEATHER ALERT: BELOW FREEZING", "Temperature is at or below freezing. Ice is possible. Drive with caution!");
    FreezeFlag = 1;
    delay(10000);
  }

  //check for extreme cold 
  if ((temperature <= -17.778) && (ExtremeColdFlag == 0))
  {
    char emailTxCold[100];
    sprintf(emailTxCold, "Extreme cold! Temperature is at %0.2f C! Stay indoors, and bring animals inside!", temperature);
    SendEmailAlert("WEATHER ALERT: EXTREME COLD", emailTxCold);
    ExtremeColdFlag = 1;
    delay(10000);
  }

  //clear flags if possible 
  if (temperature >= 2)                                   //freeze flag
  {
    FreezeFlag = 0;
  }
  if (temperature >= -15)                                 //extreme cold flag
  {
    ExtremeColdFlag = 0;
  }
  if ((temperature <= 35) && (HeatIndex <= 38))            //extreme heat flag        
  {
    HeatFlag = 0;
  }
  if (FogAvg == 0)                                        //fog flag
  {
    FogFlag = 0;
  }
  if (RainAvg == 1)                                       //rain flag
  {
    RainFlag = 0;
  }


  //if end of the day, print to user
  
  
}
