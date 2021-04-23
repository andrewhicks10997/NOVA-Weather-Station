/**************************************************************************
 ECE508 Project
 Group 8 - Weather Station
 Modules Used - BMP180, Raindrop Sensor, DHT22, BN-220(GPS), OLED
 **************************************************************************/
#include <SPI.h>                        //SPI communication library
#include <Wire.h>                       //I2C communication library
#include <Adafruit_BMP085.h>            //Library for BMP180 (Need to download zip file from here: https://github.com/adafruit/Adafruit-BMP085-Library)
#include <TinyGPS++.h>                  //library for GPS
#include <Adafruit_GFX.h>               //header file for OLED graphics
#include <Adafruit_SSD1306.h>           //header file for OLED graphics
#include <time.h>                       //library for time functions
#include <EMailSender.h>                //library for SMTP communication
#include <WiFiNINA.h>                   //library for WiFi connectivity
#include "Firebase_Arduino_WiFiNINA.h"  //library for Firebase 
#include "SETUP_INFO.h"               //setup header file 

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4


/*********Setting up the Barometer***************/
Adafruit_BMP085 bmp; //Using the BMP180, but this will still work
int Pressure = 0;
int Altitude = 0;

/*********Setting up the GPS Module***************/
TinyGPSPlus gps;

/*********Setting up the Temp Sensor (DHT22)***************/
#include <SimpleDHT.h> // headers for DHT_22
int pinDHT22 = 2; //setting pin D2 for sensor
SimpleDHT22 dht22(pinDHT22);
float temperature = 0;
float humidity = 0;
int errDHT22 = SimpleDHTErrSuccess;

/************Setup FireBase****************************/
FirebaseData firebaseData;
FirebaseData ledData;
int n = 1;
char currentTimeET[20];
unsigned long currMillis;
String stringOne;
String path = FirebasePath;

/*_____________Setup Time Functionality*****************/
int dd, hh, mm, ss;
unsigned long currSeconds;

/*********Setting up the Email Functions***************/
EMailSender emailSend("NOVAWeatherStation@gmail.com", "ECE508IoTWeatherStation123!");     //signing into client email (address and password)
uint8_t connection_state = 0;
uint16_t reconnect_interval = 10000;

/*********Setting up the Raindrop Sensor***************/
int inputPinRain = 3;
int valRain = 0;                    // variable for reading the pin status

/*********Setting up the OLED display***************/
#define SCREEN_WIDTH 128                    //OLED display width in pixels
#define SCREEN_HEIGHT 64                    //OLED display height in pixels
#define OLED_RESET 4                        //Reset pin for OLED
String oledline[9];                           //line of strings written to  
Adafruit_SSD1306 myOled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);      //initialize class for OLED

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
void get_BMP180_Values(void);                                     //BMP180 Sensor Read
void get_DHT22_Values(void);                                      //DHT22 Sensor Read
void get_rain_status(void);                                       //Raindrop Sensor Read
void updateOLED(void);                                            //Update OLED with new Data
void displayTextOLED(String oledline[]);                          //writes values to OLED
uint8_t WiFiConnect(const char* nSSID, const char* nPassword);    //connects to WiFi
void Awaits();                                                    //timeout functionality for network connectivity
void SendEmailAlert(char subject[], char contents[]);             //sends email with specified inputs
void Calc_and_Analysis(void);                                     //calculates averages of values and analyzes them (determines if the user should be emailed)
void FogStatus(void);                                             //returns if there is fog and the dew point temperature
void CalcHeatIndex(void);                                         //calculates the heat index
void WeatherAlert(void);                                          //Determines if there is a weather alert
void ConnectFirebase(void);                                       //connects to Google Firebase
void OffloadToFirebase(void);                                     //Offloads current data to Firebase
void sensorDHT22Update(int n);                                    //updated DHT22 data to firebase
void sensorBMP085Update(int n);                                   //updated BMP085 data to firebase
void sensorRainUpdate(char* r);                                   //updated Raindrop sensor value
void avgTempfun(void);                                            //averages temperatures recorded and sends to firebase
void avgHumfun(void);                                             //averages humidity recorded and sends to firebase
void avgPressfun(void);                                           //averages pressure recorded and sends to firebase

/*---------------------------------------------------------------SETUP FUNCTION-----------------------------------------------------*/  
void setup() {
  Serial.begin(9600);

  //connect to the firebase
  ConnectFirebase();
  
  //begin using the bmp module
  if (!bmp.begin()) 
  {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  }

  if(!myOled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
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
  updateOLED();
  delay(500);
  OffloadToFirebase();
  delay(500);
}

   
/*-----------------------------------------------------------BMP180 UDF----------------------------------------------------------*/
void get_BMP180_Values(void)
{

    Serial.println("BMP180 INFO: ");
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Pressure = bmp.readPressure();
    Serial.print("Pressure = ");
    Serial.print(String(Pressure));
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // Pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level Pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Altitude =  bmp.readAltitude(101500);
    Serial.print("Real altitude = ");
    Serial.print(String(Altitude));
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

/*----------------------------------------------------GPS initialization UDF---------------------------------------------------------*/

/*----------------------------------------------------GPS Sensor UDF---------------------------------------------------------*/

/*----------------------------------------------------Update OLED UDF---------------------------------------------------------*/
void updateOLED(void)
{
  //update the OLED screen
  oledline[1] = "NOVA Weather Station";
  oledline[2] = "Temp: " + String(temperature) + " C"; 
  oledline[3] = "Hum in RH%: " + String(humidity);
  oledline[4] = "Pressure: " + String(Pressure) + " Pa";
  oledline[5] = "Altitude: " + String(Altitude) + " m";
  oledline[6] = "Heat Index: " + String(HeatIndex) + " C";
  if (fog == 1)
  {
    oledline[7] = "Foggy"; 
  }
  else
  {
    oledline[7] = "No Fog";
  }
  if (valRain == 1)
  {
    oledline[8] = "No Rain";
  }
  else
  {
    oledline[8] = "Raining";
  }
  
  //display string array to OLED
  displayTextOLED(oledline);
}

/*----------------------------------------------------display text to OLED UDF---------------------------------------------------------*/
void displayTextOLED(String oledline[])
{
  int jj;
  myOled.clearDisplay();
  myOled.setTextSize(1);
  myOled.setTextColor(SSD1306_WHITE);
  myOled.setCursor(0, 0);

  for (jj=1; jj<=8; jj++) { 
    myOled.println(oledline[jj]);
    }
  
  myOled.display();  
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
  //print to user which section this comes from
  Serial.println("CALCULATED INFO");
  
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

/*-----------------------------------Calculate Heat Index UDF-----------------------------*/  
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

/*------------------------------------------------------Firebase Connection UDF----------------------------------*/
void ConnectFirebase(void)
{
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, NETWORK_SSID, NETWORK_PSA);
  Firebase.reconnectWiFi(true);
}

/*------------------------------------------------------Firebase data offload UDF----------------------------------*/
void OffloadToFirebase(void) 
{
  sensorDHT22Update(n);
  sensorBMP085Update(n);

  currSeconds = WiFi.getTime();
  convCurrentTimeET(currSeconds, currentTimeET);

  sensorRainUpdate(currentTimeET);

  if (n<=10)
  {
     n++;
  }
  else
  {
      avgTempfun();
      avgHumfun();
      avgPressfun();
      n=1;
  }
}

/*------------------------------------------------------Update DHT22 data UDF----------------------------------*/
void sensorDHT22Update(int n)
{
    if (Firebase.setInt(firebaseData, path + "/temperature/temp" + (n), temperature))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + firebaseData.dataPath());
      Serial.println("TYPE: " + firebaseData.dataType());
      Serial.println("------------------------------------");
      Serial.println();
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
    }
  

  if (Firebase.setInt(firebaseData, path+ "/humidity/hum"+(n), humidity))
  {
    Serial.println("PASSED");
    Serial.println("PATH: " + firebaseData.dataPath());
    Serial.println("TYPE: " + firebaseData.dataType());
    Serial.println("------------------------------------");
    Serial.println();
  }
  else
  {
    Serial.println("FAILED");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
}

/*------------------------------------------------------Update BMP085 data UDF----------------------------------*/
void sensorBMP085Update(int n)
{
    if (Firebase.setInt(firebaseData, path+ "/Pressure/press"+(n), Pressure))
    {
      Serial.println("PASSED");
      Serial.println("PATH: " + firebaseData.dataPath());
      Serial.println("TYPE: " + firebaseData.dataType());
      Serial.println("------------------------------------");
      Serial.println();
    }
    else
    {
      Serial.println("FAILED");
      Serial.println("REASON: " + firebaseData.errorReason());
      Serial.println("------------------------------------");
      Serial.println();
    }
    
}

/*------------------------------------------------------Update raindrop sensor data UDF----------------------------------*/
void sensorRainUpdate(char* r)
{
    stringOne= r;
    stringOne += ":  It is Raining";
    
    if (valRain == LOW){
      if (Firebase.setString(firebaseData, path+ "/precipitation", stringOne))
      {
        Serial.println("PASSED");
        Serial.println("PATH: " + firebaseData.dataPath());
        Serial.println("TYPE: " + firebaseData.dataType());
        Serial.println("------------------------------------");
        Serial.println();
      }
      else
      {
        Serial.println("FAILED");
        Serial.println("REASON: " + firebaseData.errorReason());
        Serial.println("------------------------------------");
        Serial.println();
      }
    }
    
}

/*------------------------------------------------------send average temperature to firebase UDF----------------------------------*/
void avgTempfun(void)
{
  int sum = 0;
  int avgTemp = 0;
  int i;
  int val;
  int j;
  for (i=1; i<=n; i++)
  {
    if (Firebase.getInt(firebaseData, path + "/temperature/temp" + (i))) {
      if (firebaseData.dataType() == "int") {
        val = firebaseData.intData();
      }
    } else {
      //Failed, then print out the error detail
      Serial.println(firebaseData.errorReason());
    }
    sum = sum + val;
  }
  
  avgTemp = sum/n;
  Firebase.setFloat(firebaseData, path + "/avgTemp", avgTemp);
  
  for (j=1; j<=n; j++)
  {
    Firebase.deleteNode(firebaseData, path + "/temperature/temp" + (j));
  }
}


/*------------------------------------------------------Send average humidity to firebase UDF----------------------------------*/
void avgHumfun(void)
{
  int sum = 0;
  int avgHum = 0;
  int i;
  int val;
  int j;
  for (i=1; i<=n; i++)
  {
    if (Firebase.getInt(firebaseData, path + "/humidity/hum" + (i))) {
      if (firebaseData.dataType() == "int") {
        val = firebaseData.intData();
      }
    } else {
      //Failed, then print out the error detail
      Serial.println(firebaseData.errorReason());
    }
    sum = sum + val;
  }
  
  avgHum = sum/n;
  Firebase.setFloat(firebaseData, path + "/avgHum", avgHum);
  
  for (j=1; j<=n; j++)
  {
    Firebase.deleteNode(firebaseData, path + "/humidity/hum" + (j));
  }
}

/*------------------------------------------------------Send average Pressure to firebase UDF----------------------------------*/
void avgPressfun(void)
{
  int sum = 0;
  int avgPress = 0;
  int i;
  int val;
  int j;
  for (i=1; i<=n; i++)
  {
    if (Firebase.getInt(firebaseData, path + "/Pressure/press" + (i))) {
      if (firebaseData.dataType() == "int") {
        val = firebaseData.intData();
      }
    } else {
      //Failed, then print out the error detail
      Serial.println(firebaseData.errorReason());
    }
    sum = sum + val;
  }
  
  avgPress = sum/n;
  Firebase.setFloat(firebaseData, path + "/avgPress", avgPress);
  
  for (j=1; j<=n; j++)
  {
    Firebase.deleteNode(firebaseData, path + "/pressure/press" + (j));
  }
}

/*------------------------------------------------------Get time UDF----------------------------------*/
void convCurrentTimeET(unsigned long currSeconds, char *currentTimeET) 
{
    time_t rawtime = currSeconds - 18000;
    struct tm  ts;
    char buf[70];
    ts = *localtime(&rawtime);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ts);
    sprintf(currentTimeET, buf);
}
