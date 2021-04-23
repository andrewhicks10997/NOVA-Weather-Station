
#include "Firebase_Arduino_WiFiNINA.h"
#include <SimpleDHT.h>//install DHT Library and Adafruit Unified Sensor Library
#include <WiFiNINA.h>
#include <time.h>


#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

#include <SPI.h>
#include <Wire.h>

int inputPinRain = 3;
int valRain = 0;                    // variable for reading the pin status


#define FIREBASE_HOST "ece508-weatherstation-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "LqokKpNxTtrTxz160a64aOPiZRe8ht0BiqMMItKe"
#define WIFI_SSID "ShrekIsLoveShrekIsLife"
#define WIFI_PASSWORD "VooDoo1738Ranger1969!"

#define DHTPIN 4   // Connect Data pin of DHT to D2
int led = 14;     // Connect LED to D5
int n = 1;

int dd, hh, mm, ss;
char lcdBuffer[20];
unsigned long currSeconds;
char currentTimeET[20];
unsigned long currMillis;
String stringOne;

SimpleDHT22 dht22(DHTPIN);
//SimpleDHT22 dht22(pinDHT22);
float temperature = 0;
float humidity = 0;
int errDHT22 = SimpleDHTErrSuccess;
float pressure = 0;
String path = "/TEST";


FirebaseData firebaseData;
FirebaseData ledData;

//FirebaseJson json;


void setup()
{

  Serial.begin(9600);

  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  String path = "/TEST";
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);

  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }

}

void sensorDHT22Update(int n){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  errDHT22 = dht22.read2(&temperature, &humidity, NULL);
  if (errDHT22 != 0) {
    temperature = (-1 - 32)/1.8;
    humidity = -1;
  }


  Serial.print(F("Humidity: "));
  Serial.print((float)humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print((float)temperature);



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

void sensorBMP085Update(int n){
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    pressure = bmp.readPressure();

    if (Firebase.setInt(firebaseData, path+ "/pressure/press"+(n), pressure))
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

void sensorRainUpdate(char* r){

    valRain = digitalRead(inputPinRain);

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

void avgTempfun() {
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
        Serial.println(val);
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


void avgHumfun() {
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
        Serial.println(val);
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

void avgPressfun() {
  int sum = 0;
  int avgPress = 0;
  int i;
  int val;
  int j;
  for (i=1; i<=n; i++)
  {
    if (Firebase.getInt(firebaseData, path + "/pressure/press" + (i))) {
      if (firebaseData.dataType() == "int") {
        val = firebaseData.intData();
        Serial.println(val);
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

void convCurrentTimeET(unsigned long currSeconds, char *currentTimeET) 
{
    time_t rawtime = currSeconds - 18000;
    struct tm  ts;
    char buf[70];
    ts = *localtime(&rawtime);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ts);
    sprintf(currentTimeET, buf);
};

void loop() {
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

  delay(1000);
}
