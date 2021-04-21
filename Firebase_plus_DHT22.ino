
#include "Firebase_Arduino_WiFiNINA.h"
#include <SimpleDHT.h>//install DHT Library and Adafruit Unified Sensor Library


#define FIREBASE_HOST "ece508-weatherstation-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "LqokKpNxTtrTxz160a64aOPiZRe8ht0BiqMMItKe"
#define WIFI_SSID "ShrekIsLoveShrekIsLife"
#define WIFI_PASSWORD "VooDoo1738Ranger1969!"

#define DHTPIN 4   // Connect Data pin of DHT to D2
int led = 14;     // Connect LED to D5
int n = 1;

SimpleDHT22 dht22(DHTPIN);
//SimpleDHT22 dht22(pinDHT22);
float temperature = 0;
float humidity = 0;
int errDHT22 = SimpleDHTErrSuccess;
String path = "/TEST/temperature";


FirebaseData firebaseData;
FirebaseData ledData;

//FirebaseJson json;


void setup()
{

  Serial.begin(9600);

  pinMode(led,OUTPUT);
  
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

}

void sensorUpdate(int n){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  errDHT22 = dht22.read2(&temperature, &humidity, NULL);
  if (errDHT22 != 0) {
    temperature = (-1 - 32)/1.8;
    humidity = -1;
  }
//
//  float h = humidity();
//  // Read temperature as Celsius (the default)
//  float t = temperature;
//  // Read temperature as Fahrenheit (isFahrenheit = true)
//  float f = temperature;
//
//  // Check if any reads failed and exit early (to try again).
//  if (isnan(h) || isnan(t) || isnan(f)) {
//    Serial.println(F("Failed to read from DHT sensor!"));
//    return;
//  }

  Serial.print(F("Humidity: "));
  Serial.print((float)humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print((float)temperature);
//  Serial.print(F("C  ,"));
//  Serial.print(f);
//  Serial.println(F("F  "));


    if (Firebase.setInt(firebaseData, path + "/temp" + (n), temperature))
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
  

  if (Firebase.setFloat(firebaseData, "/TEST/humidity", humidity))
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
void loop() {
  sensorUpdate(n);

  if (n<=4)
  {
     n++;
  }
  else
  {
    int sum = 0;
    int avgTemp = 0;
    int i;
    int val;
    for (i=1; i<=5; i++)
    {
      if (Firebase.getInt(firebaseData, path + "/temp" + (i))) {
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
    avgTemp = sum/5;
    Firebase.setInt(firebaseData, path + "/avgTemp", avgTemp);
    for (n=1; n<=5; n++)
    {
      Firebase.deleteNode(firebaseData, path + "/temp" + (n));
    }
    n=1;
  }

  delay(5000);
}
