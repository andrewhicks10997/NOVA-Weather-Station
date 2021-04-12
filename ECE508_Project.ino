/**************************************************************************
 ECE508 Project
 Group 8 - Weather Station
 Modules Used - BMP180, Raindrop Sensor, DHT22, BN-220(GPS), OLED
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_BMP085.h> //Library for BMP180 (Need to download zip file from here: https://github.com/adafruit/Adafruit-BMP085-Library)

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These pressure and temperature sensors use I2C to communicate, 2 pins
  are required to interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

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

/*********Setting up the Raindrop Sensor***************/
int inputPinRain = 3;
int valRain = 0;                    // variable for reading the pin status
  
void setup() {
  Serial.begin(9600);
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
