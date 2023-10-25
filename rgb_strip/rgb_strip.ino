#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define led 7
#define redPin 11
#define greenPin 10
#define bluePin 9

// set to false if using a common cathode LED
#define commonAnode false

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);

// our RGB -> eye-recognized gamma color
byte gammatable[256];

boolean colorDetection = true;

void setup() {
  Serial.begin(9600);
  
  if(tcs.begin()) {
    Serial.println("Connected!");
  } else {
    Serial.println("TCS34725 not found...");
    while(1); //stop here
  }

  pinMode(led, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(led, LOW);
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);

  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }

  }
}

void loop() {
  float red, green, blue;

  if(colorDetection){
    delay(75);
    tcs.getRGB(&red, &green, &blue);
    Serial.print("RED: "); Serial.println(int(red));
    Serial.print("GREEN: "); Serial.println(int(green));
    Serial.print("BLUE: "); Serial.println(int(blue));

    analogWrite(redPin, gammatable[(int)red]);
    analogWrite(greenPin, gammatable[(int)green]);
    analogWrite(bluePin, gammatable[(int)blue]);

    /* Using RawData */
    /*
      // uint16_t red, green, blue, clear;
      // tcs.getRawData(&red, &green, &blue, &clear);
      // Serial.print("RED: "); Serial.println(red);
      // Serial.print("GREEN: "); Serial.println(green);
      // Serial.print("BLUE: "); Serial.println(blue);
    */
  }

  // Get serial messages
  if(Serial.available()) {
    String message = Serial.readString();
    int value = 0;

    if(message.equalsIgnoreCase("led")) digitalWrite(led, !digitalRead(led));
    else if(message.equalsIgnoreCase("auto")) {
      colorDetection = true;
      Serial.println("Color Detection: ON");
    } 
    else if(message.equalsIgnoreCase("manual")) {
      colorDetection = false;
      Serial.println("Color Detection: OFF");
    }
    else {
      value = message.substring(1).toInt();
      if(message.startsWith("r")) analogWrite(redPin, value);
      if(message.startsWith("g")) analogWrite(greenPin, value);
      if(message.startsWith("b")) analogWrite(bluePin, value);
      Serial.print(message.substring(0,1)); Serial.print(": "); Serial.println(value);
    }

  }

}
