#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "WiFi.h"
#include "wifi_credentials.h"
#include <Espalexa.h>

#define led 17
#define redPin 19
#define greenPin 18
#define bluePin 16

// set to false if using a common cathode LED
#define commonAnode false

// objects
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
Espalexa espalexa;

// our RGB -> eye-recognized gamma color
byte gammatable[256];
float red, green, blue;

// set default function to detect color
boolean colorDetection = true;

// set ESP32 PWM parameters
const int freq = 5000;
const int resolution = 8;
const int redChannel = 0;
const int greenChannel = 2;
const int blueChannel = 4;

// prototypes
boolean connectWifi();
boolean wifiConnected = false;

//callback functions
void ledStripChanged(uint8_t brightness, uint32_t rgb);

void setup() {
  Serial.begin(9600);
  delay(200); 

  pinMode(led, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(led, LOW);
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);

  // set ESP32 PWM
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(greenChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);
  ledcAttachPin(redPin, redChannel);
  ledcAttachPin(greenPin, greenChannel);
  ledcAttachPin(bluePin, blueChannel);
  
  // check TCS34725 connection
  if(tcs.begin()) {
    Serial.println("Connected!");
  } else {
    while (1) {
      Serial.println("TCS34725 not found. Please check connections and reset the ESP.");  
      delay(2500);
    }
  }

  // initialise wifi connection
  wifiConnected = connectWifi();

  if(wifiConnected) {

    // define devices
    espalexa.addDevice("Fita LED", ledStripChanged);
    espalexa.begin();

  } else {
    while (1) {
      Serial.println("Cannot connect to WiFi. Please check data and reset the ESP.");
      delay(2500);
    }
  }

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
  espalexa.loop();

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
  } else {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
  }

  // Get serial messages
  if(Serial.available()) {
    String message = Serial.readString();
    int value = 0;

    if(message.equalsIgnoreCase("led")) digitalWrite(led, !digitalRead(led));
    else if(message.equalsIgnoreCase("auto")) {
      colorDetection = true;
      Serial.println("Color Detection: ON");
    } else if(message.equalsIgnoreCase("manual")) {
      colorDetection = false;
      Serial.println("Color Detection: OFF");
    } else {
      value = message.substring(1).toInt();
      if(message.startsWith("r")) analogWrite(redPin, value);
      if(message.startsWith("g")) analogWrite(greenPin, value);
      if(message.startsWith("b")) analogWrite(bluePin, value);
      Serial.print(message.substring(0,1)); Serial.print(": "); Serial.println(value);
    }

  }

}

// callback functions
void ledStripChanged(uint8_t brightness, uint32_t rgb) {

  if(brightness) {
    Serial.println("LED ON");
    Serial.print("Brightness: "); Serial.println(brightness);

    if(rgb == 16753231) {
      colorDetection = true;
      Serial.println("Color Detection: ON");
    } else {
      colorDetection = false;
      red = float((rgb >> 16) & 0xFF);
      green = float((rgb >>  8) & 0xFF);
      blue = float(rgb & 0xFF);
      Serial.println("Color Detection: OFF");
      Serial.print("RGB: "); Serial.println(rgb);
      Serial.print("R: "); Serial.println(red);
      Serial.print("G: "); Serial.println(green);
      Serial.print("B: "); Serial.println(blue);
    }

  } else {
    colorDetection = false;
    red, green, blue = 0.0;
    Serial.println("LED OFF");
  }

}

// connect to wifi – returns true if successful or false if not
boolean connectWifi() {
  boolean state = true;
  int i = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(Wifi_SSID, Wifi_PASS);
  Serial.println("");
  Serial.println("Connecting to WiFi");

  // Wait for connection
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (i > 20) {
      state = false;
      break;
    }
    i++;
  }
  Serial.println("");
  if (state) {
    Serial.print("Connected to ");
    Serial.println(Wifi_SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Connection failed.");
  }
  return state;
}
