# REAL-TIME-RISK-ALERT-SYSTEM-FOR-OLIGURIC-DIALYSIS-PATIENTS-USING-ARDUINO
The Real-Time Risk Alert System for Oliguric Dialysis Patients Using Arduino monitors vital signs like temperature, heartbeat, and pressure in patients with low urine output. It provides real-time data and sends instant alerts via GSM if abnormal values are detected, ensuring quick medical attention and enhancing patient safety.

#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

int show = -1;
int RXPin = 4;
int TXPin = 3;

SoftwareSerial sSerial(RXPin, TXPin);
#define BMP_SCL 13
#define BMP_SDO 12
#define BMP_SDA 11
#define BMP_CSB1 10

Adafruit_BMP280 bmp1(BMP_CSB1, BMP_SDA, BMP_SDO, BMP_SCL);
// 2 custom characters

byte dotOff[] = { 0b00000, 0b01110, 0b10001, 0b10001,
                  0b10001, 0b01110, 0b00000, 0b00000 };
byte dotOn[] = { 0b00000, 0b01110, 0b11111, 0b11111,
                 0b11111, 0b01110, 0b00000, 0b00000 };
// int pin1 = 7;
 float n1,n2;
 int pin = 2;//HB
 int pin_irq = 0; //IRQ that matches to pin 2
 int be;
 volatile int IRQcount; 
 int pin1 = 5;



 void IRQcounter() {
      IRQcount++;
    }
void setup()  
{
if (!bmp1.begin()) {
   // Serial.println("Sensor BMP280 device 1 was not found.");
    //lcd.setCursor(0,1);
    //lcd.print("not found");
    while (1);
  }
  //Serial.println("Initialize BMP280 1 completed.");
  //lcd.setCursor(0,1);
  //lcd.print("found");
  delay(2000);
  
int error;
pinMode(pin1, OUTPUT);
digitalWrite(pin1, LOW);
  Serial.begin(9600);
   sSerial.begin(9600);
  Serial.println("LCD...");

  // wait on Serial to be available on Leonardo
  while (!Serial)
    ;

  Serial.println("Probing for PCF8574 on address 0x27...");

  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.print(error);

  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(20, 4);  // initialize the lcd

    lcd.createChar(1, dotOff);
    lcd.createChar(2, dotOn);

  } else {
    Serial.println(": LCD not found.");
  }  // if
    lcd.setBacklight(255);
    lcd.home();
    lcd.clear();
    lcd.print("smart health ");
    lcd.setCursor(0, 1);
    lcd.print("care using iot");
    delay(2000);
//    lcd.clear();
//    lcd.print(" using iot ");
//    lcd.setCursor(0, 1);
//    delay(2000);
    attachInterrupt(pin_irq, IRQcounter, RISING);


    }
     void loop()                     // run over and over again
    {

       float pressure = bmp1.readPressure() / 100.0;
  pressure= pressure-981;

  
    float g=analogRead(A1);
    g=70*(g/1023);

    float p=analogRead(A0);
    p=100*(p/1023); 
    lo=lo+1;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(g);
    lcd.print(" HB:");
    lcd.print(p);
    lcd.setCursor(0, 1);
    lcd.print("UF:");
    lcd.print(be);
    lcd.print("  UP:");
    lcd.print(pressure);
    delay(1000);
    Serial.print(g);
    Serial.print(',');
      Serial.print(p);//pressure
    Serial.print(',');
      Serial.print(be);
    Serial.print(',');
      Serial.print(pressure);
       Serial.print(',');
    Serial.println('\n');
    delay(200);  // Pause for 5 seconds.
    if(be>4000) 
    {
    IRQcount=0;
    be=0;
    }
 
     be=IRQcount;
     delay(300); 

     if((g>45)||(p>100)||(pressure>20))
    {
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("abnormal sys... ");
  lcd.setCursor(0,1);
  digitalWrite(pin1,HIGH);
  sSerial.println("AT");    //Sets the GSM Module in Text Mode
  delay(2000);  // Delay of 1000 milli seconds or 1 second
  sSerial.println("AT+CMGF=1"); 
  delay(2000);
  sSerial.println("AT+CMGS=\"8248555869\"\r"); // Replace x with mobile number
  delay(2000);
   sSerial.print("abnormal system ");
   sSerial.print("T: ");
    sSerial.print(g);
    sSerial.print(" HB: "); 
    sSerial.print(p);
     sSerial.print("  UF: "); 
      sSerial.print(be);
      sSerial.print("   UP: "); 
      sSerial.print(pressure);
delay(1000);
  sSerial.println((char)26);// ASCII code of CTRL+Z
 delay(500); 

  
  delay(1000);
    }
    else
    {  
  digitalWrite(pin1,LOW);
  delay(1000);
    }


   }
