/*
  Author : Michael Li
  Date : 9/29/2017
  Project: Arduino based remote controlled toy car.

  Target board: Arduino UNO R3 

  Example: This code is written for the processor which transmitts
           the X and Y positions of the joystick to the toy
           car.   
  
  This example is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/
// ver 2 : Release version 0

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_Lite.h>
#include <RCSwitch.h>

#define  YRAW_MAX  1023
#define  YPOS_MAX  255
#define  XRAW_MAX  1023
#define  XPOS_MAX  255


//////////////////////////////////////////////////////
//                   Joystick                       //
//////////////////////////////////////////////////////

//Input pins
int JoyStick_X = 7; // x (A7)
int JoyStick_Y = 6; // y (A6)
int JoyStick_Z = 3; // key (D3)


//////////////////////////////////////////////////////
//                  OLED Display                    //
//////////////////////////////////////////////////////

#define OLED_RESET 4
Adafruit_SSD1306_Lite display(OLED_RESET);

//#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
//#endif

String dataline = " "; // Empty strings


//////////////////////////////////////////////////////
//                  433 Mhz RF                      //
//////////////////////////////////////////////////////
RCSwitch mySwitch = RCSwitch();

unsigned long data = 0;
unsigned int datalength = 26;
char choice;

char CR_char = 0x0d;
byte tempbuf[5];  // 1 to 175
int  buflength;
bool baddata_flag;


//////////////////////////////////////////////////////
//                  Main Program                    //
//////////////////////////////////////////////////////

unsigned long time_prev_senddata = 0;  // milli timer
const long    interval_senddata = 100;  // interval at which to blink (milliseconds)

//////////////////////////////////////////////////////
//                  Main Program                    //
//////////////////////////////////////////////////////

void setup() {

  // Joystick Setup
  pinMode (JoyStick_X, INPUT);
  pinMode (JoyStick_Y, INPUT);
  pinMode (JoyStick_Z, INPUT_PULLUP);
  mySwitch.enableTransmit(13);  // Using Pin #13 (Nano)

  // Serial Mode Setup  
  Serial.begin(9600);
  Serial.setTimeout(30000);     // 30 sec
  Serial.println("Transmitter is ready.");
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }



  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.display(); // show splashscreen
  display.display(); // show splashscreen


}

int count = 1;
int count_send = 0;
const int count_send_max = 3;

unsigned long y_position;
unsigned long x_position;
unsigned long tempdata;
unsigned long senddata;
  
void loop() {
  unsigned long current_time = millis();

  if ((current_time - time_prev_senddata) > interval_senddata) {
    //update_oled_display (count, 0, 0, 0);
    //mySwitch.send(count,datalength);     
    if (x_position == 0) {
      x_position = 1;
    }
    if (y_position == 0) {
      y_position = 1;
    }
    update_oled_display (x_position, y_position);

    senddata = (x_position * 256) + y_position; 
    Serial.println(x_position);
    Serial.println(y_position);  
    Serial.println(senddata);  
    mySwitch.send(senddata,datalength);      
    if (count_send < count_send_max) {
       count_send++;
    } else {
      count_send = 0;
      count++;  
      tempdata = analogRead (JoyStick_Y);
      y_position = map (tempdata, 0, YRAW_MAX, 0, YPOS_MAX);
      tempdata = analogRead (JoyStick_X);
      x_position = map (tempdata, 0, XRAW_MAX, 0, XPOS_MAX);
    }
    
    time_prev_senddata = current_time;
  }
  
}

void update_oled_display (int xpos, int ypos) {
 // print out the value you read:
  display.display();
  display.clearDisplay();   // clears the screen and buffer

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);  
  dataline = String("v1 X Position = " + String(xpos));
  Serial.println(dataline);  
  display.println(dataline);
  dataline = String("v1 Y Position = " + String(ypos));
  Serial.println(dataline);  
  display.println(dataline);
  
  display.display();
}
