/*
 * 
  Author : Michael Li
  Date : 9/29/2017
  Project: Arduino based remote controlled toy car.

  Target board: Arduino UNO R3 

  Example: This code is written for the processor which recieves
           the X and Y positions of the joystick from the remote
           control module.   The processor will translate the 
           positions to contorl the two DC motors' speed and direction.
  
  This example is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  
*/

// ver 4  : Release version 0

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_Lite.h>  // Display library

#include <Wire.h>      
#include <RCSwitch.h>  // Wireless library
#include <Servo.h>     // not used at this point

//////////////////////////////////////////////////////
//     Pin Assignment (Arduino UNO R3)              //
//////////////////////////////////////////////////////
#define rf_intr_pin  0  //  pin 2 (interrupt 0)
#define servopin     8

// motor control pin assignment
// Left Wheel DC Motor
int DCL_speed_pin = 5;  // speed
int DCL_dir_pin = 4;  // direction

// Righ Wheel DC Motor
int DCR_speed_pin = 6;  // speed
int DCR_dir_pin = 7;  // direction

int trigOPin_front = 9; 
int echoIPin_front = 10; 

int trigOPin_back = 11; 
int echoIPin_back = 12; 

//////////////////////////////////////////////////////
//                Distance Sensor                   //
//////////////////////////////////////////////////////
unsigned long duration;
int         distance_front_data;
int         distance_back_data;
int         prev_distance_front;
int         prev_distance_back;
int         now_distance_front;
int         now_distance_back;
const int   distance_error = 10;


boolean       distfront_confirm_flag = false;  // confirm that data is valid and repeated 2x
boolean       distfront_process_flag = false;  // shows that the data has been processed.
int           distfront_repeat_count = 0;
const int     distfront_repaat_count_min = 1;

boolean       distback_confirm_flag = false;  // confirm that data is valid and repeated 2x
boolean       distback_process_flag = false;  // shows that the data has been processed.
int           distback_repeat_count = 0;
const int     distback_repaat_count_min = 1;

boolean       forward_brake_flag = false;      // If the car is too close to the wall or objects, enable this flag
boolean       backward_brake_flag = false;      // If the car is too close to the wall or objects, enable this flag

#define IGNORE_DISTANCE 5.0  // Error distance or far
#define CLEAR_DISTANCE 60.0  // save distance to go further
//#define CLEAR_DISTANCE 20.0  // save distance to go further
//////////////////////////////////////////////////////
//                  Servo Motor                     //
//////////////////////////////////////////////////////
Servo myservo;  // create servo object to control a servo


//////////////////////////////////////////////////////
//                  OLED Display                    //
//////////////////////////////////////////////////////

#define OLED_RESET 4
Adafruit_SSD1306_Lite display(OLED_RESET);

//#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306_Lite.h!");
//#endif

String dataline = " "; // Empty strings

//////////////////////////////////////////////////////
//                  433 Mhz RF                      //
//////////////////////////////////////////////////////
RCSwitch mySwitch = RCSwitch();


//////////////////////////////////////////////////////
//                  Motor Position                  //
//////////////////////////////////////////////////////
int pos = 0;    // variable to store the servo position

#define  YPOS_MAX  255
#define  YPOS_MID  124
#define  XPOS_MAX  255
#define  XPOS_MID  124


// speed was 155
#define MAX_SPEED    100    
#define DIR_FORWARD  0
#define DIR_BACKWARD 1
#define DIR_LEFT     0
#define DIR_RIGHT    1
#define MAX_SPEED_SLOWDOWN 40

unsigned long prevvalue = 0;
unsigned long nowvalue;
boolean       confirm_flag = false;  // confirm that data is valid and repeated 2x
boolean       process_flag = false;  // shows that the data has been processed.
unsigned long data = 0;
unsigned long sensorValue_x;  // speed to reduce for right or left turn
unsigned long sensorValue_y;  // speed without turn

int           sensorValue;  // joystick position (0~127) or (128~255)
int           speedValue;   // motor speed (0~255)
int           speedsubValue;  // the motor speed value to reduce speedValue
int           speedDir;     // forward or backward
int           turnDir;      // left or right

int           speedValueL;  // left wheel speed
int           speedValueR;  // right wheel speed

int           repeat_count = 0;
int           repaat_count_min = 1;




//////////////////////////////////////////////////////
//                  Global Variables                //
//////////////////////////////////////////////////////

unsigned long time_prev_chkdist = 0;   // milli timer
const long    interval_chkdist = 100;  // interval at which to blink (milliseconds)
boolean       scan_state = true;       // select which distance sensor to read.

//////////////////////////////////////////////////////
//                  Main Program                    //
//////////////////////////////////////////////////////


void setup()
{
  // DC Motor control pins.
  pinMode(DCL_dir_pin, OUTPUT);
  pinMode(DCR_dir_pin, OUTPUT);

  // Distance sensor in/out pins
  pinMode(trigOPin_front, OUTPUT);
  pinMode(echoIPin_front, INPUT);
  pinMode(trigOPin_back, OUTPUT);
  pinMode(echoIPin_back, INPUT);
  process_distance_sensor_front();  
  process_distance_sensor_back();

  mySwitch.enableReceive(rf_intr_pin);  // Receiver on interrupt 0 => that is pin #2

  myservo.attach(servopin);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);         // make the distance senser looking straight ahead.
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.display(); // show splashscreen
  display.display(); // show splashscreen

  Serial.println("Reciever is ready.");

}


void loop() {
  if (mySwitch.available()) {
      process_motor();
  }  
  unsigned long current_time = millis();

  if ((current_time - time_prev_chkdist) > interval_chkdist) {
    time_prev_chkdist = current_time;  
    if (scan_state) {
      process_distance_sensor_front();  
      scan_state = false;
    } else {
      process_distance_sensor_back();
      scan_state = true;
    }
    if (distance_front_data > IGNORE_DISTANCE && distance_front_data < CLEAR_DISTANCE && speedDir == DIR_FORWARD) {  // Don't go any further.             
       forward_brake_flag = true;
    } 
    if (distance_back_data > IGNORE_DISTANCE && distance_back_data < CLEAR_DISTANCE && speedDir == DIR_BACKWARD) {  // Don't go any further.        
       backward_brake_flag = true;
    }
    if (forward_brake_flag | backward_brake_flag) {
       speedValue = 0;         
       speedValueL = speedValue; 
       speedValueR = speedValue; 
       drive_motor(speedDir,speedValueL,speedValueR); 
    }              
    update_oled_display(sensorValue_x, sensorValue_y, speedDir, speedValue, distance_front_data,distance_back_data,forward_brake_flag,backward_brake_flag);

  }  
}


void process_motor() {
    nowvalue = mySwitch.getReceivedValue();
    
    if (confirm_flag == true && process_flag == false) { 
        sensorValue_y = data & 0x000000FF;
        sensorValue_x = (data & 0x0000FF00) >> 8;
        Serial.print("data="); 
        Serial.print(data); 
        Serial.print(" x="); 
        Serial.print(sensorValue_x); 
        Serial.print(" y="); 
        Serial.print(sensorValue_y); 
        Serial.println(" [Process Data]");            
        sensorValue = sensorValue_y;
        
        if (sensorValue < YPOS_MID) {
          speedDir = DIR_FORWARD;
          speedValue = map(YPOS_MID-1-sensorValue,0,YPOS_MID-1,0,MAX_SPEED);
        } else {
          speedDir = DIR_BACKWARD; 
          speedValue = map( sensorValue-YPOS_MID,0,YPOS_MID-1,0,MAX_SPEED);
        }
        if (forward_brake_flag && speedDir == DIR_BACKWARD && speedValue > 30) {
          forward_brake_flag = false;
        }
        if (backward_brake_flag && speedDir == DIR_FORWARD && speedValue > 30) {
          backward_brake_flag = false;
        }
        if (distance_front_data > IGNORE_DISTANCE && distance_front_data < CLEAR_DISTANCE && speedDir == DIR_FORWARD) {  // Don't go any further.             
          forward_brake_flag = true;
        } 
        if (distance_back_data > IGNORE_DISTANCE && distance_back_data < CLEAR_DISTANCE && speedDir == DIR_BACKWARD) {  // Don't go any further.        
          backward_brake_flag = true;
        }
        if (forward_brake_flag | backward_brake_flag) {
          // Brake both wheels.
          speedValue = 0;          
          speedValueL = speedValue; 
          speedValueR = speedValue; 
        } else {
          // Checking for turns.
          sensorValue = sensorValue_x;
          if (sensorValue < XPOS_MID) {
            turnDir = DIR_LEFT;     
            speedsubValue = map(XPOS_MID-1-sensorValue,0,XPOS_MID-1,0,MAX_SPEED_SLOWDOWN);
          } else {
            turnDir = DIR_RIGHT;                    
            speedsubValue = map(sensorValue-XPOS_MID,0,XPOS_MID-1,0,MAX_SPEED_SLOWDOWN);
          }
          if (turnDir == DIR_RIGHT) {
            if (speedsubValue > speedValue) {
              speedValueR = 0;
            } else {
              speedValueR = speedValue - speedsubValue;  // slow down the right wheel for a left turn  
            }            
            speedValueL = speedValue;            

          } else {
            speedValueR = speedValue;
            if (speedsubValue > speedValue) {
              speedValueL = 0;
            } else {
              speedValueL = speedValue - speedsubValue; // slow down the left wheel for a right turn
            }            
          }        
        }
        print_all_variables();  // for debug purpose        
        drive_motor(speedDir,speedValueL,speedValueR);
        update_oled_display(sensorValue_x, sensorValue_y, speedDir, speedValue, distance_front_data,distance_back_data,forward_brake_flag,backward_brake_flag);
        mySwitch.resetAvailable();    
        process_flag = true;
    } else if (prevvalue == nowvalue && confirm_flag == false) {        
        repeat_count++;
        Serial.print(nowvalue); 
        Serial.print(" [Same Data] ");
        Serial.print(repeat_count); 
        Serial.println("X");
        if (repeat_count == repaat_count_min) {
          confirm_flag = true;
          process_flag = false;
          data = nowvalue;         
        }                       
    } else if (prevvalue != nowvalue) {
        Serial.print(nowvalue); 
        Serial.println(" [New Data]");          
        prevvalue = nowvalue;  
        confirm_flag = false;
        repeat_count = 0;
    } else {
        // No change //          
    }
}

void print_all_variables () {

        Serial.print("1) ");
        if (speedDir == DIR_FORWARD) {
            Serial.print("Direction=Foward"); 
        } else {
            Serial.print("Direction=Backward"); 
        }
        Serial.print(" Speed="); 
        Serial.print(speedValue); 
        Serial.println(" [Process Data]"); 
        //////////////////////////////////////////////////////////
        Serial.print("2) ");
        if (turnDir == DIR_LEFT) {
            Serial.print("Turn=Left");           
        } else {
            Serial.print("Turn=Right");           
        }  
        Serial.print(" Speed Subtract Value="); 
        Serial.print(speedsubValue); 
        Serial.println(" [Process Data]"); 
        //////////////////////////////////////////////////////////
        Serial.print("3) ");
        Serial.print("SpeedValueL="); 
        Serial.print(speedValueL);
        Serial.print(" SpeedValueR="); 
        Serial.print(speedValueR);
        Serial.println(" [Process Data]"); 
        //////////////////////////////////////////////////////////
        Serial.print("4) ");
        Serial.print("Distance_Front="); 
        Serial.print(distance_front_data);

        if (forward_brake_flag) {
          Serial.print(" Forward Brake ON");   
        } else {
          Serial.print(" Forward Brake OFF");   
        }
        Serial.println(" [Process Data]"); 
        //////////////////////////////////////////////////////////
        Serial.print("5) ");
        Serial.print("Distance_back="); 
        Serial.print(distance_back_data);

        if (backward_brake_flag) {
          Serial.print(" Backward Brake ON");   
        } else {
          Serial.print(" Backward Brake OFF");   
        }
        Serial.println(" [Process Data]"); 
}

void process_distance_sensor_front() {   
    duration = getduration_front();
    now_distance_front = finddistance(duration);  

    if (distfront_confirm_flag == true && distfront_process_flag == false) { 
        //Serial.print(distance_front_data); 
        //Serial.println(" [Process Data-distance_front]");                    
        distfront_process_flag = true;
    } else if ((abs(prev_distance_front - now_distance_front) < 10) && distfront_confirm_flag == false) {        
        distfront_repeat_count++;
        //Serial.print(now_distance_front); 
        //Serial.print(" [Same Data-distance_front] ");
        //Serial.print(distfront_repeat_count); 
        //Serial.println("X");
        if (distfront_repeat_count == distfront_repaat_count_min) {
          distfront_confirm_flag = true;
          distfront_process_flag = false;
          distance_front_data = now_distance_front;         
        }                       
    } else if (abs(prev_distance_front - now_distance_front) >= 10) {
        //Serial.print(now_distance_front); 
        //Serial.println(" [New Data-distance_front]");          
        prev_distance_front = now_distance_front;  
        distfront_confirm_flag = false;
        distfront_repeat_count = 0;
    } else {
        // No change //          
    }
}

void process_distance_sensor_back() {   
    duration = getduration_back();
    now_distance_back = finddistance(duration);  

    if (distback_confirm_flag == true && distback_process_flag == false) { 
        //Serial.print(distance_back_data); 
        //Serial.println(" [Process Data-distance_back]");                    
        distback_process_flag = true;
    } else if ((abs(prev_distance_back - now_distance_back) < 10) && distback_confirm_flag == false) {        
        distback_repeat_count++;
        //Serial.print(now_distance_back); 
        //Serial.print(" [Same Data-distance_back] ");
        //Serial.print(distback_repeat_count); 
        //Serial.println("X");
        if (distback_repeat_count == distback_repaat_count_min) {
          distback_confirm_flag = true;
          distback_process_flag = false;
          distance_back_data = now_distance_back;         
        }                       
    } else if (abs(prev_distance_back - now_distance_back) >= 10) {
        //Serial.print(now_distance_back); 
        //Serial.println(" [New Data-distance_back]");          
        prev_distance_back = now_distance_back;  
        distback_confirm_flag = false;
        distback_repeat_count = 0;
    } else {
        // No change //          
    }
}

//////////////////////////////////////////////////////
//                  Subroutines                     //
//////////////////////////////////////////////////////

unsigned long getduration_front (){
  unsigned long duration;
  //const unsigned long timeout = 1370;     // 15cm x 29.0 us/cm x 2 + 500 us (overhead)
  
  const unsigned long timeout = 60000;     // 60 msec (same as the cycle time)
  const unsigned int  trig_width = 10;    // 10 usec
//const unsigned long cycle_time = 60000; // cycle time for one measurment : 60 ms or 60000 us (delayMicroseconds)
  const unsigned long cycle_time = 60;    // cycle time for one measurment : 60 ms (delay)
  

  digitalWrite(trigOPin_front, LOW);  
  delayMicroseconds(2);            // pauses for 2 microseconds   

  digitalWrite(trigOPin_front, HIGH);  
  delayMicroseconds(trig_width);    // pauses for X microseconds   
  
  digitalWrite(trigOPin_front, LOW);   
  delayMicroseconds(2);            // pauses for 2 microseconds   
  
  duration = pulseIn(echoIPin_front, HIGH, timeout);
  //delayMicroseconds(cycle_time);     // pauses for 60 milliseconds for another measurement 
  delay(cycle_time);                   // change to delay for ESP8266's background program.
  return duration;
}

unsigned long getduration_back (){
  unsigned long duration;
  //const unsigned long timeout = 1370;     // 15cm x 29.0 us/cm x 2 + 500 us (overhead)
  
  const unsigned long timeout = 60000;     // 60 msec (same as the cycle time)
  const unsigned int  trig_width = 10;    // 10 usec
//const unsigned long cycle_time = 60000; // cycle time for one measurment : 60 ms or 60000 us (delayMicroseconds)
  const unsigned long cycle_time = 60;    // cycle time for one measurment : 60 ms (delay)
  

  digitalWrite(trigOPin_back, LOW);  
  delayMicroseconds(2);            // pauses for 2 microseconds   

  digitalWrite(trigOPin_back, HIGH);  
  delayMicroseconds(trig_width);    // pauses for X microseconds   
  
  digitalWrite(trigOPin_back, LOW);   
  delayMicroseconds(2);            // pauses for 2 microseconds   
  
  duration = pulseIn(echoIPin_back, HIGH, timeout);
  //delayMicroseconds(cycle_time);     // pauses for 60 milliseconds for another measurement 
  delay(cycle_time);                   // change to delay for ESP8266's background program.
  return duration;
}
float finddistance(unsigned long duration){
   return (duration/2.0/29.0);      // sound of speed = 331.5+0.6x23c = 345.3
                                     // distance (m) = duration (s) x 345.3 (m/s) / 2
                                     // distance (cm) = duration / 2 x (345.3 m/s x 100 cm/s x s/1e6 us)
                                     //            duration (us) / 2 / 29.0 (us/cm)
  
}


void drive_motor(int speedDir, int speedValueL, int speedValueR) {
  if (speedDir == DIR_FORWARD){
     digitalWrite(DCL_dir_pin,LOW);
     digitalWrite(DCR_dir_pin,LOW);
     analogWrite(DCL_speed_pin, speedValueL); //PWM Speed Control
     //analogWrite(DCL_speed_pin, 0); //PWM Speed Control
     analogWrite(DCR_speed_pin, speedValueR); //PWM Speed Control
  } else {
     digitalWrite(DCL_dir_pin,HIGH);
     digitalWrite(DCR_dir_pin,HIGH);
     analogWrite(DCL_speed_pin, 255-speedValueL); //PWM Speed Control
     //analogWrite(DCL_speed_pin, 255); //PWM Speed Control
     analogWrite(DCR_speed_pin, 255-speedValueR); //PWM Speed Control
  }
}

void update_oled_display (int pos_x, int pos_y, int dirvalue, int speedvalue, int dist_front, int dist_back, boolean frontbrake, boolean backbrake) {
 // print out the value you read:
  display.display();
  display.clearDisplay();   // clears the screen and buffer

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);  

  dataline = String("v4 PosX=" + String(pos_x) + " PosY=" + String(pos_y));  
  //Serial.println(dataline);  
  display.println(dataline);

  if (dirvalue == DIR_FORWARD) {
    dataline = String("Dir= FWD, Speed=" + String(speedvalue));  
  } else {
    dataline = String("Dir= BWD, Speed=" + String(speedvalue));  
  }
  
  
  //Serial.println(dataline);  
  display.println(dataline);
  if (frontbrake) {
    dataline = String("Dist_front=" + String(dist_front) + " Brake");  
  } else {
    dataline = String("Dist_front=" + String(dist_front));  
  }  
  //Serial.println(dataline);  
  display.println(dataline);

  if (backbrake) {
    dataline = String("Dist_back=" + String(dist_back) + " Brake");  
  } else {
    dataline = String("Dist_back=" + String(dist_back));
  }  
  //Serial.println(dataline);  
  display.println(dataline);
  display.display();
}
