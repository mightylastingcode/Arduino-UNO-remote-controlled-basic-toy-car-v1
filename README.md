# Arduino-UNO-remote-controlled-basic-toy-car-v1
Use Arduino UNO and Nano to build a toy car and a remote control module.



RecieveDemo_rc_car_xy_0.ino (example)

  Target board: Arduino UNO R3 (Toy Car)

  Example: This code is written for the processor which recieves
           the X and Y positions of the joystick from the remote
           control module.   The processor will translate the 
           positions to contorl the two DC motors' speed and direction.

SendDemo_rc_car_xy_0.ino (example)

  Target board: Arduino NANO (Remote Control Module)

  Example: This code is written for the processor which transmitts
           the X and Y positions of the joystick to the toy
           car.   
  
Libraries included in this repository:

Adafruit_GFX_Library : Support OLED display

Adafruit_SSD1306_Lite : Support OLED display (up to 4 lines of text only to conserve
                        RAM space for Arduino)

RCSwitch : Support wireless 433Mhz modules



See this blog for more details of the project.
  https://www.miketechuniverse.com/single-post/2017/05/14/My-Arduino-Test-Vehicle
