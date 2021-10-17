# DCC_turnout_control

These files show how to build a device that will move up to eight servos.

It will connect to a JMRI server using C/MRI and the servos can be controlled through JMRI.

So far there is no feedback or other features and the turnout open and closed positions will need to be set manually in the Arduino code.

Here is the text from the "turnout_servo_sketchbook.ino" file as of the date of this file:

  This program is based on example code for the   Adafruit 16-channel PWM & servo driver written by Limor Fried/Ladyada for Adafruit Industries.

  Other ideas and code from:
  
  http://www.motorhomesites.org.uk/railway/JMRI_Arduino_Setup.php
  
  https://www.jmri.org/help/en/html/hardware/arduino/index.shtml
  
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/BlinkWithoutDelay
  
  Libraries used came from:
  
  https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
  
  https://github.com/madleech/Auto485
  
  https://github.com/madleech/ArduinoCMRI
  
  JMRI is here:
  
  https://www.jmri.org/
  
  Information about C/MRI is here:
  
  https://jlcenterprises.net/

  The program will control up to eight servos using the writeMicroseconds command for PWM control.

  Slow motion is obtained by incrementing the PWM command.

  Equipment needed:
   * Microcontroller - for this version I used an Arduino Uno Rev3
   * Adafruit PCA9685 PMW servo driver
   * Generic RC airplane type servos
   * Generic toggle switches
   * DCC System (I use an NCE PowerCab)
   * JMRI server (I use JMRI on a Raspberry Pi)
   * USB communications between JMRI server and Microcontroller. 
