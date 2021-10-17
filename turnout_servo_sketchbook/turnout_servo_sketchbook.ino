 
/*
 * This program is based on example code for the
 * Adafruit 16-channel PWM & servo driver
 * written by Limor Fried/Ladyada for Adafruit Industries.
 * 
 * Other ideas and code from:
 *  http://www.motorhomesites.org.uk/railway/JMRI_Arduino_Setup.php
 *  https://www.jmri.org/help/en/html/hardware/arduino/index.shtml
 *  
 *  The program will control up to eight servos using the
 *  writeMicroseconds command for PWM control.
 *  
 *  Slow motion is obtained by incrementing the PWM command.
 *  
 *  Equipment needed:
 *    Microcontroller - for this version I used an Arduino Uno Rev3
 *    Adafruit PCA9685 PMW servo driver
 *    Generic RC airplane type servos
 *    Generic toggle switches
 *    DCC System (I use an NCE PowerCab)
 *    JMRI server (I use JMRI on a Raspberry Pi)
 *    USB communications between JMRI server and Microcontroller.
 */

#include <CMRI.h> //  Arduino C/MRI library https://github.com/madleech/ArduinoCMRI
#include <Auto485.h> // Automatic RS485 Wrapper library https://github.com/madleech/Auto485
#include <Adafruit_PWMServoDriver.h> // Adafruit PWM servo driver library
      // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library

//  Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41); //testing adafruit board 1
//  Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x42);

#define CMRI_ADD1 1 //  sets CMRI address for the first device
#define DE_PIN 2 //  somthing to do with RS-485

//  setup a CMRI device
CMRI cmri(CMRI_ADD1,24,48); // defaults to an SMINI 0,24,48

/*  servo travel limits - these will need to be set up for the individual servos
 *  USMIN is minimum travel, USST is starting poing, and USMAX is maximum travel */

const int USLIM[3][8] = {
  {1200,1200,1200,1000,1000,1000,1000,1000},
  {1200,1200,1200,1000,1000,1000,1000,1000},
  {1600,1600,1600,1800,1800,1800,1800,1800},
};

Auto485 bus(DE_PIN); // arduino pin 2 for DE and RE pins

const int SERVO_FREQ = 60;  // Try 50 Hz if 60 does not work

//  time delays
unsigned long ztime = 0;
const long tdel1 = 5; // time delay 1 (ms)

/*  
 *  Turnout inputs:
 *  These are the digital addresses (CMRI-1) for the turnouts.
 *  Since there are eight turnouts we will use 40-47 to
 *  represent eight CMRI addresses from 41 to 48.
 */
const int turnOut[] = {40,41,42,43,44,45,46,47};

/*  servo position variable in microseconds (approximate but seems to be consistent)
 *  servos are numbered from 0 to 15. */
 
int psec[8];
int swstate[8];

void moveTurnout(byte x,int y) {

    if (y==1) {
      if (psec[x] < USLIM[2][x]) {
        psec[x] = psec[x] + 5;
        pwm1.writeMicroseconds(x, psec[x]);
      }
    }
  
    else {
      if (psec[x] > USLIM[0][x]) {
        psec[x] = psec[x] - 5;
        pwm1.writeMicroseconds(x, psec[x]);
      }
    }
}

void setup() {
 
/*  these 3 lines set up the PWM driver. They must be run for every board.
 *  the following commands set up the PWM boards' operating frequencies.
 *  this is necessary for consistent servo positions. */
 
  pwm1.begin(); //the 0x40 board is "pwm0"
    //t
    //this is necessary for consistent servo positions.
    pwm1.setOscillatorFrequency(27000000);
    pwm1.setPWMFreq(SERVO_FREQ);

  //  this starts the RS485 bus defined above
  bus.begin(9600);

  //input the initial position variables in usec
  
    for (byte i = 0; i <8; i++) {
      psec[i] = USLIM[1][i];
    }
 
    for (byte i = 0; i <8; i++) {
      moveTurnout(i,swstate[i]);
    }
  }

void loop() {

//  main processing node of cmri library
  cmri.process();

 // we are not in any hurry
 unsigned long otime = millis();

  if (otime - ztime >= tdel1) {

    // save the last time you blinked the LED
    
    ztime = otime;


/*  this for statement reads CMRI and sets the swstate[i] according to
 *  the pin state. This section can be elaborated to add interlocks or
 *  other automation. */
  
    for (byte i = 0; i <8; i++) {
      swstate[i] = cmri.get_bit(turnOut[i]);
    }

/*
 *  this statement calls the movement function
 *  for every turnout based on its swstate[]
 */
  
    for (byte i = 0; i <8; i++) {
        moveTurnout(i,swstate[i]);
    }
  }
}
