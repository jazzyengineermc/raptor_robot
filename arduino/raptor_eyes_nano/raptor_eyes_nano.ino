/***************************************************************************
  Code is adapted from:

  Created by Rui Santos
  https://randomnerdtutorials.com/

  AND

  The APDS9960 library example 'gesture_sensor'
   
  This is a library for the APDS9960 digital proximity, ambient light, RGB, and gesture sensor

  This sketch puts the sensor in gesture mode and decodes gestures.
  To use this, first put your hand close to the sensor to enable gesture mode.
  Then move your hand about 6" from the sensor in the up -> down, down -> up, 
  left -> right, or right -> left direction.

  Designed specifically to work with the Adafruit APDS9960 breakout
  ----> http://www.adafruit.com/products/3595

  These sensors use I2C to communicate. The device's I2C address is 0x39

  Written by Dean Miller for Adafruit Industries.

  Edited and adapted by Matty Bee - while this works for me and I hope it works for you too - I don't take any responsibility for any mishaps
  Happy coding and happy learning!
  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "LedControl.h"
#include "binary.h"

int potVal;
int looking;

/*
 DIN connects to pin 11
 CLK connects to pin 12
 CS connects to pin 10 
 How many displays do you have?
*/
LedControl lc=LedControl(11,12,10,2);

void setup() {
  delay(2000);
  
  Serial.begin(115200);

    //we have already set the number of devices when we created the LedControl
  int devices=lc.getDeviceCount();
  //we have to init all devices in a loop
  for(int address=0;address<devices;address++) {
    /*The MAX72XX is in power-saving mode on startup*/
    lc.shutdown(address,false);
    /* Set the brightness to a medium values */
    lc.setIntensity(address,8);
    /* and clear the display */
    lc.clearDisplay(address);
}
  openeyes();
}

//  This is a list of all the different bitmaps/frames for each part of the eye animation

// neutral eye
byte eyeopen[8]= {
B00111100,
B01000010,
B10000001,
B10011001,
B10011001,
B10000001,
B01000010,
B00111100};

// eyes closed
byte eyeclosed[8]= {
B00011000,
B00011000,
B00011000,
B00011000,
B00011000,
B00011000,
B00011000,
B00011000};

// eyes partially closed
byte eyepartial[8]= {
B00011000,
B00100100,
B00100100,
B00111100,
B00111100,
B00100100,
B00100100,
B00011000};

// eyes partially open
byte eyepartialopen[8]= {
B00011000,
B00100100,
B01000010,
B01011010,
B01011010,
B01000010,
B00100100,
B00011000};

// eyes slightleft
byte slightleft[8]= {
B00111100,
B01000010,
B10011001,
B10011001,
B10000001,
B10000001,
B01000010,
B00111100};

// eyes left
byte left[8]= {
B00111100,
B01011010,
B10011001,
B10000001,
B10000001,
B10000001,
B01000010,
B00111100};

// eyes realleft
byte realleft[8]= {
B00111100,
B01011010,
B10000001,
B10000001,
B10000001,
B01000010,
B00111100,
B00000000};

// eyes slightright
byte slightright[8]= {
B00111100,
B01000010,
B10000001,
B10000001,
B10011001,
B10011001,
B01000010,
B00111100};

// eyes right
byte right[8]= {
B00111100,
B01000010,
B10000001,
B10000001,
B10000001,
B10011001,
B01011010,
B00111100};

// eyes realright
byte realright[8]= {
B00000000,
B01111110,
B10000001,
B10000001,
B10000001,
B10000001,
B01011010,
B00111100};


// the loop function runs over and over again forever
void loop() {
  
      closeeyes();
      delay(200);
      openeyes();
      delay(5000);
      closeeyes();
      delay(200);
      openeyes();
      delay(5000);
      lookright();
      delay(100);
      lookleft();
      delay(100);
  
  drawLooking();                              // Check potentiometer and draw the eyes to match the value from the pot
  
} // end of the Loop function

// The following functions relate to drawing/animating the eyes

void lookleft() {                    //  The function to get the eyes to look right (draws frame by frame)
  lc.setRow(0,0,slightright[0]);
  lc.setRow(0,1,slightright[1]);
  lc.setRow(0,2,slightright[2]);
  lc.setRow(0,3,slightright[3]);
  lc.setRow(0,4,slightright[4]);
  lc.setRow(0,5,slightright[5]);
  lc.setRow(0,6,slightright[6]);
  lc.setRow(0,7,slightright[7]);
  lc.setRow(1,0,slightright[0]);
  lc.setRow(1,1,slightright[1]);
  lc.setRow(1,2,slightright[2]);
  lc.setRow(1,3,slightright[3]);
  lc.setRow(1,4,slightright[4]);
  lc.setRow(1,5,slightright[5]);
  lc.setRow(1,6,slightright[6]);
  lc.setRow(1,7,slightright[7]);
  delay(150);
  lc.setRow(0,0,right[0]);
  lc.setRow(0,1,right[1]);
  lc.setRow(0,2,right[2]);
  lc.setRow(0,3,right[3]);
  lc.setRow(0,4,right[4]);
  lc.setRow(0,5,right[5]);
  lc.setRow(0,6,right[6]);
  lc.setRow(0,7,right[7]);
  lc.setRow(1,0,right[0]);
  lc.setRow(1,1,right[1]);
  lc.setRow(1,2,right[2]);
  lc.setRow(1,3,right[3]);
  lc.setRow(1,4,right[4]);
  lc.setRow(1,5,right[5]);
  lc.setRow(1,6,right[6]);
  lc.setRow(1,7,right[7]);
  delay(150);
  lc.setRow(0,0,realright[0]);
  lc.setRow(0,1,realright[1]);
  lc.setRow(0,2,realright[2]);
  lc.setRow(0,3,realright[3]);
  lc.setRow(0,4,realright[4]);
  lc.setRow(0,5,realright[5]);
  lc.setRow(0,6,realright[6]);
  lc.setRow(0,7,realright[7]);
  lc.setRow(1,0,realright[0]);
  lc.setRow(1,1,realright[1]);
  lc.setRow(1,2,realright[2]);
  lc.setRow(1,3,realright[3]);
  lc.setRow(1,4,realright[4]);
  lc.setRow(1,5,realright[5]);
  lc.setRow(1,6,realright[6]);
  lc.setRow(1,7,realright[7]);
  delay(600);
  lc.setRow(0,0,right[0]);
  lc.setRow(0,1,right[1]);
  lc.setRow(0,2,right[2]);
  lc.setRow(0,3,right[3]);
  lc.setRow(0,4,right[4]);
  lc.setRow(0,5,right[5]);
  lc.setRow(0,6,right[6]);
  lc.setRow(0,7,right[7]);
  lc.setRow(1,0,right[0]);
  lc.setRow(1,1,right[1]);
  lc.setRow(1,2,right[2]);
  lc.setRow(1,3,right[3]);
  lc.setRow(1,4,right[4]);
  lc.setRow(1,5,right[5]);
  lc.setRow(1,6,right[6]);
  lc.setRow(1,7,right[7]);
  delay(150);
  lc.setRow(0,0,slightright[0]);
  lc.setRow(0,1,slightright[1]);
  lc.setRow(0,2,slightright[2]);
  lc.setRow(0,3,slightright[3]);
  lc.setRow(0,4,slightright[4]);
  lc.setRow(0,5,slightright[5]);
  lc.setRow(0,6,slightright[6]);
  lc.setRow(0,7,slightright[7]);
  lc.setRow(1,0,slightright[0]);
  lc.setRow(1,1,slightright[1]);
  lc.setRow(1,2,slightright[2]);
  lc.setRow(1,3,slightright[3]);
  lc.setRow(1,4,slightright[4]);
  lc.setRow(1,5,slightright[5]);
  lc.setRow(1,6,slightright[6]);
  lc.setRow(1,7,slightright[7]);
  delay(150);
  lc.setRow(0,0,eyeopen[0]);
  lc.setRow(0,1,eyeopen[1]);
  lc.setRow(0,2,eyeopen[2]);
  lc.setRow(0,3,eyeopen[3]);
  lc.setRow(0,4,eyeopen[4]);
  lc.setRow(0,5,eyeopen[5]);
  lc.setRow(0,6,eyeopen[6]);
  lc.setRow(0,7,eyeopen[7]);
  lc.setRow(1,0,eyeopen[0]);
  lc.setRow(1,1,eyeopen[1]);
  lc.setRow(1,2,eyeopen[2]);
  lc.setRow(1,3,eyeopen[3]);
  lc.setRow(1,4,eyeopen[4]);
  lc.setRow(1,5,eyeopen[5]);
  lc.setRow(1,6,eyeopen[6]);
  lc.setRow(1,7,eyeopen[7]);
}

void lookright() {                    //  The function to get the eyes to look left (draws frame by frame)
  lc.setRow(0,0,slightleft[0]);
  lc.setRow(0,1,slightleft[1]);
  lc.setRow(0,2,slightleft[2]);
  lc.setRow(0,3,slightleft[3]);
  lc.setRow(0,4,slightleft[4]);
  lc.setRow(0,5,slightleft[5]);
  lc.setRow(0,6,slightleft[6]);
  lc.setRow(0,7,slightleft[7]);
  lc.setRow(1,0,slightleft[0]);
  lc.setRow(1,1,slightleft[1]);
  lc.setRow(1,2,slightleft[2]);
  lc.setRow(1,3,slightleft[3]);
  lc.setRow(1,4,slightleft[4]);
  lc.setRow(1,5,slightleft[5]);
  lc.setRow(1,6,slightleft[6]);
  lc.setRow(1,7,slightleft[7]);
  delay(150);
  lc.setRow(0,0,left[0]);
  lc.setRow(0,1,left[1]);
  lc.setRow(0,2,left[2]);
  lc.setRow(0,3,left[3]);
  lc.setRow(0,4,left[4]);
  lc.setRow(0,5,left[5]);
  lc.setRow(0,6,left[6]);
  lc.setRow(0,7,left[7]);
  lc.setRow(1,0,left[0]);
  lc.setRow(1,1,left[1]);
  lc.setRow(1,2,left[2]);
  lc.setRow(1,3,left[3]);
  lc.setRow(1,4,left[4]);
  lc.setRow(1,5,left[5]);
  lc.setRow(1,6,left[6]);
  lc.setRow(1,7,left[7]);
  delay(150);
  lc.setRow(0,0,realleft[0]);
  lc.setRow(0,1,realleft[1]);
  lc.setRow(0,2,realleft[2]);
  lc.setRow(0,3,realleft[3]);
  lc.setRow(0,4,realleft[4]);
  lc.setRow(0,5,realleft[5]);
  lc.setRow(0,6,realleft[6]);
  lc.setRow(0,7,realleft[7]);
  lc.setRow(1,0,realleft[0]);
  lc.setRow(1,1,realleft[1]);
  lc.setRow(1,2,realleft[2]);
  lc.setRow(1,3,realleft[3]);
  lc.setRow(1,4,realleft[4]);
  lc.setRow(1,5,realleft[5]);
  lc.setRow(1,6,realleft[6]);
  lc.setRow(1,7,realleft[7]);
  delay(600);
  lc.setRow(0,0,left[0]);
  lc.setRow(0,1,left[1]);
  lc.setRow(0,2,left[2]);
  lc.setRow(0,3,left[3]);
  lc.setRow(0,4,left[4]);
  lc.setRow(0,5,left[5]);
  lc.setRow(0,6,left[6]);
  lc.setRow(0,7,left[7]);
  lc.setRow(1,0,left[0]);
  lc.setRow(1,1,left[1]);
  lc.setRow(1,2,left[2]);
  lc.setRow(1,3,left[3]);
  lc.setRow(1,4,left[4]);
  lc.setRow(1,5,left[5]);
  lc.setRow(1,6,left[6]);
  lc.setRow(1,7,left[7]);
  delay(150);
  lc.setRow(0,0,slightleft[0]);
  lc.setRow(0,1,slightleft[1]);
  lc.setRow(0,2,slightleft[2]);
  lc.setRow(0,3,slightleft[3]);
  lc.setRow(0,4,slightleft[4]);
  lc.setRow(0,5,slightleft[5]);
  lc.setRow(0,6,slightleft[6]);
  lc.setRow(0,7,slightleft[7]);
  lc.setRow(1,0,slightleft[0]);
  lc.setRow(1,1,slightleft[1]);
  lc.setRow(1,2,slightleft[2]);
  lc.setRow(1,3,slightleft[3]);
  lc.setRow(1,4,slightleft[4]);
  lc.setRow(1,5,slightleft[5]);
  lc.setRow(1,6,slightleft[6]);
  lc.setRow(1,7,slightleft[7]);
  delay(150);
  lc.setRow(0,0,eyeopen[0]);
  lc.setRow(0,1,eyeopen[1]);
  lc.setRow(0,2,eyeopen[2]);
  lc.setRow(0,3,eyeopen[3]);
  lc.setRow(0,4,eyeopen[4]);
  lc.setRow(0,5,eyeopen[5]);
  lc.setRow(0,6,eyeopen[6]);
  lc.setRow(0,7,eyeopen[7]);
  lc.setRow(1,0,eyeopen[0]);
  lc.setRow(1,1,eyeopen[1]);
  lc.setRow(1,2,eyeopen[2]);
  lc.setRow(1,3,eyeopen[3]);
  lc.setRow(1,4,eyeopen[4]);
  lc.setRow(1,5,eyeopen[5]);
  lc.setRow(1,6,eyeopen[6]);
  lc.setRow(1,7,eyeopen[7]);
}

void closeeyes() {                    //  The function to get the eyes to close (draws frame by frame)
  lc.setRow(0,0,eyepartialopen[0]);
  lc.setRow(0,1,eyepartialopen[1]);
  lc.setRow(0,2,eyepartialopen[2]);
  lc.setRow(0,3,eyepartialopen[3]);
  lc.setRow(0,4,eyepartialopen[4]);
  lc.setRow(0,5,eyepartialopen[5]);
  lc.setRow(0,6,eyepartialopen[6]);
  lc.setRow(0,7,eyepartialopen[7]);
  lc.setRow(1,0,eyepartialopen[0]);
  lc.setRow(1,1,eyepartialopen[1]);
  lc.setRow(1,2,eyepartialopen[2]);
  lc.setRow(1,3,eyepartialopen[3]);
  lc.setRow(1,4,eyepartialopen[4]);
  lc.setRow(1,5,eyepartialopen[5]);
  lc.setRow(1,6,eyepartialopen[6]);
  lc.setRow(1,7,eyepartialopen[7]);
  delay(100);
    
    // eyes half close
  lc.setRow(0,0,eyepartial[0]);
  lc.setRow(0,1,eyepartial[1]);
  lc.setRow(0,2,eyepartial[2]);
  lc.setRow(0,3,eyepartial[3]);
  lc.setRow(0,4,eyepartial[4]);
  lc.setRow(0,5,eyepartial[5]);
  lc.setRow(0,6,eyepartial[6]);
  lc.setRow(0,7,eyepartial[7]);
  lc.setRow(1,0,eyepartial[0]);
  lc.setRow(1,1,eyepartial[1]);
  lc.setRow(1,2,eyepartial[2]);
  lc.setRow(1,3,eyepartial[3]);
  lc.setRow(1,4,eyepartial[4]);
  lc.setRow(1,5,eyepartial[5]);
  lc.setRow(1,6,eyepartial[6]);
  lc.setRow(1,7,eyepartial[7]);
  delay(100);

      // eyes close
  lc.setRow(0,0,eyeclosed[0]);
  lc.setRow(0,1,eyeclosed[1]);
  lc.setRow(0,2,eyeclosed[2]);
  lc.setRow(0,3,eyeclosed[3]);
  lc.setRow(0,4,eyeclosed[4]);
  lc.setRow(0,5,eyeclosed[5]);
  lc.setRow(0,6,eyeclosed[6]);
  lc.setRow(0,7,eyeclosed[7]);
  lc.setRow(1,0,eyeclosed[0]);
  lc.setRow(1,1,eyeclosed[1]);
  lc.setRow(1,2,eyeclosed[2]);
  lc.setRow(1,3,eyeclosed[3]);
  lc.setRow(1,4,eyeclosed[4]);
  lc.setRow(1,5,eyeclosed[5]);
  lc.setRow(1,6,eyeclosed[6]);
  lc.setRow(1,7,eyeclosed[7]);
}

void openeyes() {                    //  The function to get the eyes to open (draws frame by frame)
 
    // eyes half close
  lc.setRow(0,0,eyepartial[0]);
  lc.setRow(0,1,eyepartial[1]);
  lc.setRow(0,2,eyepartial[2]);
  lc.setRow(0,3,eyepartial[3]);
  lc.setRow(0,4,eyepartial[4]);
  lc.setRow(0,5,eyepartial[5]);
  lc.setRow(0,6,eyepartial[6]);
  lc.setRow(0,7,eyepartial[7]);
  lc.setRow(1,0,eyepartial[0]);
  lc.setRow(1,1,eyepartial[1]);
  lc.setRow(1,2,eyepartial[2]);
  lc.setRow(1,3,eyepartial[3]);
  lc.setRow(1,4,eyepartial[4]);
  lc.setRow(1,5,eyepartial[5]);
  lc.setRow(1,6,eyepartial[6]);
  lc.setRow(1,7,eyepartial[7]);
  delay(100);

  lc.setRow(0,0,eyepartialopen[0]);
  lc.setRow(0,1,eyepartialopen[1]);
  lc.setRow(0,2,eyepartialopen[2]);
  lc.setRow(0,3,eyepartialopen[3]);
  lc.setRow(0,4,eyepartialopen[4]);
  lc.setRow(0,5,eyepartialopen[5]);
  lc.setRow(0,6,eyepartialopen[6]);
  lc.setRow(0,7,eyepartialopen[7]);
  lc.setRow(1,0,eyepartialopen[0]);
  lc.setRow(1,1,eyepartialopen[1]);
  lc.setRow(1,2,eyepartialopen[2]);
  lc.setRow(1,3,eyepartialopen[3]);
  lc.setRow(1,4,eyepartialopen[4]);
  lc.setRow(1,5,eyepartialopen[5]);
  lc.setRow(1,6,eyepartialopen[6]);
  lc.setRow(1,7,eyepartialopen[7]);
  delay(100);

  lc.setRow(0,0,eyeopen[0]);
  lc.setRow(0,1,eyeopen[1]);
  lc.setRow(0,2,eyeopen[2]);
  lc.setRow(0,3,eyeopen[3]);
  lc.setRow(0,4,eyeopen[4]);
  lc.setRow(0,5,eyeopen[5]);
  lc.setRow(0,6,eyeopen[6]);
  lc.setRow(0,7,eyeopen[7]);
  lc.setRow(1,0,eyeopen[0]);
  lc.setRow(1,1,eyeopen[1]);
  lc.setRow(1,2,eyeopen[2]);
  lc.setRow(1,3,eyeopen[3]);
  lc.setRow(1,4,eyeopen[4]);
  lc.setRow(1,5,eyeopen[5]);
  lc.setRow(1,6,eyeopen[6]);
  lc.setRow(1,7,eyeopen[7]);

}

void drawLooking() {                    
//  The function to get the eyes to follow the potentiometer - maps the values received from the pot to 7 different values - 
//  each of those values represents one of the frames and draws it.  0 is far left, 3 is neutral and 7 is far right.


  potVal = 512; //analogRead(A0);               // read the potentiometer value
  // Nevermind that, setup ros subscriber and look left/right based on other events
  looking = map(potVal,0,1023,0,7);  // map it to a number 0-7 and store it to looking variable
  
//  Serial.print("Pot Value is: ");  // You can use these to debug or test out your pot if it's not working properly to see what value is actually throwing
//  Serial.println(looking);         // Just uncomment these two lines to print to the Serial

//  Check the value of looking and print the relevant eyes accordingly
  
  if (looking == 0) {
  lc.setRow(0,0,realleft[0]);
  lc.setRow(0,1,realleft[1]);
  lc.setRow(0,2,realleft[2]);
  lc.setRow(0,3,realleft[3]);
  lc.setRow(0,4,realleft[4]);
  lc.setRow(0,5,realleft[5]);
  lc.setRow(0,6,realleft[6]);
  lc.setRow(0,7,realleft[7]);
  lc.setRow(1,0,realleft[0]);
  lc.setRow(1,1,realleft[1]);
  lc.setRow(1,2,realleft[2]);
  lc.setRow(1,3,realleft[3]);
  lc.setRow(1,4,realleft[4]);
  lc.setRow(1,5,realleft[5]);
  lc.setRow(1,6,realleft[6]);
  lc.setRow(1,7,realleft[7]);
  }
else if (looking == 1) {
  lc.setRow(0,0,left[0]);
  lc.setRow(0,1,left[1]);
  lc.setRow(0,2,left[2]);
  lc.setRow(0,3,left[3]);
  lc.setRow(0,4,left[4]);
  lc.setRow(0,5,left[5]);
  lc.setRow(0,6,left[6]);
  lc.setRow(0,7,left[7]);
  lc.setRow(1,0,left[0]);
  lc.setRow(1,1,left[1]);
  lc.setRow(1,2,left[2]);
  lc.setRow(1,3,left[3]);
  lc.setRow(1,4,left[4]);
  lc.setRow(1,5,left[5]);
  lc.setRow(1,6,left[6]);
  lc.setRow(1,7,left[7]);
  }

else if (looking == 2) {
  lc.setRow(0,0,slightleft[0]);
  lc.setRow(0,1,slightleft[1]);
  lc.setRow(0,2,slightleft[2]);
  lc.setRow(0,3,slightleft[3]);
  lc.setRow(0,4,slightleft[4]);
  lc.setRow(0,5,slightleft[5]);
  lc.setRow(0,6,slightleft[6]);
  lc.setRow(0,7,slightleft[7]);
  lc.setRow(1,0,slightleft[0]);
  lc.setRow(1,1,slightleft[1]);
  lc.setRow(1,2,slightleft[2]);
  lc.setRow(1,3,slightleft[3]);
  lc.setRow(1,4,slightleft[4]);
  lc.setRow(1,5,slightleft[5]);
  lc.setRow(1,6,slightleft[6]);
  lc.setRow(1,7,slightleft[7]);
  }

else if (looking == 3) {
  lc.setRow(0,0,eyeopen[0]);
  lc.setRow(0,1,eyeopen[1]);
  lc.setRow(0,2,eyeopen[2]);
  lc.setRow(0,3,eyeopen[3]);
  lc.setRow(0,4,eyeopen[4]);
  lc.setRow(0,5,eyeopen[5]);
  lc.setRow(0,6,eyeopen[6]);
  lc.setRow(0,7,eyeopen[7]);
  lc.setRow(1,0,eyeopen[0]);
  lc.setRow(1,1,eyeopen[1]);
  lc.setRow(1,2,eyeopen[2]);
  lc.setRow(1,3,eyeopen[3]);
  lc.setRow(1,4,eyeopen[4]);
  lc.setRow(1,5,eyeopen[5]);
  lc.setRow(1,6,eyeopen[6]);
  lc.setRow(1,7,eyeopen[7]);
}

else if (looking == 4) {
  lc.setRow(0,0,slightright[0]);
  lc.setRow(0,1,slightright[1]);
  lc.setRow(0,2,slightright[2]);
  lc.setRow(0,3,slightright[3]);
  lc.setRow(0,4,slightright[4]);
  lc.setRow(0,5,slightright[5]);
  lc.setRow(0,6,slightright[6]);
  lc.setRow(0,7,slightright[7]);
  lc.setRow(1,0,slightright[0]);
  lc.setRow(1,1,slightright[1]);
  lc.setRow(1,2,slightright[2]);
  lc.setRow(1,3,slightright[3]);
  lc.setRow(1,4,slightright[4]);
  lc.setRow(1,5,slightright[5]);
  lc.setRow(1,6,slightright[6]);
  lc.setRow(1,7,slightright[7]);
  }

else if (looking == 5) {
  lc.setRow(0,0,right[0]);
  lc.setRow(0,1,right[1]);
  lc.setRow(0,2,right[2]);
  lc.setRow(0,3,right[3]);
  lc.setRow(0,4,right[4]);
  lc.setRow(0,5,right[5]);
  lc.setRow(0,6,right[6]);
  lc.setRow(0,7,right[7]);
  lc.setRow(1,0,right[0]);
  lc.setRow(1,1,right[1]);
  lc.setRow(1,2,right[2]);
  lc.setRow(1,3,right[3]);
  lc.setRow(1,4,right[4]);
  lc.setRow(1,5,right[5]);
  lc.setRow(1,6,right[6]);
  lc.setRow(1,7,right[7]);
  }

else if (looking == 6) {
  lc.setRow(0,0,realright[0]);
  lc.setRow(0,1,realright[1]);
  lc.setRow(0,2,realright[2]);
  lc.setRow(0,3,realright[3]);
  lc.setRow(0,4,realright[4]);
  lc.setRow(0,5,realright[5]);
  lc.setRow(0,6,realright[6]);
  lc.setRow(0,7,realright[7]);
  lc.setRow(1,0,realright[0]);
  lc.setRow(1,1,realright[1]);
  lc.setRow(1,2,realright[2]);
  lc.setRow(1,3,realright[3]);
  lc.setRow(1,4,realright[4]);
  lc.setRow(1,5,realright[5]);
  lc.setRow(1,6,realright[6]);
  lc.setRow(1,7,realright[7]);
  }

}