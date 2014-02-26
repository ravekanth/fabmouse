/* THE FABMOUSE
**  
** This code has to be uploaded to the Fabmouse.
** For more details, see http://hci.rwth-aachen.de/fabmouse
**
** This library uses the FreeIMU library provided at-
** http://www.varesano.net/projects/hardware/FreeIMU
** Created: 19/02/2014
** Modified: 19/02/2014
**
** Verena Kuhr, Marty Pye, Andrii Matviienko, Ravi Kanth, Claude Bemtgen
**
*/


#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <SPI.h> 

#include <Wire.h>

//define mappings for function keys
//These are used to encode the gyro/acclerometer data to be sent as keyboard reports
#define AXIS_X 0x3A   //F1 - 112
#define AXIS_Y 0x3B   //F2 - 113
#define AXIS_Z 0x3C   //F3 - 114
#define POS 0x3D      //F4 - 115
#define NEG 0x3E      //F5 - 116
#define VAL_0 0x3F    //F6 & 0 - 117
#define VAL_1 0x40    //F7 & 1 - 118
#define VAL_2 0x41    //F8 & 2 - 119
#define VAL_3 0x42    //F9 & 3 - 120
#define VAL_4 0x43    //F10 & 4 - 121
#define DELIM 0x44    //F11 & delimiter - 122

//Optical sensor stuff
#define CLOCK_PIN 6
#define DATA_PIN 4


/////////////////////////////////////////////////////////////////////////////////
// global variables
/////////////////////////////////////////////////////////////////////////////////

boolean three_d_mode_activated = false;

float angles[3]; // store Euler angles
float oldAngles[3] = {0,0,0}; // save Euler Angles
byte encodedArray[6];
int changeX, changeY, changeZ; // Store changes in Axes
byte nullByte = 0x0;

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

//Initialize the buffer
uint8_t buf[8] = { 0 }; 

//Optical sensor stuff
int nReset = 9;
int nCS = 7;

//button pins
int leftButtonPin = 8;
int rightButtonPin = 11;
boolean leftButtonWasPressed;
boolean rightButtonWasPressed;

/////////////////////////////////////////////////////////////////////////////////
// SETUP
/////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  //Optical sensor stuff
  pinMode (CLOCK_PIN, OUTPUT);
  pinMode (DATA_PIN, INPUT);
  pinMode (nReset, OUTPUT);
  pinMode (nCS, OUTPUT);
  
  pinMode(leftButtonPin, OUTPUT);
  digitalWrite(leftButtonPin, HIGH);
  pinMode(rightButtonPin, OUTPUT);
  digitalWrite(rightButtonPin, HIGH);  
//  pinMode(leftButtonPin, INPUT);     
//  pinMode(rightButtonPin, INPUT);     

  
  digitalWrite(nReset, LOW);
  digitalWrite(nCS, HIGH);
  
  Serial.begin(115200);
  delay(1000);
  digitalWrite(nReset, HIGH);
  delay(1000);
  Wire.begin();

  // GYRO/ACCEL stuff
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}



/////////////////////////////////////////////////////////////////////////////////
// LOOP
/////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  
  //check if we are in 3d-mode
  int leftButtonState = digitalRead(leftButtonPin);
  int rightButtonState = digitalRead(rightButtonPin);
  
  // button Released
  if (leftButtonWasPressed == true && leftButtonState == HIGH) {
    leftButtonWasPressed = false;
  }
  if (rightButtonWasPressed == true && rightButtonState == HIGH) {
    rightButtonWasPressed = false;
  }
  
  
  // WE activated 3D mode
  if (leftButtonState == LOW && rightButtonState == LOW && leftButtonWasPressed == false && rightButtonWasPressed == false) 
  {    
   
   //Init our axes 
    delay(5);
    sixDOF.init(); //begin the IMU
    delay(5);
  
    //we are in 3d-mode
    three_d_mode_activated = !three_d_mode_activated;

    leftButtonWasPressed = true;
    rightButtonWasPressed = true;
  } 
  
  if(!three_d_mode_activated)
  {
    if (leftButtonState == LOW && leftButtonWasPressed == false && rightButtonState == HIGH) 
    {
      pressLeft();
      leftButtonWasPressed = true;
      rightButtonWasPressed = false;
    }
    
    if (leftButtonState == HIGH && rightButtonState == LOW && rightButtonWasPressed == false) 
    {
      pressRight();
      rightButtonWasPressed = true;
      leftButtonWasPressed = false;
    }
    
//    if (read_register(0x02)){
      int rawX = read_register(0x03);
      if(rawX > 128)
          rawX -= 256;
        
      int rawY = read_register(0x04);
      if(rawY > 128)
        rawY -= 256;
      
//      Serial.println(rawX);
//      Serial.println(rawY);
      byte deltaX = (char) -rawX;
      byte deltaY = (char) rawY; 
    
      if (deltaX != 0 || deltaY !=0)
      movement(deltaX,deltaY);
    
      delay(10);
//    }
  }
  else
  {
    //We want the new values every 200ms
    delay(200);
  
    sixDOF.getEuler(angles);
  
    //Get the changes in Angles in 200 ms 
    changeX = -angles[2]-oldAngles[0];
    changeY = angles[1]-oldAngles[1];
    changeZ = angles[0]-oldAngles[2];
  
    //Check and update angle values if change in any direction is more than 5
    // Like this we are not sending data all the time, but only if we have major changes
    if (abs(changeX) > 5 || abs(changeY) > 5 || abs(changeZ) > 5) {
      updateAngles(-angles[2],angles[1],angles[0]);
    }
    
  }

}


/////////////////////////////////////////////////////////////////////////////////
// Helper functions
/////////////////////////////////////////////////////////////////////////////////


// Set the "oldAngle" and send the values of the new angle
// USE: 3D stuff
//////////////////////////////////////////////////////////////

void updateAngles(int x, int y, int z)
{

    encodeValue(abs(x));
    if(x < 0)
    {
      press(AXIS_X, NEG, encodedArray[4], encodedArray[5], nullByte);
      press(nullByte, nullByte, encodedArray[2], encodedArray[3], nullByte); 
      press(nullByte, nullByte, encodedArray[0], encodedArray[1], nullByte); 
    }
    else
    {
      press(AXIS_X, POS, encodedArray[4], encodedArray[5], nullByte);
      press(nullByte, nullByte, encodedArray[2], encodedArray[3], nullByte); 
      press(nullByte, nullByte, encodedArray[0], encodedArray[1], nullByte); 
    }

    encodeValue(abs(y));
    if(y < 0)
    {
      press(AXIS_Y, NEG, encodedArray[4], encodedArray[5], nullByte);
      press(nullByte, nullByte, encodedArray[2], encodedArray[3], nullByte); 
      press(nullByte, nullByte, encodedArray[0], encodedArray[1], nullByte);  
    }
    else
    {
      press(AXIS_Y, POS, encodedArray[4], encodedArray[5], nullByte);
      press(nullByte, nullByte, encodedArray[2], encodedArray[3], nullByte); 
      press(nullByte, nullByte, encodedArray[0], encodedArray[1], nullByte);  
    }

    encodeValue(abs(z));
    if(z < 0)
    {
      press(AXIS_Z, NEG, encodedArray[4], encodedArray[5], nullByte);
      press(nullByte, nullByte, encodedArray[2], encodedArray[3], nullByte); 
      press(nullByte, nullByte, encodedArray[0], encodedArray[1], DELIM);  
    }
    else
    {
      press(AXIS_Z, POS, encodedArray[4], encodedArray[5], nullByte);
      press(nullByte, nullByte, encodedArray[2], encodedArray[3], nullByte); 
      press(nullByte, nullByte, encodedArray[0], encodedArray[1], DELIM); 
    }
    
  
  //Update stored angle array
  oldAngles[0] = x;
  oldAngles[1] = y;
  oldAngles[2] = z;

}


// Encode values as function keys
// USE: 3D stuff
//////////////////////////////////////////////////////////////

void encodeValue(int value){
  int valueDigit;

  for (int i=0; i<3; i++){
    valueDigit = value%10;
    switch (valueDigit){
    case 0:
      encodedArray[2*i] = VAL_3;
      encodedArray[(2*i)+1] = VAL_4;
      break;

    case 1: 
      encodedArray[2*i] = VAL_0;
      encodedArray[(2*i)+1] = VAL_1;
      break;

    case 2:
      encodedArray[2*i] = VAL_0;
      encodedArray[(2*i)+1] = VAL_2;
      break;

    case 3:
      encodedArray[2*i] = VAL_0;
      encodedArray[(2*i)+1] = VAL_3;
      break;

    case 4:
      encodedArray[2*i] = VAL_0;
      encodedArray[(2*i)+1] = VAL_4;
      break;
      
    case 5: 
      encodedArray[2*i] = VAL_1;
      encodedArray[(2*i)+1] = VAL_2;
      break;

    case 6:
      encodedArray[2*i] = VAL_1;
      encodedArray[(2*i)+1] = VAL_3;
      break;

    case 7:
      encodedArray[2*i] = VAL_1;
      encodedArray[(2*i)+1] = VAL_4;
      break;

    case 8:
      encodedArray[2*i] = VAL_2;
      encodedArray[(2*i)+1] = VAL_3;
      break;

    case 9:
      encodedArray[2*i] = VAL_2;
      encodedArray[(2*i)+1] = VAL_4;
      break;
      
    default:
      encodedArray[2*i] = 0x04; //Check for Errors
      break;
    }
    value = value/10;
  }

}


// Read the register of the optical sensor
// USE: optical sensor
//////////////////////////////////////////////////////////////

byte read_register (byte address)
{
  digitalWrite(nCS, LOW);
  int i = 7;
  byte returnValue = 0;
  
  pinMode (DATA_PIN, OUTPUT);
  for (; i>=0; i--)
  {

     digitalWrite (CLOCK_PIN, LOW);
     digitalWrite (DATA_PIN, address & (1 << i));
     digitalWrite (CLOCK_PIN, HIGH);
  }
  
  pinMode (DATA_PIN, INPUT);
  delayMicroseconds(100);
 
  for (i=7; i>=0; i--)
  {
    digitalWrite (CLOCK_PIN, LOW);
    digitalWrite (CLOCK_PIN, HIGH);
    returnValue |= (digitalRead (DATA_PIN) << i);
  }
  delayMicroseconds(100);
  digitalWrite(nCS, HIGH);
  return returnValue;
}



// Send the raw HID reports
// USE: optical sensor & 3D stuff
//////////////////////////////////////////////////////////////

void press(byte axis, byte dir, byte val1, byte val2, byte delim)
{
  

  Serial.write(0xFD);
  Serial.write(0x9);
  Serial.write(0x1);
  Serial.write(0x06); //ALT & SHIFT
  Serial.write(nullByte);
  Serial.write(axis);
  Serial.write(dir);
  Serial.write(val1);
  Serial.write(val2);
  Serial.write(delim);
  Serial.write(nullByte);  
  
  Serial.write(0xFD);
  Serial.write(0x9);
  Serial.write(0x1);
  Serial.write(nullByte);
  Serial.write(nullByte);
  Serial.write(nullByte);
  Serial.write(nullByte);
  Serial.write(nullByte);
  Serial.write(nullByte);
  Serial.write(nullByte);
  Serial.write(nullByte);    
}


void movement(byte x, byte y)
{
    Serial.write(0xFD);
    Serial.write(0x5);
    Serial.write(0x2); 
    Serial.write(nullByte);
    Serial.write(x);
    Serial.write(y);
    Serial.write(nullByte);
}


void pressLeft()
{
    Serial.write(0xFD);
    Serial.write(0x5);
    Serial.write(0x2); 
    Serial.write(0x01); //Don't know if this is the right one
    Serial.write(nullByte);
    Serial.write(nullByte);
    Serial.write(nullByte);
}

void pressRight()
{
   Serial.write(0xFD);
    Serial.write(0x5);
    Serial.write(0x2); 
    Serial.write(0x02); //Don't know if this is the right one
    Serial.write(nullByte);
    Serial.write(nullByte);
    Serial.write(nullByte);
}

