//Firmware for Tacsim throttle
//Alan H.
//17 April 2021

//Libraries:
//bbSPI for using shift registers by Nick Gammon http://www.gammon.com.au/Arduino/bitBangedSPI.zip
//MHeironimus Joystick library https://github.com/MHeironimus/ArduinoJoystickLibrary (Plus a modification to allow 8 axes in Win10 https://github.com/MHeironimus/ArduinoJoystickLibrary/issues/177)
//MCP3208 Library by labfruits https://github.com/labfruits/mcp320x
#include <Joystick.h>
#include <bitBangedSPI.h>
#include <Mcp320x.h>
#include <SPI.h>

//CHANGE SETTINGS HERE

//FORMAT: string divided by commas, first section classification, second and third section are buttons it binds to (except for encoders)
//Classification: 
//0 unassigned (put whatever for the second and third sections, it doesn't matter)
//1 button (binding section is 2nd index normal bind, 3rd section secondary bind, which is pressed at the same time. -1 if unused)
//2 toggle (binding section is 2nd index normal bind, 3rd section secondary bind, which is pressed at the same time. -1 if unused)
//3 on and off toggle (binding section should be 2nd index ON bind, and 3rd index OFF bind. -1 if either is unused, e.g. {3,-1,25} means a toggle that pulses button 25 when toggled OFF, and nothing when toggled ON)
//4 encoder (binding section should be 2nd index normal bind, and 3rd index HARDWARE COUNTERPART (CW position if the assigned button is CCW, and vice versa), 1 indexed for consistency (i.e. first hardware button = 1, second is 2, etc.). 
int digitalpins[72][3] = { //Array of length the number of buttons on the shift registers, values are the digital pins they map to. SOFTWARE BUTTONS ARE ZERO INDEXED
  {3,21,22}, //1
  {3,23,24},
  {3,27,29},
  {3,28,30}, //4
  {0,0,0},
  {1,5,-1},
  {1,3,-1},
  {1,4,-1}, //8
  {0,0,0},
  {1,41,-1},
  {1,40,-1},
  {1,25,-1}, //12
  {3,32,33}, 
  {3,34,35},
  {3,36,38},
  {3,37,39}, //16
  {0,0,0}, 
  {1,44,-1},
  {1,47,-1},
  {0,0,0}, //20
  {4,45,22},
  {4,46,21},
  {4,42,24},
  {4,43,23}, //24
  {3,49,-1},
  {3,62,63},
  {3,64,65},
  {3,60,61}, //28
  {3,52,53},
  {3,51,-1},
  {3,54,55},
  {3,56,57}, //32
  {3,50,-1},
  {1,66,-1},
  {3,48,-1},
  {3,58,59}, //36
  {0,0,0},
  {1,68,-1},
  {1,69,-1},
  {1,67,-1}, //40
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0}, //44
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0}, //48
  {1,19,-1}, //Grip binds
  {1,18,-1},
  {1,20,-1},
  {1,17,-1}, //52
  {1,13,-1},
  {1,14,-1},
  {1,15,-1},
  {1,16,-1}, //56
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0}, //60
  {0,0,0},
  {1,6,-1},
  {1,8,-1},
  {1,7,31}, //64; bind 8 and 32 to this section, for discord VC (discord doesnt tell the difference between devices, so having two buttons (one of which is much higher than what most other devices have) combined makes sure I don't accidentally activate PTT)
  {0,0,0},
  {1,2,-1},
  {1,1,-1},
  {1,0,-1}, //68
  {1,9,-1},
  {1,10,-1},
  {1,11,-1},
  {1,12,-1} //72
}; 


//Encoders. Format: hardware pin 1, hardware pin 2, software pin 1, software pin 2. Hardware pins are 1-indexed, software pins are 0-indexed
int encoders[6][4] = { //Six encoders, because why not (I only used two)
  {21, 22, 45, 46},
  {23, 24, 42, 43},
  {0,0,0,0},
  {0,0,0,0},
  {0,0,0,0},
  {0,0,0,0},
};

unsigned long encoderTimers[6]; //Times for encoder buttons to deactivate, in milliseconds


#define TOGGLE_DELAY 60 //Delay in ms
#define ENCODER_DELAY 60 //Delay in ms

#define NUMSR 9 //Number of daisy chained shift registers

//Define Shift Register pins
bitBangedSPI bbSPI (bitBangedSPI::NO_PIN, A2, 15);  // MOSI, MISO, SCK (No connect, data, clock)

const int LATCH = A3; //AKA Load or "CS"

//Axis data struct
struct axis{
  String label;
  int range[3];
  int centerdeadzone;
};
//Define Axis spec
axis ADCChannels[8] = { //Label, Range {min, center (-1 for no center), max} or {max, center (-1 for no center), min} if inverted, Center deadzone (both sides)
  {"z", {2486, -1, 1504}, 0}, //right throttle
  {"rz", {1582, -1, 2571}, 0}, //left throttle
  {"ry", {3079, 2135, 1231}, 150}, //TDC slew vertical
  {"rx", {3132, 2207, 1190}, 150}, //TDC slew horizontal
  {"rudder", {0, -1, 4095}, 0}, //knob 3
  {"throttle", {0, -1, 4095}, 0}, //knob 4
  {"x", {0, -1, 4095}, 0}, //knob 1
  {"y", {0, -1, 4095}, 0} //knob 2
};


#define FILTER 6 //Filter amount for all axes.
#define MAINRELDZ  50  //Main axis relative deadzone (on each side)(keeps the two axes the same value if they are within this amount of each other)


//Define ADC
#define SPI_CS      8       //SPI slave select
#define ADC_VREF    5000    //5V Vref
#define ADC_CLK     1600000 //SPI clock 1.6Mhz

MCP3208 adc(ADC_VREF, SPI_CS);

//JOYSTICK SETTINGS
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_JOYSTICK,
  70, //number of buttons
  0, //number of hat switches
  //Set as many axis to "true" as you have potentiometers for
  true, // y axis
  true, // x axis
  true, // z axis
  true, // rx axis
  true, // ry axis
  true, // rz axis
  true, // rudder
  true, // throttle
  false, // accelerator
  false, // brake
  false); // steering wheel
  
//END CHANGE SETTINGS

SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);

void setup() {
  // put your setup code here, to run once:
  // initialize serial
  Serial.begin(115200);
  
  Serial.println ("Initializing");
  
  //Shift registers:
  bbSPI.begin ();
  pinMode (LATCH, OUTPUT);
  digitalWrite (LATCH, HIGH);

  //ADC:
  // configure PIN mode
  pinMode(SPI_CS, OUTPUT);

  // set initial PIN state
  digitalWrite(SPI_CS, HIGH);
  
  // Initialize Joystick
  Joystick.begin();

  //Set axis ranges
  Joystick.setYAxisRange(ADCChannels[7].range[0], ADCChannels[7].range[2]);
  Joystick.setXAxisRange(ADCChannels[6].range[0], ADCChannels[6].range[2]);
  Joystick.setZAxisRange(ADCChannels[0].range[0], ADCChannels[0].range[2]);
  Joystick.setRxAxisRange(ADCChannels[3].range[0], ADCChannels[3].range[2]);
  Joystick.setRyAxisRange(ADCChannels[2].range[0], ADCChannels[2].range[2]);
  Joystick.setRzAxisRange(ADCChannels[1].range[0], ADCChannels[1].range[2]);
  Joystick.setRudderRange(ADCChannels[4].range[0], ADCChannels[4].range[2]);
  Joystick.setThrottleRange(ADCChannels[5].range[0], ADCChannels[5].range[2]);
  //End of setup
}

//Arrays with shift register states
byte SRStates[NUMSR];
byte oldSRStates[NUMSR]; // previous state
bool SRState1D[8*NUMSR + 1]; //state but each button has its own index in the array
bool oldSRState1D[8*NUMSR + 1]; //old state but each button has its own index in the array

//Arrays with previous axis readings to average for filtering.
uint16_t previousAxisVals[8][FILTER];
int nextAvgIndex[8];

void updateButtonValue(int hwbutton, byte buttonState, byte oldState){
  byte type = digitalpins[hwbutton-1][0];
  switch (type){
    case 0: //Unused button
      break;
    case 1: //Normal button
      if (buttonState == 0){ //LOW state means button is pressed, with pullups
        Joystick.pressButton(digitalpins[hwbutton-1][1]); //Press button
        if (digitalpins[hwbutton-1][2] != -1){ //Check if secondary button is assigned
          Joystick.pressButton(digitalpins[hwbutton-1][2]);
        }
      }else{ //HIGH state means button is released, with pullups
        Joystick.releaseButton(digitalpins[hwbutton-1][1]); //Release button
        if (digitalpins[hwbutton-1][2] != -1){ //Check if secondary is assigned
          Joystick.releaseButton(digitalpins[hwbutton-1][2]); 
        }
      }
      break;
    case 2: //Toggle
      //if ((buttonState == 0 && oldState == 1) || (buttonState == 1 && oldState == 0)){ //Button state has changed
      Joystick.pressButton(digitalpins[hwbutton-1][1]); //Press button
      if (digitalpins[hwbutton-1][2] != -1){ //Check if secondary button is assigned
        Joystick.pressButton(digitalpins[hwbutton-1][2]);
      }
      delay(TOGGLE_DELAY); //Keep pressed for given amount of time
      Joystick.releaseButton(digitalpins[hwbutton-1][1]); //Release button
      if (digitalpins[hwbutton-1][2] != -1){ //Check if secondary button is assigned
        Joystick.releaseButton(digitalpins[hwbutton-1][2]);
      }
      //}
      break;
    case 3: //Toggle ON and OFF
      //if (buttonState == 0 && oldState == 1){  //Check if toggled on and button is assigned
      if (buttonState == 0){
        if (digitalpins[hwbutton-1][1] != -1){
          Joystick.pressButton(digitalpins[hwbutton-1][1]); //Press button, wait, release
          delay(TOGGLE_DELAY); 
          Joystick.releaseButton(digitalpins[hwbutton-1][1]); 
        }
      //}else if (buttonState == 1 && oldState == 0){  //Check if toggled off and button is assigned
      }else{
        if (digitalpins[hwbutton-1][2] != -1){
          Joystick.pressButton(digitalpins[hwbutton-1][2]); //Press button, wait, release
          delay(TOGGLE_DELAY);
          Joystick.releaseButton(digitalpins[hwbutton-1][2]);
        }
      }
      break;
    /*case 4: //Encoder
      int counterhw = digitalpins[hwbutton-1][2]; //Get opposite direction pin state
      bool counterstate = SRState1D[counterhw];
      Serial.println(counterstate);
      if (!buttonState && counterstate){ //If this pin is closed and the other pin is open, the encoder is rotating in this direction
        Joystick.pressButton(digitalpins[hwbutton-1][1]); //Press, delay, release, ignoring possible turns within the delay.
        delay(ENCODER_DELAY);
        Joystick.releaseButton(digitalpins[hwbutton-1][1]);
      }
      break;*/ //Ignore this, this does not work
  }
  
}

void checkEncoders(){
  for (int i = 0; i < 6; i ++){ //check each encoder
    if (encoders[i][0] != 0){
      int hw1 = encoders[i][0];
      int hw2 = encoders[i][1];
      int sw1 = encoders[i][2];
      int sw2 = encoders[i][3];
      if (encoderTimers[i] != 0 && millis() >= encoderTimers[i]){
        Joystick.releaseButton(sw1);
        Joystick.releaseButton(sw2);
        encoderTimers[i] = 0;
      }
      if (encoderTimers[i] == 0 && SRState1D[hw1] == 0 && SRState1D[hw1] != oldSRState1D[hw1]){
        if (SRState1D[hw1] != SRState1D[hw2]){ //one direction
          Joystick.pressButton(sw1); //Press, delay, release, ignoring possible turns within the delay.
          encoderTimers[i] = millis() + ENCODER_DELAY;
          //delay(ENCODER_DELAY);
          //Joystick.releaseButton(sw1);
        }
      }else if (encoderTimers[i] == 0 && SRState1D[hw2] == 0 && SRState1D[hw2] != oldSRState1D[hw2]){
        if (SRState1D[hw2] != SRState1D[hw1]){//the other direction
          Joystick.pressButton(sw2); //Press, delay, release, ignoring possible turns within the delay.
          encoderTimers[i] = millis() + ENCODER_DELAY;
          //delay(ENCODER_DELAY);
          //Joystick.releaseButton(sw2);
        }
      }
    }
  }
}

uint16_t filteravg(int index, uint16_t raw){ //Running average filter
  previousAxisVals[index][nextAvgIndex[index]++] = raw; 
  if (nextAvgIndex[index] >= FILTER){ //Wrap index to add value around to the oldest value when the end is reached
    nextAvgIndex[index] = 0;
  }
  float runningAvg = 0;
  for (int i = 0; i < FILTER; i++){ //Average current contents of axis value buffer
    runningAvg += previousAxisVals[index][i];
  }
  runningAvg /= FILTER;
  return runningAvg;
}

void processAxis(int index, uint16_t rawval){
  axis axisinfo = ADCChannels[index];
  int16_t processed = filteravg(index, rawval); //Running average values
  //int16_t processed = rawval;
  if (axisinfo.range[1] != -1){ //Has center
    int16_t distToCenter = processed - axisinfo.range[1];
    int16_t softwarecenter = (axisinfo.range[2]+axisinfo.range[0])/2;
    if (abs(distToCenter) < axisinfo.centerdeadzone){ //Axis is within center dead zone
      processed = softwarecenter;
    }else if (distToCenter > 0){ //Axis is greater than center
      processed = map(processed, axisinfo.range[1] + axisinfo.centerdeadzone, max(axisinfo.range[0], axisinfo.range[2]), softwarecenter, max(axisinfo.range[0], axisinfo.range[2]));
    }else if (distToCenter < 0){ //Axis is less than center
      processed = map(processed, axisinfo.range[1] - axisinfo.centerdeadzone, min(axisinfo.range[0], axisinfo.range[2]), softwarecenter, min(axisinfo.range[0], axisinfo.range[2]));
    }
  }

  
  if (axisinfo.label.equals("x")){ //Horrible horribleness to set axes because I can't be bothered to learn function pointers
    Joystick.setXAxis(processed);
  } else if (axisinfo.label.equals("y")){
    Joystick.setYAxis(processed);
  } else if (axisinfo.label.equals("z")){
    Joystick.setZAxis(processed);
  } else if (axisinfo.label.equals("rx")){
    Joystick.setRxAxis(processed);
  } else if (axisinfo.label.equals("ry")){
    Joystick.setRyAxis(processed);
  } else if (axisinfo.label.equals("rz")){
    Joystick.setRzAxis(processed);
  } else if (axisinfo.label.equals("rudder")){
    Joystick.setRudder(processed);
  } else if (axisinfo.label.equals("throttle")){
    Joystick.setThrottle(processed);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //Shift registers
  
  digitalWrite (LATCH, LOW);    // pulse parallel load latch
  digitalWrite (LATCH, HIGH);
  for (int i = 0; i < NUMSR; i++){ //Iterate through each SR and record the inputs
    SRStates[i] = bbSPI.transfer(0);
    byte mask = 1;
    for (int j = 1; j <= 8; j++) //Iterate through each input pin on the SR
      {
      SRState1D[8*i+j] = SRStates[i] & mask;
      if ((SRStates[i] & mask) != (oldSRStates[i] & mask))
        {
        Serial.print ("Switch ");
        Serial.print (8*i+j);
        Serial.print (" now ");
        Serial.println ((SRStates[i] & mask) ? "open" : "closed"); //This assumes pullup resistors
        
        updateButtonValue(8*i+j, SRStates[i] & mask, oldSRStates[i] & mask);
        
        }  // end of bit has changed
      
      mask <<= 1;  
      }  // end of for each bit
  }

  checkEncoders();

  for (int i = 0; i < NUMSR; i++){
    oldSRStates[i] = SRStates[i];
  }
  for (int i = 0; i < 8*NUMSR; i++){
    oldSRState1D[i] = SRState1D[i];
  }

  SPI.begin(); //Start SPI
  SPI.beginTransaction(settings);
  //Update axes
  for (int i = 0; i < 8; i++){
    //Array with channel definitions for ADC
    uint16_t raw;
    switch (i){
      case 0:
        raw = adc.read(MCP3208::Channel::SINGLE_0);
        //Serial.print("CH1 ");
        //Serial.println(raw);
        break;
      case 1:
        raw = adc.read(MCP3208::Channel::SINGLE_1);
        //Serial.print("CH2 ");
        //Serial.println(raw);
        break;
      case 2:
        raw = adc.read(MCP3208::Channel::SINGLE_2);
        //Serial.print("CH3 ");
        //Serial.println(raw);
        break;
      case 3:
        raw = adc.read(MCP3208::Channel::SINGLE_3);
        //Serial.print("CH4 ");
        //Serial.println(raw);
        break;
      case 4:
        raw = adc.read(MCP3208::Channel::SINGLE_4);
        //Serial.print("CH5 ");
        //Serial.println(raw);
        break;
      case 5:
        raw = adc.read(MCP3208::Channel::SINGLE_5);
        //Serial.print("CH6 ");
        //Serial.println(raw);
        break;
      case 6:
        raw = adc.read(MCP3208::Channel::SINGLE_6);
        //Serial.print("CH7 ");
        //Serial.println(raw);
        break;
      case 7:
        raw = adc.read(MCP3208::Channel::SINGLE_7);
        //Serial.print("CH8 ");
        //Serial.println(raw);
        break;
    }
    processAxis(i, raw);
  }
  SPI.endTransaction(); //End SPI to make sure it does not interfere with bit banged shift register SPI
  SPI.end();

  delay(1);
}
