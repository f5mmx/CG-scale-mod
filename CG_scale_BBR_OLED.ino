// CG-scale originally designed and published by Olav Kallhovd; https://github.com/olkal/CG_scale

/*
  -------------------------------------------------------
  Update by Pointu 23/01/2019:
  - Use of an other OLED I2C display
  - Corrected if EEPROM Value return NAN (not a number) use default
  - Cleaning Code and Comments
  -------------------------------------------------------
  Update by J'm 10/01/2018:
  - Use of OLED I2C display
  - Corrected HX711 initialization
  - Removed timeout no longer needed with OLED display
  -------------------------------------------------------
  Copyright J'm f5mmx, France, 2018 (jmb91650@gmail.com)
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  This is a beerware; if you like it and if we meet some day, you can pay me a beer in return!
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  Only one Arduino board is needed, as OLED-display with i2c-bus is used.
  I used this: 0.96" I2C IIC Serial 128X64 128*64 Blue OLED LCD LED Display Module for Arduino

  Board type used is Arduino Nano (328 16 MHz). A Arduino Pro Mini can also be used,
  but there are no free pins for voltage measurement as HX711's need 4 and i2c-bus 2 analog pins.

  Display is connected via i2c-bus.
  >>>>> to fit to your screen modify function print2lcd at the end this file.

  Load cell amplifiers are used with this library: https://github.com/bogde/HX711

  Connections / pins:
  - rear load cell:   A2-A3
  - front load cell:  A0-A1
  - Oled i2c-bus:     A4-A5 (SDA-SCL)
  - battery voltage   A6

  Calibration function
  It is possible to set the calibration factors with 3 buttons. A known calibration weight is needed.

  Altered values are saved to EEPROM.
  - navigation buttons / pins
      btn 1 digital2 func
      btn 2 digital3 -
      btn 3 digital4 +

  Hold btn 1 down a few seconds and release it to enter calibration menu. Select sensor and steps by clicking btn1, and
  make adjustments to scale factor by clicking buttons 2 and 3 until the weight is what it is supposed to be. Go through
  all the steps, after the last one values are written to EEPROM.

  Voltage divider with R1=10 R2=1 kohm resistors are used, and reference voltage is set to 1.1 V (ARef = internal).

  Operating cycle is defined ONLY by how fast the sensors give the measurements.

  HX711 amplifiers seem to consume quite a lot.
*/

// libraries
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>  //https://github.com/olikraus/U8g2_Arduino
#include <HX711.h>    //https://github.com/bogde/HX711

// Ref to olikraus lib
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);

// cell amplifiers
HX711 front_scale;
HX711 rear_scale;

// if sensor wires are crossed and scale shows negative values...
int invert_front_sensor = -1; // if invert -1 else 1;
int invert_rear_sensor = 1;

// scale sensor states, flags to avoid excess commands (and time consumption)
bool frontOn = false;
bool rearOn = false;

// calibration factors, modify these to adjust the output with known calibration weight...
float sens_cal_1 = 453.25; //default value, this is overwritten with value stored by calibration function (954.0)
float sens_cal_2 = 450.22; //default value, ... (799)

// pins
byte batRefPin = A6; //battery voltage measurement
byte btn1Pin = 2;
byte btn2Pin = 3;
byte btn3Pin = 4;

// EEPROM addresses
unsigned int s1Addr = 0;
unsigned int s2Addr = 4;

// variables
float battValue; //battery voltage
float eW, tW; //measurement results from front/rear load cells
float CGratio; //a ratio that is calculated from values above...
float CG; //final result

// physical dimensions (distances), make sure that these are coherent with the mechanical unit
float WingPegDist = 117.09; //calibration value in mm, projected distance between wing support points, measure with calliper
float LEstopperDist = 30.0; //calibration value in mm, projected distance from front wing support point to leading edge (stopper pin), measure with calliper

float treshold = 5; //min weight, below this "0" is shown

float CGoffset = ((WingPegDist / 2) + LEstopperDist);
//---------------------------------------------------------
// LOCALIZATION - modify these according to your preferences
//---------------------------------------------------------
bool weight_oz = false; //if for some strange reason ounces are preferred instead of grams
String uGr = "g";

// label use, only uncomment the table need
String lbl[10] = {"Balance F5MMX", "Tension Bat:", "Initialisation..", "Masse:", "CG:", "Calibration?", "Bp 1 = Oui", "Sauvegarde...", "Nez", "Queue"};
//String lbl[10] = {"My CG Scale", "Battery:", "Initializing..", "Mass:", "CG:", "Calibrate?", "Bp 1 = Yes", "Saving...", "Noze", "Tail"};

//---------------------------------------------------------
// other parameters
//---------------------------------------------------------
byte n_avg = 6; //number of measurements from which the average is calculated (by the HX711 library) was 8

// button/mode things
byte mode = 0; //0=norm 1,2,3... = calibration
byte btn = 0;
bool btn1dwn, btn2dwn, btn3dwn; //button states
float adjStep = 10; //mode-dependent step
String adjSensor = "..."; //Variable String for label

String prevL1, prevL2; //previous lines printed
byte prevLine;

//-----------------------------------------------------------------
// initial setup
//-----------------------------------------------------------------
void setup()
{
  Serial.begin(9600);

  u8g2.begin();
  u8g2.setFont(u8g2_font_t0_14_tr); // font to use on screen

  //button pin modes
  pinMode(btn1Pin, INPUT_PULLUP); //button1 func
  pinMode(btn2Pin, INPUT_PULLUP); //button2 +
  pinMode(btn3Pin, INPUT_PULLUP); //button3 +

  //change reference voltage to 1.1 v (internal)
  analogReference(INTERNAL);

  // HX711 init pins
  front_scale.begin(A0, A1);
  rear_scale.begin(A2, A3);

  // language et other

  if (weight_oz) {
    uGr = "oz";
  } else {
    uGr = "g";
  }

  // splash
  print2lcd(1, lbl[0]);
  print2lcd(2, lbl[2]);

  // read calibration factors from EEPROM
  readFromEEPROM();
  front_scale.set_scale(sens_cal_1);
  front_scale.tare();
  rear_scale.set_scale(sens_cal_2);
  rear_scale.tare();

  // power up the sensors
  changeScaleState(front_scale, frontOn, true);
  changeScaleState(rear_scale, rearOn, true);

  // initial state
  mode = 0; //normal mode
  btn = 0; //button index, 0=no buttons pressed

  for (int count = 6; count > 0; count--) {
    print2lcd(2, lbl[2] + String(count - 1)); //countdown during init
    delay(50);
  }

  front_scale.tare();
  rear_scale.tare();

}
//---------------------------------------------------------
// main operation
//---------------------------------------------------------
void loop()
{
  areButtonsPressed();   // read buttons state

  switch (mode) {
    case 0: //normal operation
      {
        readSensors(); //read sensor values
        float mass = eW + tW; //total weight
        if (mass < treshold) {
          mass = 0; //less than treshold = 0 g
        }
        float cog = calculateCG(); //calculate CG:

        print2lcd(1, lbl[3] + " " + String(mass, 0) + uGr);

        if (cog > 0) //print CG if feasible
        {
          print2lcd(2, lbl[4] + " " + String(cog, 1) + " mm");
        }
        else
        {
          print2lcd(2, "");
        }
        break;
      }
    case 1: //Question "calibrate?"
      {
        print2lcd(1, lbl[5]);
        print2lcd(2, lbl[6]);
        break;
      }
    case 8:
      {
        print2lcd(1, lbl[7]);
        print2lcd(2, "");
        saveToEEPROM();
        changeScaleState(front_scale, frontOn, true);
        changeScaleState(rear_scale, rearOn, true);
        mode = 0; // back to business
        break;
      }
    default: //cbr modes 1-7
      {
        print2lcd(1, adjSensor + ", Pas=" + String(adjStep, 2));

        if (mode > 1 && mode < 5) //front sensor
        {
          front_scale.set_scale(sens_cal_1); //update scaling factor
          eW = front_scale.get_units(4) * invert_front_sensor * -1 ; //to invert if needed //n_avg
          if (weight_oz == true) eW = eW * 0.035274;

          printGoodVal(sens_cal_1, eW);//print factor and result to lcd
        }
        else //rear sensor
        {
          changeScaleState(front_scale, frontOn, false);
          changeScaleState(rear_scale, rearOn, true);
          rear_scale.set_scale(sens_cal_2); //update scaling factor
          tW = rear_scale.get_units(4) * invert_rear_sensor; // to invert if needed
          if (weight_oz == true) {
            tW = tW * 0.035274;
          }
          printGoodVal(sens_cal_2, tW);//print factor and result to lcd
        }
        break;
      }
  }

} //end of loop

//---------------------------------------------------------
// Functions
//---------------------------------------------------------

// change HX711 state
void changeScaleState (HX711 aScale, bool aState, bool aUp) {
  if (aUp == true and aState == false) {
    aScale.power_up();
    aState = true;
  } else if (aUp == false and aState == true) {
    aScale.power_down();
    aState = false;
  }
}

// format good val for print to lcd
void printGoodVal (float aSens_cal, float aVal) {
  print2lcd(2, String(aSens_cal, 2) + " " + String(aVal, 2) + uGr);
}

// reading the sensor values
// values are put to global variables eW,tW
void readSensors()
{
  eW = front_scale.get_units(n_avg) * -1 * invert_front_sensor; //front
  if (eW < 0) eW = 0; //no negative values
  if (weight_oz == true) eW = eW * 0.035274; //oz?

  tW = rear_scale.get_units(n_avg) * invert_rear_sensor;  //rear
  if (tW < 0) tW = 0; //no negative values
  if (weight_oz == true) tW = tW * 0.035274; //oz?
}

// calculate CG
float calculateCG()
{
  if (eW > treshold && tW > treshold) //proceed only if there are relevant values
  {
    CGratio = tW / (eW + tW);
    return (((WingPegDist) * CGratio)) - ((WingPegDist) / 2) + CGoffset;
  }
  else
  {
    return 0;
  }
}

// read battery voltage
float readBattVoltage()
{
  int val = analogRead(batRefPin);
  return val * 1.1 / 1023 / 0.818 * 9; //multiplier 0.818 is pre-calculated from resistor values and ref voltage.
}

//---------------------------------------------------------
// EEPROM
//---------------------------------------------------------
// read calibration factors
void readFromEEPROM()
{
  // reset EEPROM if button1 = down
  if (digitalRead(2) == 0)
  {
    for (int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0xff);
    print2lcd(1, "EEPROM RST");

    sens_cal_1 = 900;
    sens_cal_2 = 900;

    EEPROM.put(s1Addr, sens_cal_1);
    EEPROM.put(s2Addr, sens_cal_1);

    delay(1);
  }
  else //read values from eeprom
  {
    EEPROM.get(s1Addr, sens_cal_1);
    EEPROM.get(s2Addr, sens_cal_2);
  }
  // care is Not A Number
  if (isnan(sens_cal_1)) {
    sens_cal_1 = 900;
  }
  if (isnan(sens_cal_2)) {
    sens_cal_2 = 900;
  }
}

// save calibration factors
void saveToEEPROM()
{
  EEPROM.put(s1Addr, sens_cal_1);
  EEPROM.put(s2Addr, sens_cal_2);
}
//---------------------------------------------------------
// Buttons Management
//---------------------------------------------------------

// handle buttons
void areButtonsPressed()
{
  btn = 0; //button index, 0 = no buttons pressed
  //detect button up

  if (btn1dwn == false && digitalRead(2) == 0) //b1
  {
    btn1dwn = true;
  } else if (btn1dwn == true && digitalRead(2) == 1) {
    btn1dwn = false;
    btn = 1;
  }

  if (btn2dwn == false && digitalRead(3) == 0)  //b2
  {
    btn2dwn = true;
  } else if (btn2dwn == true && digitalRead(3) == 1) {
    btn2dwn = false;
    btn = 2;
  }

  if (btn3dwn == false && digitalRead(4) == 0) //b3
  {
    btn3dwn = true;
  }
  else if (btn3dwn == true && digitalRead(4) == 1) {
    btn3dwn = false;
    btn = 3;
  }

  //---------------------------------------------------------
  // mode-spesific operations
  //---------------------------------------------------------

  if (mode == 0 && btn == 1)  //from basic operation to calibration mode, question?
  {
    mode = 1; //question
    btn = 0;
  } else if (mode == 1 && btn > 1)  //btn2 or 3 -> cancel, back to mode 0
  {
    mode = 0;
    btn = 0;
  } else if (mode == 1 && btn == 1)  //btn1 = yes --> change to calibration mode a
  {
    mode = 2;
    btn = 0;
  } else  if (mode == 2) //front, step = 10
  {
    if (btn == 1) mode++; //next
    adjSensor = lbl[8];
    adjStep = 10;
    if (btn == 2) sens_cal_1 = sens_cal_1 - adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1 + adjStep;
    btn = 0;
  } else if (mode == 3) //front, step = 1
  {
    if (btn == 1) mode++; //next
    adjSensor = lbl[8];
    adjStep = 1;
    if (btn == 2) sens_cal_1 = sens_cal_1 - adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1 + adjStep;
    btn = 0;
  } else if (mode == 4) //front, step = 0.05
  {
    if (btn == 1) mode++; //next
    adjSensor = lbl[8];
    adjStep = 0.05;
    if (btn == 2) sens_cal_1 = sens_cal_1 - adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1 + adjStep;
    btn = 0;
  } else if (mode == 5) //rear, step = 10
  {
    if (btn == 1) mode++; //next
    adjSensor = lbl[9];
    adjStep = 10;
    if (btn == 2) sens_cal_2 = sens_cal_2 - adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2 + adjStep;
    btn = 0;
  } else if (mode == 6) //rear, step = 1
  {
    if (btn == 1) mode++; //next
    adjSensor = lbl[9];
    adjStep = 1;
    if (btn == 2) sens_cal_2 = sens_cal_2 - adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2 + adjStep;
    btn = 0;
  } else if (mode == 7) //rear, step = 0.05
  {
    if (btn == 1) mode++; //next
    adjSensor = lbl[9];
    adjStep = 0.05;
    if (btn == 2) sens_cal_2 = sens_cal_2 - adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2 + adjStep;
    btn = 0;
  }
}

// printing to OLED
// here you can change parms to fit to your screen
void print2lcd(int line_1_2, String text)
{
  //clear line
  prevLine = line_1_2;
  if (line_1_2 == 1) prevL1 = text; else {
    prevL2 = text;
  }

  u8g2.firstPage();
  do {
    /* pointu screen */
    u8g2.drawFrame(0, 1, 128, 43);
    u8g2.setCursor(6, 18);
    u8g2.print(prevL1);
    u8g2.setCursor(6, 34);
    u8g2.print(prevL2);
    u8g2.setCursor(6, 58);
    u8g2.print(lbl[1] + " ");
    u8g2.print(String(readBattVoltage(), 1) + "V");
    u8g2.drawFrame(0, 43, 128, 64 - 43);

    /* J'm screen
        u8g2.drawFrame(0, 0, 128, 46);
        u8g2.setCursor(3, 14);
        u8g2.print(prevL1);
        u8g2.setCursor(3, 28);
        u8g2.print(prevL2);
        u8g2.setCursor(3, 60);
        u8g2.print(t1 + " ");
        u8g2.print(String(batt, 1) + "V");
        u8g2.drawFrame(0, 48, 128, 64 - 48);
    */

  } while ( u8g2.nextPage() );

  //update timestanp, line number and such
  prevLine = line_1_2;
}

