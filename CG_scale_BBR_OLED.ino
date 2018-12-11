//CG-scale originally designed and published by Olav Kallhovd; https://github.com/olkal/CG_scale

/*
  -------------------------------------------------------
  Update 10/01/2018:
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

  Load cell amplifiers are used with this library: https://github.com/bogde/HX711

  Connections / pins:
  - rear load cell:   A0-A1
  - front load cell:  A2-A3
  - LCD i2c-bus:      A4-A5 (SDA-SCL)
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

//libraries
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <HX711.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);

//load cell amplifiers
HX711 front_scale;
HX711 rear_scale;

//pins
byte batRefPin = A6; //battery voltage measurement
byte button1Pin = 2;
byte button2Pin = 3;
byte button3Pin = 4;

//variable
float battValue; //battery voltage
float eW, tW; //measurement results from front/rear load cells
float CGratio; //a ratio that is calculated from values above...
float CG; //final result

//physical dimensions (distances), make sure that these are coherent with the mechanical unit
float WingPegDist = 119.09; //calibration value in mm, projected distance between wing support points, measure with calliper
float LEstopperDist = 30.0; //calibration value in mm, projected distance from front wing support point to leading edge (stopper pin), measure with calliper

float CGoffset = ((WingPegDist / 2) + LEstopperDist);
//*******************************************************************
//LOCALIZATION - modify these according to your preferences
bool weight_oz = false; //if for some strange reason ounces are preferred instead of grams

//splash text, shown after startup, eg "My own CG-scale"
String t0 = "Balance F5MMX";

//battery status, eg "Battery:"
String t1 = "Tension AQ:";

//What is shown during the initialization phase, eg. "Initializing.."
String t2 = "Initialisation..";

//Label for weight, eg "Weight:"
String t3 = "Masse:";

//Label for CG result, eg "CG:"
String t4 = "CG:";

//Label for calibr. question, eg "Calibrate?";
String t5 = "Calibration?";

//Label for calibr.question row2, eg "Btn 1 = Yes"
String t6 = "Bp 1 = Oui";
//********************************************************************
//other parameters
byte n_avg = 4; //number of measurements from which the average is calculated (by the HX711 library) was 8

//if sensor wires are crossed and scale shows negative values...
bool invert_front_sensor = false; // true;
bool invert_rear_sensor  = false;
byte front1;
byte rear1;

//calibration factors, modify these to adjust the output with known calibration weight...
float sens_cal_1 = 954.0; //default value, this is overwritten with value stored by calibration function
float sens_cal_2 = 799.0; //default value, ...

float treshold = 5; //min weight, below this "0" is shown

//bool battAlarmShown = false; //flag to control batt info display usage (is shown only once per session)
//a counter to count.. something
byte counter = 0; //counter

//EEPROM addresses
unsigned int s1Addr = 0;
unsigned int s2Addr = 4;

//button/mode things
byte mode = 0; //0=norm 1,2,3... = calibration
byte btn = 0;
bool b1down, b2down, b3down; //button states
float adjStep = 10; //mode-dependent step
String adjSensor = "Nez";

//scale sensor states, flags to avoid excess commands (and time consumption)
bool frontOn = false;
bool rearOn = false;

String prevL1, prevL2; //previous lines printed
byte prevLine;


//-----------------------------------------------------------------
//setup
void setup()
{
  Serial.begin(9600);

  u8g2.begin();
  u8g2.setFont(u8g2_font_t0_14_tr);

  if (invert_front_sensor) front1 = -1; else front1 = 1;
  if (invert_rear_sensor) rear1 = -1; else rear1 = 1;

  //button pin modes
  pinMode(button1Pin, INPUT_PULLUP); //button1 func
  pinMode(button2Pin, INPUT_PULLUP); //button2 +
  pinMode(button3Pin, INPUT_PULLUP); //button3 +

  //change reference voltage to 1.1 v (internal)
  analogReference(INTERNAL);

  //HX711 init
  front_scale.begin(A0, A1);
  rear_scale.begin(A2, A3);

  //splash
  print2lcd(1, t0);
  print2lcd(2, t2 + String(5));

  //read calibration factors from EEPROM
  readFromEEPROM();
  front_scale.set_scale(sens_cal_1);
  front_scale.tare();
  rear_scale.set_scale(sens_cal_2);
  rear_scale.tare();

  //power up the sensors
  front_scale.power_up();
  frontOn = true;
  rear_scale.power_up();
  rearOn = true;

  //sw operation mode
  mode = 0; //normal mode

  //button index, 0=no buttons pressed
  btn = 0;
}
//-----------------------------------------------------------------
//main operation
void loop()
{
  //buttons?
  areButtonsPressed();

  if (mode == 0) //normal operation
  {
    //read sensor values
    readSensors();

    //countdown during init
    if (counter < 6 && counter > 0) print2lcd(2, t2 + String(5 - counter));

    //set sensors to zero (tare)
    if (counter == 5)
    {
      front_scale.tare();
      rear_scale.tare();
    }

    //update the counter
    counter++;

    //operation after init phase
    if (counter > 5)
    {
      //total weight
      float mass = eW + tW;
      if (mass < treshold) mass = 0; //less than treshold = 0 g

      //calculate CG:
      float cog = calculateCG();

      //print total mass to lcd
      if (weight_oz == false) print2lcd(1, t3 + " " + String(mass, 0) + " g");
      if (weight_oz == true)  print2lcd(1, t3 + " " + String(mass, 0) + " oz");

      //print CG if feasible
      if (String(cog, 1) != "0.0")
      {
        print2lcd(2, t4 + " " + String(cog, 1) + " mm");
      }
      else
      {
        print2lcd(2, "                               ");
      }

      //counter rotation...
      if (counter > 20) //run between 10 and 20
      {
        counter = 10; //run between 10 and 20
      }
    }
  } //end of mode 0

  //other modes
  //Question "calibrate?"
  if (mode == 1)
  {
    print2lcd(1, t5);
    print2lcd(2, t6);
  }

  //cbr modes 1-7
  if (mode > 1 && mode < 8)
  {
    print2lcd(1, adjSensor + ", Pas=" + String(adjStep, 2));

    if (mode > 1 && mode < 5) //front sensor
    {
      if (frontOn == false) {
        front_scale.power_up();
        frontOn = true;
      }
      if (rearOn == true)   {
        rear_scale.power_down();
        rearOn = false;
      }

      front_scale.set_scale(sens_cal_1); //update scaling factor
      eW = front_scale.get_units(4) * front1 * -1; //to invert if needed //n_avg
      if (weight_oz == true) eW = eW * 0.035274;

      //   Serial.print("front mesure: ");
      //   Serial.println(eW);

      //print factor and result to lcd
      if (weight_oz == false) print2lcd(2, String(sens_cal_1, 2) + " " + String(eW, 2) + " g");
      if (weight_oz == true)  print2lcd(2, String(sens_cal_1, 2) + " " + String(eW, 2) + " oz");
    }
    else //rear sensor
    {
      if (frontOn == true)  {
        front_scale.power_down();
        frontOn = false;
      }
      if (rearOn == false)  {
        rear_scale.power_up();
        rearOn = true;
      }

      rear_scale.set_scale(sens_cal_2); //update scaling factor
      tW = rear_scale.get_units(4) * rear1; // to invert if needed
      if (weight_oz == true) tW = tW * 0.035274;

      //print factor and result to lcd
      if (weight_oz == false) print2lcd(2, String(sens_cal_2, 2) + " " + String(tW, 2) + " g");
      if (weight_oz == true)  print2lcd(2, String(sens_cal_2, 2) + " " + String(tW, 2) + " oz");
    }
  }

  if (mode == 8) //save values to eeprom
  {
    print2lcd(1, "Sauvegarde...");
    print2lcd(2, "");

    saveToEEPROM();

    if (frontOn == false)  {
      front_scale.power_up();
      frontOn = true;
    }
    if (rearOn == false)   {
      rear_scale.power_up();
      rearOn = true;
    }

    mode = 0; // back to business
  }

} //end of loop

//-----------------------------------------------------------------
//Functions
//printing to LCD
void print2lcd(int line_1_2, String text)
{
  float batt;
  batt = readBattVoltage();
  //clear line
  prevLine = line_1_2;
  if (line_1_2 == 1) prevL1 = text;
  if (line_1_2 == 2) prevL2 = text;

  u8g2.firstPage();
  do {
    u8g2.drawFrame(0, 0, 128, 46);
    u8g2.setCursor(3, 14);
    u8g2.print(prevL1);
    u8g2.setCursor(3, 28);
    u8g2.print(prevL2);
    u8g2.setCursor(3, 60);
    u8g2.print(t1 + " ");

    u8g2.print(String(batt, 1) + "V");
    u8g2.drawFrame(0, 48, 128, 64 - 48);
  } while ( u8g2.nextPage() );

  //update timestanp, line number and such
  prevLine = line_1_2;
}

//reading the sensor values
//values are put to global variables eW,tW
void readSensors()
{
  //front
  eW = front_scale.get_units(n_avg) * -1;

  if (invert_front_sensor == true) eW = eW * -1;

  if (eW < 0) eW = 0; //no negative values
  if (weight_oz == true) eW = eW * 0.035274; //oz?

  //rear
  tW = rear_scale.get_units(n_avg);

  if (invert_rear_sensor == true) tW = tW * -1;

  if (tW < 0) tW = 0; //no negative values
  if (weight_oz == true) tW = tW * 0.035274; //oz?
}

//calculate CG
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

//read battery voltage
float readBattVoltage()
{
  int val = analogRead(batRefPin);
  return val * 1.1 / 1023 / 0.818 * 9; //multiplier 0.818 is pre-calculated from resistor values and ref voltage.
}

//handle buttons
void areButtonsPressed()
{
  btn = 0; //button index, 0 = no buttons pressed
  //detect button up
  //b1
  if (b1down == false && digitalRead(2) == 0) {
    b1down = true;
  }
  else {
    if (b1down == true && digitalRead(2) == 1) {
      b1down = false;
      btn = 1;
    }
  }

  //b2
  if (b2down == false && digitalRead(3) == 0) {
    b2down = true;
  }
  else {
    if (b2down == true && digitalRead(3) == 1) {
      b2down = false;
      btn = 2;
    }
  }

  //b3
  if (b3down == false && digitalRead(4) == 0) {
    b3down = true;
  }
  else {
    if (b3down == true && digitalRead(4) == 1) {
      b3down = false;
      btn = 3;
    }
  }

  //-------------------------------------------
  //mode-spesific operations
  //from basic operation to calibration mode, question?
  if (mode == 0 && btn == 1 && counter > 10)
  {
    mode = 1; //question
    btn = 0;
  }

  //btn2 or 3 -> cancel, back to mode 0
  if (mode == 1 && btn > 1)
  {
    mode = 0;
    btn = 0;
  }

  //btn1 = yes --> change to calibration mode a
  if (mode == 1 && btn == 1)
  {
    mode = 2;
    btn = 0;
  }

  //front, step = 10
  if (mode == 2)
  {
    if (btn == 1) mode++; //next
    adjSensor = "Nez";
    adjStep = 10;
    if (btn == 2) sens_cal_1 = sens_cal_1 - adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1 + adjStep;
    btn = 0;
  }
  //front, step = 1
  if (mode == 3)
  {
    if (btn == 1) mode++; //next
    adjSensor = "Nez";
    adjStep = 1;
    if (btn == 2) sens_cal_1 = sens_cal_1 - adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1 + adjStep;
    btn = 0;
  }
  //front, step = 0.05
  if (mode == 4)
  {
    if (btn == 1) mode++; //next
    adjSensor = "Nez";
    adjStep = 0.05;
    if (btn == 2) sens_cal_1 = sens_cal_1 - adjStep;
    if (btn == 3) sens_cal_1 = sens_cal_1 + adjStep;
    btn = 0;
  }

  //rear, step = 10
  if (mode == 5)
  {
    if (btn == 1) mode++; //next
    adjSensor = "Queue";
    adjStep = 10;
    if (btn == 2) sens_cal_2 = sens_cal_2 - adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2 + adjStep;
    btn = 0;
  }
  //rear, step = 1
  if (mode == 6)
  {
    if (btn == 1) mode++; //next
    adjSensor = "Queue";
    adjStep = 1;
    if (btn == 2) sens_cal_2 = sens_cal_2 - adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2 + adjStep;
    btn = 0;
  }
  //rear, step = 0.05
  if (mode == 7)
  {
    if (btn == 1) mode++; //next
    adjSensor = "Queue";
    adjStep = 0.05;
    if (btn == 2) sens_cal_2 = sens_cal_2 - adjStep;
    if (btn == 3) sens_cal_2 = sens_cal_2 + adjStep;
    btn = 0;
  }
}

//EEPROM
//read calibration factors
void readFromEEPROM()
{
  //reset EEPROM if button1 = down
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
}

//save calibration factors
void saveToEEPROM()
{
  EEPROM.put(s1Addr, sens_cal_1);
  EEPROM.put(s2Addr, sens_cal_2);
}

