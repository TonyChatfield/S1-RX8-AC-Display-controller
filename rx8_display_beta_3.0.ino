/*****************************************************
*
*  S1-RX8-AC-Display-controller
*  Copyright (c) 2017 TonyChatfield
*  for Arduino Mega 2560
*
******************************************************/

#include <SPI.h>
#include <Time.h>
#include <Wire.h>
//#include "DS3231.h"
#include "RTClib.h"

RTC_DS1307 rtc;
//RTC_DS3231 rtc;

#define acAmpTX 8
#define acAmpRX 12

//PORTA Inputs
#define matrixRow2 22
#define matrixRow4 23
#define matrixRow1 24
#define matrixRow3 25
#define rearDemistLED 26
#define AirConLED 27
#define reCircLED 28
#define matrixColB 29

//PORTB
#define DATAOUT 51 // MOSI
#define SPICLOCK  52 //sck
#define ssPin 53 // PB0

//PORTC Outputs
#define freshAirLED 30
#define matrixColA 31
#define frontDemistLED 32
#define fanInOne 33
#define autoLED 34
#define tempInTwo 35
#define fanInTwo 36
#define tempInOne 37

//PORT L
#define mode0 49
#define mode1 48

//acAmpTX
byte acAmpDataOut[5];
byte acAmpDataDefault[5] = {0x04, 0x80, 0x80, 0x80, 0xFC}; //Default data for the A\C amplifier TX; basicly it means I am here and no buttons have been pressed.
unsigned long acAmpTxBetweenTime;
unsigned long acAmpTxWaitTime = 18; //Control the cycle time otherwise we have a continuous data stream allowing for processing time this equates to 6-7 ms (most of the time).

// Push button matrix stuff
byte buttonCount = 0;
byte pushedButtonOld = 0;
byte pushedButtonCurrent = 0;
byte longPushButton = 0;
byte shortPushButton = 0;
long buttonPushed = 0;
byte buttonDebounce = 50;
bool buttonCycle = false;
byte buttonControl = 15; //Control the point where shortPushButton becomes longPushButton

// Rotary switches
bool aFanOld = 0;
bool bFanOld = 0;
bool aTempOld = 0;
bool bTempOld = 0;

// Menu
byte menuIndexMin = 0;
byte menuIndexMax = 255;
byte menuIndex = 0;
byte menuValueMin = 0;
byte menuValueMax = 255;
byte menuValue = 0;
bool menuStartValue = false;
bool setHourFirst = true;
bool inMenu = false;


//RX data stuff
byte acAmpIndex = 0;
unsigned int acAmpNewByte;
unsigned int acAmpRxData[6]; //Where we keep our inbound data.
unsigned long acAmpReceivedTime;
unsigned long acAmpCycleTime;
bool acAmpOn = false;
bool acAmpRunning = false;
bool acAmpRxChanged = true;
bool acAmpMessageToProcess = false;
bool acAmpDisplayClear = false;

//Data processing
byte currentTemp;
byte currentHour;
byte currentMinute;
byte previousMinute;
unsigned long nowTimeCycle;
byte nowHour = 0;
byte nowMinute = 0;
bool minuteChange = false;
bool acAmpCentigrade = true;
bool clock24Hour = true; //false = 12 hour clock
bool acAmpAmbient = false;
bool confMode = false;

// iconArray for display icons 90-9D, first 5 LSB, calculated via addition. iconArray[0] = current, iconArray[1] = LCD display registry values.
byte iconArray[2][14] = {0};
byte sevenSegmentArray[1][7] = {0};


/**********************************************
// User configurable
**********************************************/
String welcome = "Test Beta2.0"; // populate something into the text display - 12 Char max anything else is truncated.


void setup() {
  // we use iconArray[1] to define the registers we write too with the data thats populated into iconArray[0]
  iconArray[1][0] = 0x90; iconArray[1][1] = 0x91; iconArray[1][2] = 0x92; iconArray[1][3] = 0x93; iconArray[1][4] = 0x94; iconArray[1][5] = 0x95; iconArray[1][6] = 0x96;
  iconArray[1][7] = 0x97; iconArray[1][8] = 0x98; iconArray[1][9] = 0x99; iconArray[1][10] = 0x9A; iconArray[1][11] = 0x9B; iconArray[1][12] = 0x9C; iconArray[1][13] = 0x9D;

  // Control pins for the display SPI
  pinMode(mode0, OUTPUT);
  pinMode(mode1, OUTPUT);
  pinMode(ssPin, OUTPUT);

  //Push buttons and rotary switches
  pinMode(matrixColA, OUTPUT);
  pinMode(matrixColB, OUTPUT);
  pinMode(matrixRow1, INPUT_PULLUP);
  pinMode(matrixRow2, INPUT_PULLUP);
  pinMode(matrixRow3, INPUT_PULLUP);
  pinMode(matrixRow4, INPUT_PULLUP);
  pinMode(fanInOne, INPUT_PULLUP);
  pinMode(fanInTwo, INPUT_PULLUP);
  pinMode(tempInOne, INPUT_PULLUP);
  pinMode(tempInTwo, INPUT_PULLUP);

  //Control panel LED's
  pinMode(rearDemistLED, OUTPUT);
  pinMode(frontDemistLED, OUTPUT);
  pinMode(freshAirLED, OUTPUT);
  pinMode(autoLED, OUTPUT);
  pinMode(AirConLED, OUTPUT);

  // Real time clock
  rtc.begin();
  //rtc.adjust(DateTime(2014, 1, 21, 4, 33, 0)); //a fixed time
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // your PC's time. ***** You may need to comment out after 1 upload to set the time - see wiki*******
  DateTime now = rtc.now();
  nowHour = now.hour();
  nowMinute = now.minute();


  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); //I couldn't see a defined max speed in the NJU6623 datasheet, 10Meg is quite an increase on the original 9600, so need to monitor incase this is not stable.
 // Serial.begin (115200, SERIAL_8N1);
  Serial2.begin (4800, SERIAL_8E1);

  delay(300);
  lcdDisplayStartUp(); // run the LCD default settings and clear the display icons

}

void loop()
{
  checkAndSendHardwareControl();
  calculateTime();

  if (!confMode)
  {
    acAmpGetSerialData();

    if (millis() - acAmpCycleTime >= 200) // we force the state of acAmpRxChanged if there has been no recent data update, this way we minimise unnecessary processing but at acAmp power off our icons clear more\less instantly (to the human eye).
    {
      acAmpRxChanged = true;
      // Enabling these would give us a timed refresh regardless of change, this may be needed if we encounter issues\interference
      // acAmpMessageToProcess = true;
      // acAmpCycleTime = millis();
    }

    if (acAmpMessageToProcess && acAmpRxChanged || minuteChange)
    {
      acAmpLedFuntions(acAmpOn);
      if (acAmpOn)
      {
        acAmpRecievedStatus();

        acAmpProcessAC();
        acAmpProcessFan();
        acAmpProcessTemp();
        acAmpProcessModeControls();
      }

      iconArrayCheckThenSet();
      sevenSegmentCheckThenSet();
      populateFixedLcdDisplay(welcome);
      minuteChange = false;
      acAmpRxChanged = false;
      acAmpMessageToProcess = false;
    }
  }
  else
  {
    acAmpOn = false;
    //This will be confMode
    confMenu();
    sevenSegmentCheckThenSet();
    iconArrayCheckThenSet();
  }
}


/*****************************************************************
// acAmp inbound
*****************************************************************/

void acAmpRecievedStatus() // Based on the first byte of our inbound message we have 3 main data states from AC Amplifier. Off, Normal control and Ambient
{
  if (acAmpRxData[0] == 0x0F) // we are off apart from showing Mode settings
  {
    acAmpRunning = false;
  }

  acAmpAmbient = (acAmpRxData[0] == 0x0E) ? true : false; // 0D Normal || 0E Ambient

}



void acAmpProcessModeControls() // acAmp control settings. Mode 1 = feet, 2 = feet\demist, 3 = face, 4 = face\feet
{ byte acAmpMode;

  if (bitRead(acAmpRxData[4], 0) && bitRead(acAmpRxData[4], 1))
  {
    acAmpMode = 1;
  }
  else if (bitRead(acAmpRxData[4], 0) && !bitRead(acAmpRxData[4], 1))
  {
    acAmpMode = 2;
  }
  else if (!bitRead(acAmpRxData[4], 0) && !bitRead(acAmpRxData[4], 1))
  {
    acAmpMode = 3;
  }
  else if (!bitRead(acAmpRxData[4], 0) && bitRead(acAmpRxData[4], 1))
  {
    acAmpMode = 4;
  }
  iconCalculateMode(acAmpMode);
}



void acAmpProcessFan() // Find the current fan speed
{ byte acAmpFanSpeed = 0;
  bitWrite(acAmpFanSpeed, 2, bitRead(acAmpRxData[2], 6));
  bitWrite(acAmpFanSpeed, 1, bitRead(acAmpRxData[2], 5));
  bitWrite(acAmpFanSpeed, 0, bitRead(acAmpRxData[2], 4));

  acAmpFanSpeed = (acAmpFanSpeed <= 6) ? ++acAmpFanSpeed : 0;
  acAmpRunning = (acAmpFanSpeed) ? true : false;
  iconCalculateFan(acAmpFanSpeed);
}



void acAmpProcessAC() //The 3 A/C States
{
  if (!bitRead(acAmpRxData[3], 6)) // Auto
  {
    iconArray[0][7] += 0x10;
  }
  if (!bitRead(acAmpRxData[3], 5)) // A/C
  {
    iconArray[0][8] += 0x08;
  }
  if (!bitRead(acAmpRxData[3], 4)) // ECO
  {
    iconArray[0][8] += 0x10;
  }
}



// Need to set leds based on acAmp button state, we are directly manipulating the ports.
void acAmpLedFuntions(bool _acAmpOn)
{
  if (_acAmpOn)
  {
    PORTC = (!bitRead(acAmpRxData[4], 6)) ? PORTC |= (1 << PC5) : PORTC &= ~(1 << PC5); // FrontDemist
    PORTA = (!bitRead(acAmpRxData[4], 5)) ? PORTA |= (1 << PA4) : PORTA &= ~(1 << PA4); // RearDemist
    PORTC = (bitRead(acAmpRxData[4], 3)) ? PORTC &= ~(1 << PC7) : PORTC |= (1 << PC7); // Fresh
    PORTA = (bitRead(acAmpRxData[4], 3)) ? PORTA |= (1 << PA6) : PORTA &= ~(1 << PA6); // Recirc
    PORTC = (!bitRead(acAmpRxData[3], 6)) ? PORTC |= (1 << PC3) : PORTC &= ~(1 << PC3); // Auto
    PORTA = (!bitRead(acAmpRxData[3], 5)) ? PORTA |= (1 << PA5) : PORTA &= ~(1 << PA5); // A/C
  }
  else // When the acAmp is off, We turn off all the LED's.
  {
    PORTC &= ~(1 << PC5);
    PORTA &= ~(1 << PA4);
    PORTC &= ~(1 << PC7);
    PORTA &= ~(1 << PA6);
    PORTC &= ~(1 << PC3);
    PORTA &= ~(1 << PA5);
  }
}



// Get data from the acAmplifier. Note we don't sit and wait around for a full payload.
void acAmpGetSerialData()
{
  if (Serial2.available() > 0)
  { //acAmpOn = true;
    acAmpReceivedTime = millis();
    acAmpNewByte = Serial2.read();
    acAmpDisplayClear = false;

    if (acAmpNewByte == 0x0F || acAmpNewByte == 0x0D || acAmpNewByte == 0x0E) // We don't know the last byte but we know the first byte will be one of these
    {
      acAmpIndex = 0;
      if (acAmpNewByte != acAmpRxData[acAmpIndex])
      {
        acAmpRxData[acAmpIndex] = acAmpNewByte;
        acAmpRxChanged = true;
      }
    }
    else
    {
      acAmpIndex++;
      if (acAmpNewByte != acAmpRxData[acAmpIndex])
      {
        acAmpRxData[acAmpIndex] = acAmpNewByte;
        acAmpRxChanged = true;

        if (acAmpIndex == 5) //we know the length of the data transfer. If we knew how to calculate the checksum we could validate received data but for now we assume it's correct.
        {
          acAmpOn = (acAmpRxData[4] != 0xFF) ? true : false ; // Stops icon flicker at startup
          acAmpMessageToProcess = true;
          acAmpIndex = 0;
          acAmpCycleTime = millis();
        }
      }
    }
  }
  else
  {
    if (millis() - acAmpReceivedTime > 30 && !acAmpDisplayClear) // If there is no data we assume the A\C amplifier is off and we run 1 cycle to clear the display. This is easier than having to monitor assorted vehicle power states.
    {
      acAmpOn = false;
      acAmpMessageToProcess = true;
      acAmpDisplayClear = true;
    }
  }
}



/*****************************************************************
  // Icon display
  // For the icon displays we calculate a binary value for the 5 seperate registers with simple addition in hex
  //                      Position\Register 0x01,0x02,0x04,0x08,0x10
  // int icon90 = [00]; mM R/H lower, mM lower, fullstop between Minutes, Mm R/H lower, Mn lower
  // int icon91 - [01]; mM L/H upper, mM Upper, mM R/H upper, mM centre, L/H vertical divider
  // int icon92 = [02]; Mm L/H upper, Mn upper, Mm R/H upper, Mm centre, mM L/H lower
  // int icon93 = [03]; not used, not used, HH:MM Divider, Mm L/H lower, CD IN
  // int icon94 = [04]; not used, not used, not used, not used, MD IN
  // int icon95 = [05]; Dolby Logo, not used, not used, not used, ST
  // int icon96 = [06]; AF, AMB, Thermo symbol, degF, degC
  // int icon97 = [07]; Car logo (aircon), PTY, fresh air Arrow, recirculate Arrow, AUTO
  // int icon98 = [08]; fan4, RPT, fan5, A\C, ECO
  // int icon99 = [09]; not used, Fan base, fan1, fan2, fan3
  // int icon9A = [10]; Auto-M, TP, TA, RDM, Full Stop between DDRAM #10 & #11
  // int icon9B = [11]; not used, fan7, not used, R/H vertical divider, Full Stop between DDRAM #11 & #12
  // int icon9C = [12]; Seated Man, Front Demist, Down\Feet Arrow, not used, not used
  // int icon9D = [13]; not used, Temp Decimal, Face Arrow, fan6, mid display colon,
*****************************************************************/

void iconCalculateMode(byte _modeSet) // Mode 1 = feet, 2 = feet\demist, 3 = face, 4 = face\feet
{
  switch (_modeSet)
  {
    case 1: iconArray[0][12] += 0x04;
      break;
    case 2: if (!bitRead(acAmpRxData[4], 2) && bitRead(acAmpRxData[4], 6)) // feet/demist has dependancies on addition demist related bytes
      {
        iconArray[0][12] += 0x02;
        iconArray[0][12] += 0x04;
      }
      else if (bitRead(acAmpRxData[4], 2))
      {
        iconArray[0][12] += 0x02;
      }
      break;
    case 3: iconArray[0][13] += 0x04;
      break;
    case 4: iconArray[0][12] += 0x04; iconArray[0][13] += 0x04;
      break;
  }
}



// Set fan speed icons including the base outline
void iconCalculateFan(byte _fanSpeed)
{
  switch (_fanSpeed)
  {
    case 1: iconArray[0][9] += 0x06;
      break;
    case 2: iconArray[0][9] += 0x0E;
      break;
    case 3: iconArray[0][9] += 0x1E;
      break;
    case 4: iconArray[0][8] += 0x01; iconArray[0][9] += 0x1E;
      break;
    case 5: iconArray[0][8] += 0x05; iconArray[0][9] += 0x1E;
      break;
    case 6: iconArray[0][8] += 0x05; iconArray[0][9] += 0x1E; iconArray[0][13] += 0x08;
      break;
    case 7: iconArray[0][8] += 0x05; iconArray[0][9] += 0x1E; iconArray[0][13] += 0x08; iconArray[0][11] += 0x02;
      break;
  }
}



//Calculate the seven segment display values that are generated via the icon matrix, Mm and mM
void iconCalculateMinute(byte _digit, byte _value)
{
  if (_digit == 1) //Mm
  {
    switch (_value)
    {
      case 0: iconArray[0][0] += 0x18; iconArray[0][2] += 0x07; iconArray[0][3] += 0x08;
        break;
      case 1: iconArray[0][0] += 0x08; iconArray[0][2] += 0x04; iconArray[0][3] += 0x00;
        break;
      case 2: iconArray[0][0] += 0x10; iconArray[0][2] += 0x0E; iconArray[0][3] += 0x08;
        break;
      case 3: iconArray[0][0] += 0x18; iconArray[0][2] += 0x0E; iconArray[0][3] += 0x00;
        break;
      case 4: iconArray[0][0] += 0x08; iconArray[0][2] += 0x0D; iconArray[0][3] += 0x00;
        break;
      case 5: iconArray[0][0] += 0x18; iconArray[0][2] += 0x0B; iconArray[0][3] += 0x00;
        break;
        // we should never see these for Mm minute, but just incase someone wants them for an alternate purpose.
        // case 6: iconArray[0][0] += 0x18; iconArray[0][2] +=0x0B; iconArray[0][3] += 0x08;
        // break;
        // case 7: iconArray[0][0] += 0x08; iconArray[0][2] +=0x06; iconArray[0][3] += 0x00;
        // break;
        // case 8: iconArray[0][0] += 0x18; iconArray[0][2] +=0x0F; iconArray[0][3] += 0x08;
        // break;
        // case 9: iconArray[0][0] += 0x08; iconArray[0][2] +=0x0F; iconArray[0][3] += 0x00;
        // break;
    }
  }
  else if (_digit == 2) //mM
  {
    switch (_value)
    {
      case 0: iconArray[0][0] += 0x03; iconArray[0][1] += 0x07; iconArray[0][2] += 0x10;
        break;
      case 1: iconArray[0][0] += 0x01; iconArray[0][1] += 0x04; iconArray[0][2] += 0x00;
        break;
      case 2: iconArray[0][0] += 0x02; iconArray[0][1] += 0x0E; iconArray[0][2] += 0x10;
        break;
      case 3: iconArray[0][0] += 0x03; iconArray[0][1] += 0x0E; iconArray[0][2] += 0x00;
        break;
      case 4: iconArray[0][0] += 0x01; iconArray[0][1] += 0x0D; iconArray[0][2] += 0x00;
        break;
      case 5: iconArray[0][0] += 0x03; iconArray[0][1] += 0x0B; iconArray[0][2] += 0x00;
        break;
      case 6: iconArray[0][0] += 0x03; iconArray[0][1] += 0x0B; iconArray[0][2] += 0x10;
        break;
      case 7: iconArray[0][0] += 0x01; iconArray[0][1] += 0x06; iconArray[0][2] += 0x00;
        break;
      case 8: iconArray[0][0] += 0x03; iconArray[0][1] += 0x0F; iconArray[0][2] += 0x10;
        break;
      case 9: iconArray[0][0] += 0x03; iconArray[0][1] += 0x0F; iconArray[0][2] += 0x00;
        break;
    }
  }
}



// Default values for iconArray [0][1] vertical divider, [0][3] HH:MM divider, [0][11] vertical divider, iconArray[1][x] = hex mapping for display address.
void iconDefaultDisplayValues()
{
  iconArray[0][1] += 0x10; // Vertical divider R/H
  iconArray[0][11] += 0x08; // Vertical divider L/H
  iconArray[0][3] += 0x04; // HH:MM Divider

  if (acAmpOn)
  {
    iconArray[0][12] += 0x01; // The seated man
    iconArray[0][7] = (bitRead(acAmpRxData[4], 3)) ? iconArray[0][7] += 0x09 : iconArray[0][7] += 0x05; //airSource
    if (acAmpAmbient)
    {
      iconArray[0][6] += 0x02; // AMB
      iconArray[0][6] = (acAmpCentigrade) ? iconArray[0][6] += 0x10 : iconArray[0][6] += 0x08; // C or F
    }
    if (acAmpRunning && !acAmpAmbient)
    {
      iconArray[0][13] += 0x02; // decimal place in TMP
      iconArray[0][6] += 0x04; // Thermo symbol
    }
  }
}



void iconArrayCheckThenSet()
{
  minuteCalculate(currentMinute); //Must be recalculated every time we update Icons regardless of change.
  iconDefaultDisplayValues();     //Must be recalculated every time we update Icons regardless of change.
  for (int i = 0; i < 14; i++)
  {
    lcdDisplaySetIconDisplay(iconArray[1][i], iconArray[0][i]);
    iconArray[0][i] = 0;
  }
}


/*****************************************************************
  // SevenSegment Display
  // Calculate the seven segmeent display values that are generated via CGRam for Hh, hH, Tmp, tMp, tmP
  // Parameters; the HEX value for the bit mapped to column (0x10 = 8C Hh, 0x08 = 8C hH, 0x01 = 8D Tmp, 0x04 = 8D tMp, 0x02 = 8C tmP)
*****************************************************************/

void sevenSegmentCalculate(byte _column, byte _value)
{
  switch (_value)
  {
    case 0: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][1] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][4] += _column; sevenSegmentArray[0][5] += _column; sevenSegmentArray[0][6] += _column;
      break;
    case 1: sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][5] += _column;
      break;
    case 2: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][4] += _column; sevenSegmentArray[0][6] += _column;
      break;
    case 3: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][5] += _column; sevenSegmentArray[0][6] += _column;
      break;
    case 4: sevenSegmentArray[0][1] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][5] += _column;
      break;
    case 5: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][1] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][5] += _column; sevenSegmentArray[0][6] += _column;
      break;
    case 6: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][1] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][4] += _column; sevenSegmentArray[0][5] += _column; sevenSegmentArray[0][6] += _column;
      break;
    case 7: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][5] += _column;
      break;
    case 8: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][1] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][4] += _column; sevenSegmentArray[0][5] += _column; sevenSegmentArray[0][6] += _column;
      break;
    case 9: sevenSegmentArray[0][0] += _column; sevenSegmentArray[0][1] += _column; sevenSegmentArray[0][2] += _column; sevenSegmentArray[0][3] += _column; sevenSegmentArray[0][5] += _column;
      break;
  }
}



void sevenSegmentCheckThenSet()
{
  hourCalculate(currentHour); //we include this here as must be reclaculated everytime we update regardless of change.
  lcdDisplayPopulateCgramAddress(0x00);
}


/*****************************************************************
  // Time and temp
*****************************************************************/

void calculateTime()
{
  if (millis() - nowTimeCycle >= 10000 || confMode) // 'DateTime now' takes over 1 milliSecond to return a result, so we don't run it every cycle.
  {
    DateTime now = rtc.now();
    nowTimeCycle = millis();
    nowHour = now.hour();
    nowMinute = now.minute();
  }

  if (!clock24Hour && nowHour > 12) // 12 or 24 hour, for now set in main controls via clock24Hour
  {
    currentHour = nowHour - 12;
  }
  else
  {
    currentHour = nowHour;
  }

  currentMinute = nowMinute;



  //If the hour or minute are continually updating the other icons flicker\vanish as they are not set constantly, so we flag a minute change to manage display updates.
  minuteChange = (previousMinute != currentMinute) ? true : false;
  previousMinute = currentMinute;
}



//Process Hour as 2 separate digits
void hourCalculate(byte _currentHour)
{
  byte theHourHh = _currentHour / 10;
  sevenSegmentCalculate(0x10, theHourHh); //wirte to the sevenSegment array

  byte theHourhH = _currentHour % 10;
  sevenSegmentCalculate(0x08, theHourhH); //wirte to the sevenSegment array
}



// Process Minute as 2 separate digits
void minuteCalculate(byte _currentMinute)
{
  byte theMinuteMm = _currentMinute / 10;
  iconCalculateMinute(1, theMinuteMm); // write to the Icon array

  byte theMinutemM = _currentMinute % 10;
  iconCalculateMinute(2, theMinutemM); // write to the Icon array
}



//Define temperature control mapping - The F calculation incomplete
void acAmpProcessTemp()
{
  byte currentTemp = 0;
  byte displayOrder = 0;
  byte fTemp = 0;
  unsigned int tempOrder[3] = {0x01, 0x04, 0x02};
  for (byte i = 0; i <= 2; i++)
  {
    displayOrder = tempOrder[i];

    for (byte j = 0; j <= 3; j++)
    {
      bitWrite(currentTemp, j, bitRead(acAmpRxData[i + 1], j));
    }

    if (minuteChange || acAmpMessageToProcess) //as the temp is via seven segment, we need to make sure we update if the time changes.
    {
      sevenSegmentCalculate(displayOrder, currentTemp); //write to the sevenSegment array
    }
  }
}


void updateClockTime()
{
  inMenu = true;
  if (shortPushButton == 1)
  {
    setHourFirst = (!setHourFirst) ? true : false;
    menuStartValue = false;
  }

  if (setHourFirst && inMenu)
  {
    menuValueMax = (clock24Hour) ? 23 : 12;
    populateFixedLcdDisplay("Set Hour");
    menuValue = (!menuStartValue) ? currentHour : menuValue ;
    currentHour = menuValue;
    menuStartValue = true;
    DateTime now = rtc.now();
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), menuValue, now.minute(), now.second()));
  }

  if (!setHourFirst && inMenu)
  {
    menuValueMax = 59;
    populateFixedLcdDisplay("Set Minute");
    menuValue = (!menuStartValue) ? currentMinute : menuValue ;
    currentMinute = menuValue;
    menuStartValue = true;
    DateTime now = rtc.now();
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), menuValue, now.second()));

  }
}


/*****************************************************************
  // Hardware controls
*****************************************************************/

void checkAndSendHardwareControl()// Check hardware controls and push to A/C Amplifier with a controlled cycle
{
  if ((millis() - acAmpTxBetweenTime >= acAmpTxWaitTime))
  {
    memcpy(acAmpDataOut, acAmpDataDefault, 10);
    checkFanRotation();
    checkTempRotation();
    checkPushedButton();
    if (!confMode)
    {
      acAmpSend();
    }

  }
}



void checkFanRotation()
{
  bool aFanNew = 0;
  bool bFanNew = 0;
  aFanNew = (PINC & (1 << PC4));
  bFanNew = (PINC & (1 << PC1));

  if  (aFanNew != aFanOld || bFanNew != bFanOld)
  {
    if ((aFanNew && !aFanOld && !bFanNew && !bFanOld) || (!aFanNew && aFanOld && bFanNew && bFanOld) )  //Right movement
    {
      if (!confMode)
      {
        acAmpDataOut[3] = 0x090; acAmpDataOut[4] = 0xEC;
      }
      else
      {
        menuIndex = (menuIndex < menuIndexMax) ? ++menuIndex : menuIndexMin ;
      }
    }
    else if ((aFanNew && aFanOld && !bFanNew && bFanOld) || (!aFanNew && !aFanOld && bFanNew && !bFanOld)) //Left movement
    {
      if (!confMode)
      {
        acAmpDataOut[3] = 0x0F0; acAmpDataOut[4] = 0x8C;
      }
      else
      {
        menuIndex = (menuIndex > menuIndexMin) ? --menuIndex : menuIndexMax ;
      }
    }
  }
  aFanOld = aFanNew;
  bFanOld = bFanNew;
}



void checkTempRotation()
{
  bool aTempNew = 0;
  bool bTempNew = 0;
  aTempNew = (PINC & (1 << PC0));
  bTempNew = (PINC & (1 << PC2));

  if  (aTempNew != aTempOld || bTempNew != bTempOld)
  {
    if ((aTempNew && !aTempOld && !bTempNew && !bTempOld) || (!aTempNew && aTempOld && bTempNew && bTempOld) )  //Right movement
    {
      if (!confMode)
      {
        acAmpDataOut[3] = 0x081; acAmpDataOut[4] = 0xFB;
      }
      else
      {
        menuValue = (menuValue < menuValueMax) ? ++menuValue : menuValueMin ;
      }
    }
    else if ((aTempNew && aTempOld && !bTempNew && bTempOld) || (!aTempNew && !aTempOld && bTempNew && !bTempOld)) //Left movement
    {
      if (!confMode)
      {
        acAmpDataOut[3] = 0x087; acAmpDataOut[4] = 0xF5;
      }
      else
      {
        menuValue = (menuValue > menuValueMin) ? --menuValue : menuValueMax ;
      }
    }
  }
  aTempOld = aTempNew;
  bTempOld = bTempNew;
}



void checkPushedButton()
{
  bool matrixCycle = 0;
  bool controlButton = false;

  checkMatrixCycle();  // Buttons have a matrix switch arrangement.

  if (pushedButtonCurrent == 0 && buttonCycle) //Long button press is single shot, so must be reset by removing finger.
  {
    buttonCycle = false;
  }

  if ((!longPushButton && pushedButtonCurrent == pushedButtonOld) && pushedButtonCurrent && (millis() - buttonPushed >= buttonDebounce)) // index the button count
  {
    buttonCount++;
    buttonPushed = millis();
  }

  if (pushedButtonCurrent != pushedButtonOld)
  {
    if (pushedButtonCurrent == 0 && buttonCount > 0 && buttonCount < buttonControl) // Short button press trigger.
    {
      shortPushButton = pushedButtonOld;
    }
    buttonPushed = millis();
    pushedButtonOld = pushedButtonCurrent;
    buttonCount = 0;
  }

  if (!confMode && shortPushButton) // Do something with a short button press
  {
    shortButtonAction(shortPushButton);
    shortPushButton = 0;
  }

  if (!buttonCycle && pushedButtonCurrent && buttonCount > buttonControl) // Do something with a long button press
  {
    longPushButton = pushedButtonCurrent;
    longButtonAction(longPushButton);
    if (!confMode || longPushButton == 2)
    {
      longPushButton = 0;
    }
    buttonCycle = true;
  }
}



void checkMatrixCycle() // we poll the pin matrix this takes 2 cycles and we need to set pins high\low
{
  pushedButtonCurrent = 0;
  for (byte i = 0; i <= 1; i++)
  {
    delayMicroseconds(2);
    if (i == 0)
    {
      PORTC |= (1 << PC6);
      PORTA &= ~(1 << PA7);
    }
    else
    {
      PORTC &= ~(1 << PC6);
      PORTA |= (1 << PA7);
    }

    if (!(PINA & (1 << PA2)))
    {
      if (i == 1)
      {
        pushedButtonCurrent = 1; // Auto switch
        break;
      }
      else
      {
        pushedButtonCurrent = 2; // Mode switch
        break;
      }
    }
    else if (!(PINA & (1 << PA0)))
    {
      if (i == 1)
      {
        pushedButtonCurrent = 3; // A/C switch
        break;
      }
      else
      {
        pushedButtonCurrent = 4; // frontDemist switch
        break;
      }
    }
    else if (!(PINA & (1 << PA3)))
    {
      if (i == 1)
      {
        pushedButtonCurrent = 5; //rearDemist switch
        break;
      }
      else
      {
        pushedButtonCurrent = 6; // AirSource switch
        break;
      }
    }
    else if (!(PINA & (1 << PA1)))
    {
      if (i == 1)
      {
        pushedButtonCurrent = 7; //Error if we get here with RX8
        break;
      }
      else
      {
        pushedButtonCurrent = 8; // Off switch
        break;
      }
    }
  }
}



void shortButtonAction(byte _shortPushButton)
{
  if (!confMode) // Lookup the correct transmission data if normally running
  {
    switch (_shortPushButton)
    {
      case 1: acAmpDataOut[1] = 0x82; acAmpDataOut[4] = 0xFA;
        break;
      case 2: acAmpDataOut[1] = 0x90; acAmpDataOut[4] = 0xEC;
        break;
      case 3: acAmpDataOut[1] = 0x84; acAmpDataOut[4] = 0xF8;
        break;
      case 4: acAmpDataOut[1] = 0xA0; acAmpDataOut[4] = 0xDC;
        break;
      case 5: acAmpDataOut[1] = 0xC0; acAmpDataOut[4] = 0xBC;
        break;
      case 6: acAmpDataOut[1] = 0x88; acAmpDataOut[4] = 0xF4;
        break;
      case 7://we should never get here with RX8;
        break;
      case 8: acAmpDataOut[1] = 0x81; acAmpDataOut[4] = 0xFB;
        break;
    }
  }
}



void longButtonAction(byte _longPushButton)
{
  //pushedLongButton
  if (!confMode) // we can set control options on long button press
  {
    switch (_longPushButton)
    {
      case 1 :
        break;
      case 2 :  confMode = true; menuIndex = 1;
        break;
      case 3 :
        break;
      case 4 :
        break;
      case 5 :
        break;
      case 6 :
        break;
      case 7 :
        break;
      case 8 : acAmpAmbient = !acAmpAmbient ? true : false; acAmpDataOut[2] = 0xA0; acAmpDataOut[4] = 0xDC;
        break;
    }
  }
  else if (confMode && _longPushButton == 2) //Enusre there is an exit for confMode incase menu is buggy
  {
    confMode = false;
    acAmpOn = true;
    acAmpRxChanged = true;
    acAmpMessageToProcess = true;
    menuValue = 0;
    menuStartValue = false;
  }
}



void acAmpSend() //Send hardware changes to the AC Amplifier
{
  for (byte acAmpCycle = 0; acAmpCycle < 5; acAmpCycle ++)\
  {
    Serial2.write(acAmpDataOut[acAmpCycle]);
  }
  acAmpTxBetweenTime = millis();
}


/*****************************************************************
  // Menu
*****************************************************************/

void confMenu()
{
  menuIndexMin = 1; // Make sure we start at menu position 1.
  menuIndexMax = 3;
  inMenu = false;

  if (longPushButton == 8) // Roll back the menu
  {
    confMode = false;
    acAmpOn = true;
    acAmpRxChanged = true;
    acAmpMessageToProcess = true;
    menuStartValue = false;
    menuIndex = 1;
  }

  switch (menuIndex) // Also set subMenu and index options
  {
    case 1 : populateFixedLcdDisplay("Temp C or F"); if (shortPushButton == 1) acAmpCentigrade = (!acAmpCentigrade) ? true : false; iconArray[0][6] = (acAmpCentigrade) ? iconArray[0][6] += 0x10 : iconArray[0][6] += 0x08;
      break;
    case 2 : populateFixedLcdDisplay("24/12 Hour"); if (clock24Hour) {
        sevenSegmentCalculate(0x01, 2);
        sevenSegmentCalculate(0x04, 4);
      } else {
        sevenSegmentCalculate(0x01, 1);
        sevenSegmentCalculate(0x04, 2);
      } if (shortPushButton == 1) clock24Hour = (!clock24Hour) ? true : false;
      break;
    case 3 : menuIndexMax = 3; updateClockTime();
      break;
  }

  longPushButton = 0;
  shortPushButton = 0;
}


/*****************************************************************
  // LCD Display
*****************************************************************/

void populateFixedLcdDisplay(String _lcdInput)
{
  _lcdInput.remove(12);

  if (_lcdInput.length() < 12)
  {
    for (byte i = _lcdInput.length() + 1; i <= 12; i++)
    {
      _lcdInput.concat(' ');
    }
  }

  byte _start = 0x80;

  PORTB &= ~(1 << PB0);
  SPI.transfer(0x80);
  PORTB |= (1 << PB0);

  for (byte i = 0; i < _lcdInput.length(); i++)
  {
    PORTL |= (1 << PL0);
    PORTB &= ~(1 << PB0);
    SPI.transfer(_lcdInput.charAt(i));
    PORTB |= (1 << PB0);
    delayMicroseconds(42);
    PORTL &= ~(1 << PL0);
  }
}



void lcdDisplaySetIconDisplay(byte _iconAddress, byte _iconValue)
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(_iconAddress);
  PORTB |= (1 << PB0);
  PORTL |= (1 << PL0);
  PORTB &= ~(1 << PB0);
  SPI.transfer(_iconValue);
  PORTB |= (1 << PB0);
  delayMicroseconds(42);
  PORTL &= ~(1 << PL0);
}



void lcdDisplayPopulateCgramAddress(byte _CgramAddress) // Set the CGRam Address
{
  PORTL |= (1 << PL1);
  PORTB &= ~(1 << PB0);
  SPI.transfer(_CgramAddress);
  PORTB |= (1 << PB0);
  PORTL &= ~(1 << PL1);
  unsigned long time01 = millis();
  for (byte i = 0; i < 7; i++) //populate from the Array
  {
    PORTL |= (1 << PL0);
    PORTB &= ~(1 << PB0);
    SPI.transfer(sevenSegmentArray[0][i]);
    PORTB |= (1 << PB0);
    delayMicroseconds(42);
    PORTL &= ~(1 << PL0);
    sevenSegmentArray[0][i] = 0;
  }
  lcdDisplayCgramToDdram(0x00, 0x8C);   // Populate into DDRam address
  lcdDisplayCgramToDdram(0x00, 0x8D);   // Populate into DDRam address
}



void  lcdDisplayCgramToDdram(byte _CgramSource, byte _DdramDestination) // Copies a CGRam char into the DDram field
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(_DdramDestination);
  PORTB |= (1 << PB0);
  PORTL |= (1 << PL0);
  PORTB &= ~(1 << PB0);
  SPI.transfer(_CgramSource);
  PORTB |= (1 << PB0);
  delayMicroseconds(42);
  PORTL &= ~(1 << PL0);
}



void lcdDisplayStartUp()
{

  PORTL &= ~(1 << PL1);
  PORTL &= ~(1 << PL0);

  //display off
  PORTL |= (1 << PL0);
  SPI.transfer(0x08);
  PORTL &= ~(1 << PL0);
  PORTB |= (1 << PB0);

  clearLcdDisplay();
  returnHomeLcdDisplay();
  flashingCursorLcdDisplay();
  setStaticPortLcdDisplay();
  entryModeIncrementLcdDisplay();
  setContrastLcdDisplay();
  IncrementLcdDisplay();
  resetIconDisplay();
  resetLcdDisplay();
}



void clearLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x1);
  PORTB |= (1 << PB0);
}



void returnHomeLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x02);
  PORTB |= (1 << PB0);
}



void flashingCursorLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x0C);
  PORTB |= (1 << PB0);
}



void setStaticPortLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x00);
  PORTB |= (1 << PB0);
}



void setContrastLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x46);
  PORTB |= (1 << PB0);
}



void entryModeIncrementLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x06);
  PORTB |= (1 << PB0);
}



void IncrementLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x14);
  PORTB |= (1 << PB0);
}



void resetIconDisplay()
{
  for (byte i = 0x90; i < 0x9F; i++)
  {
    PORTB &= ~(1 << PB0);
    SPI.transfer(i);
    PORTB |= (1 << PB0);
    PORTL |= (1 << PL0);
    PORTB &= ~(1 << PB0);
    SPI.transfer(0x00);
    PORTB |= (1 << PB0);
    delayMicroseconds(42);
    PORTL &= ~(1 << PL0);
  }
}



void resetLcdDisplay()
{
  PORTB &= ~(1 << PB0);
  SPI.transfer(0x80);
  PORTB |= (1 << PB0);

  for (byte i = 0; i < 12; i++)
  {
    PORTL |= (1 << PL0);
    PORTB &= ~(1 << PB0);
    SPI.transfer(0x20);
    PORTB |= (1 << PB0);
    delayMicroseconds(42);
    PORTL &= ~(1 << PL0);
  }
}





