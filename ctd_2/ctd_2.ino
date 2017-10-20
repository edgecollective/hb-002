
// conductivity

const int selectPins[3] = {9, 6, 11}; // S0, S1, S2

#define MUXPIN 6

#define FREQUENCY 30000 
#define CONDUCT_INPUT 5 // the input pin to the board  

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

bool isLEDOn = false;


// datalogging

#define VBATPIN A7
#include <RTCZero.h>

RTCZero rtc;
int AlarmTime;


#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>


#define SLEEP_TIME 30 // in seconds

#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV  // Flash chip type.
                                              // If you change this be
                                              // sure to change the fatfs
                                              // object type below to match.

#define FLASH_SS       SS1                    // Flash chip SS pin.
#define FLASH_SPI_PORT SPI1                   // What SPI port is Flash on?

Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);     // Use hardware SPI 

// Alternatively you can define and use non-SPI pins!
//Adafruit_SPIFlash flash(SCK1, MISO1, MOSI1, FLASH_SS);

// Finally create an Adafruit_M0_Express_CircuitPython object which gives
// an SD card-like interface to interacting with files stored in CircuitPython's
// flash filesystem.
Adafruit_M0_Express_CircuitPython pythonfs(flash);

#include <Wire.h>

#include "TSYS01.h"  // https://www.bluerobotics.com/store/electronics/celsius-sensor-r1/
#include "MS5837.h" // http://docs.bluerobotics.com/bar30/

TSYS01 temp_sensor;
MS5837 pressure_sensor;

int mIndex=0;

void setup() {

  analogReadResolution(12);

  // input pin
  pinMode(CONDUCT_INPUT, OUTPUT);
 startTimer(FREQUENCY);

 // Set up the select pins, as outputs
   for (int i=0; i<3; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], LOW);
  }

  
 
  // Initialize serial port 

   Serial.begin(9600);
   
   pinMode(LED_BUILTIN, OUTPUT);

   rtc.begin();
   
   Wire.begin();

  temp_sensor.init();

  pressure_sensor.init();
  pressure_sensor.setModel(MS5837::MS5837_30BA);
  pressure_sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  
  // Initialize flash library and check its chip ID.
  if (!flash.begin(FLASH_TYPE)) {
   // Serial.println("Error, failed to initialize flash chip!");

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);  
  
      while(1);
  
  }
  
  if (!pythonfs.begin()) {
    
 digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);  
  
    while(1);
  }

}

void loop() {

temp_sensor.read();
pressure_sensor.read();

float temp = temp_sensor.temperature(); //  Blue Robotics TSYS01 'Fast response' temp sensor

float pressure = pressure_sensor.pressure();  // Blue Robotics MS5837 'Bar30' pressure sensor

float MS_temp = pressure_sensor.temperature();  // Blue Robotics MS5837 'Bar30'temperature

float depth = pressure_sensor.depth(); // Blue Robotics MS5837 'Bar30' depth estimate 


float measuredvbat = analogRead(VBATPIN);
measuredvbat *= 2;    // we divided by 2, so multiply back
measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
measuredvbat /= 4096; // convert to voltage

// conductivity -------------------

  int numSamples=3;
int conductValue=0;

selectMuxPin(MUXPIN);
for (int i=0;i<numSamples;i++) {
conductValue += analogRead(A0);
 //Serial.println(sensorValue);
 delay(200);  
}
float conductAve=float(conductValue)/numSamples;


  // Create or append to a data.txt file and add a new line
  // to the end of it.  CircuitPython code can later open and
  // see this file too!
  File data = pythonfs.open("data.txt", FILE_WRITE);
  if (data) {
    // Write a new line to the file:
    data.print(mIndex);
    data.print(",");
    data.print(temp,3); //  Blue Robotics TSYS01 'Fast response' temp sensor
    data.print(",");
    data.print(MS_temp,3);  // Blue Robotics MS5837 'Bar30' temperature
    data.print(",");
    data.print(pressure,3);  // Blue Robotics MS5837 'Bar30' pressure sensor
    data.print(",");
    data.print(depth,3); // Blue Robotics MS5837 'Bar30' depth estimate 
    data.print(",");
    data.print(measuredvbat,3);
    data.print(",");
    data.println(conductAve,3); // conductivity sensor

    
    
    data.close();
    // See the other fatfs examples like fatfs_full_usage and fatfs_datalogging
    // for more examples of interacting with files.

  


//Serial.println("Logged.");
    Serial.print(mIndex);
    Serial.print(",");
    Serial.print(temp,3); //  Blue Robotics TSYS01 'Fast response' temp sensor
    Serial.print(",");
    Serial.print(MS_temp,3);  // Blue Robotics MS5837 'Bar30' temperature
    Serial.print(",");
    Serial.print(pressure,3);  // Blue Robotics MS5837 'Bar30' pressure sensor
    Serial.print(",");
    Serial.print(depth,3); // Blue Robotics MS5837 'Bar30' depth estimate 
    Serial.print(",");
    Serial.print(measuredvbat,3);
    Serial.print(",");
    Serial.println(conductAve,3); // conductivity sensor


  delay(SLEEP_TIME*1000);

  
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);  

  AlarmTime += SLEEP_TIME; // Adds 5 seconds to alarm time
  AlarmTime = AlarmTime % 60; // checks for roll over 60 seconds and corrects
  rtc.setAlarmSeconds(AlarmTime); // Wakes at next alarm time, i.e. every 5 secs


 // sleep code
  /*
  rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only
  rtc.attachInterrupt(alarmMatch); // Attach function to interupt
  rtc.standbyMode();    // Sleep until next alarm match
  */
  
  }
  else {
    
 digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);    
  }

  mIndex++;

  
}


void alarmMatch() // Do something when interrupt called
{
  
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    digitalWrite(CONDUCT_INPUT, isLEDOn);
    isLEDOn = !isLEDOn;
  }
}


void selectMuxPin(byte pin)
{
  if (pin > 7) return; // Exit if pin is out of scope
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}



