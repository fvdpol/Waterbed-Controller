// Waterbed controller
// ===================
//
// Frank van de Pol, 6 November 2018
//
// running on JeeNode with esp-bridge (wemos d1 mini hardware)
//
// ESP Link wiring:
// todo
//
// can be flashed over wifi by selecting board: "Arduino Uno Wifi board"
//
//
// program memory usage:
//    storage: 20388 (63%) > 21370 (66%)
//    dynamic:   932 (45%) >   918 (44%)
//
#include <avr/wdt.h>
#include <EEPROM.h>

#include <JeeLib.h>
#include <OneWire.h>
#include <SerialCommand.h>
#include <PID_v1.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header


#define TRANSMIT_INTERVAL 150  /* in 0.1 s */


// functionality
//
// - watchdog reset
// - serial command interface
// - sensor value validation
// - sensor value averaging & smoothing
// - PID controller
// - Power & Energy counter
// - Energy counter reset
//
//
// todo:
//
// uptime is simply reporting the millies; this will will wrap after just 50 days; make something better....
// configurable output range (clamping)
// configurable reporting frequency (default: 15s)
// add user interface
//  - i2c display
//  - push buttons [ - ] [menu] [ + ]
// configurable energy settings (idle/100%)
// change jeenode port io to regular arduino library io
// add fail-safe (series) relay
// add bypass relay (bypass ssr to reduce dissipation lossses)
//
// retain settings in eeprom
//  - tuning parameters
//  - output range
//  - setpoint
//  - reporting freq
//  - sensor configuration
// store settings if not changed for 5..10m (reduce wear); use arduino update/put() functions
//
// refactor temperature sensor handling
//    -- dynamic sensors
//    -- scan addresses
//    -- assigned scan result to slot
//    -- select sensor(s) for PID process value
//    -- max 8 sensors (3 in bed/matress bottom; bedroom; bed/matress top
//    -- short label/desc
//    -- valid range
//    -- smoothing/filtering
//
// error detection/display -- maybe beeper to alert for new errors?
//

// Arduino PID Control:
//
// http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-sample-time/
// http://playground.arduino.cc/Code/PIDLibrary
//
// http://fermentationriot.com/arduinopid.php


// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library



OneWire  ds(4);        // Jeenode Port 1 Digital, so arduino pin 4
//Port     relay(1);     // Jeenode Port 1 -- the solid state relay is connected the Analog port
Port     relay(2);     // Jeenode Port 2 -- the solid state relay is connected the Analog port




// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;



hd44780_I2Cexp lcd(0x20); // declare lcd object: auto locate & auto config expander chip



const float idlePower = 2.0;    // power consumption when idle (ie. from supporting electronics)
const float fullPower = 360.0;  // power consumption of the heater if 100% on


//
// sensors:
// remote (long lead) -- water temperature
// ROM = 28 18 68 6C 1 0 0 E2
// Chip = DS18B20
//
// local (short lead) -- room temperature
// ROM = 28 2C 79 82 3 0 0 71
// Chip = DS18B20

// Waterbed cable:

// YELLOW
//ROM = 28 12 D1 71 4 0 0 1
//  Chip = DS18B20
//
// GREEN:
//ROM = 28 E E0 71 4 0 0 6E
//  Chip = DS18B20
//
// RED:
//ROM = 28 7E E6 71 4 0 0 6A
//  Chip = DS18B20



// potential other measurements:
//  - room temperature
//  - temperature of SCR heatsink
//  - temperature of MCU/CPU
//  - temperature top of bed


//byte addr_bed[]  = {0x28, 0x18, 0x68, 0x6C, 0x01, 0x00, 0x00, 0xE2};  // waterbed sensor
//byte addr_room[] = {0x28, 0x2C, 0x79, 0x82, 0x03, 0x00, 0x00, 0x71};  // room sensor


byte addr_s1[] = {0x28, 0x12, 0xD1, 0x71, 0x04, 0x00, 0x00, 0x01};     // waterbed sensor Y - Frank #1
byte addr_s2[] = {0x28, 0x0E, 0xE0, 0x71, 0x04, 0x00, 0x00, 0x6E};     // waterbed sensor G - Frank #2
byte addr_s3[] = {0x28, 0x7E, 0xE6, 0x71, 0x04, 0x00, 0x00, 0x6A};     // waterbed sensor R - Sveta


// defined limits for possible sensor values (used for validity check)
const float sensor_under = 10.0;
const float sensor_over  = 40.0;


float output_range_min = 0.0;
float output_range_max = 100.0;


float temp_s1 = -9999.0;
float temp_s2 = -9999.0;
float temp_s3 = -9999.0;

float temp_s1_raw = 0.0;
float temp_s2_raw = 0.0;
float temp_s3_raw = 0.0;


// correction added to the raw sensor value (calibration)
const float correction_s1 = 0.0;
const float correction_s2 = 0.0;
const float correction_s3 = +0.20;


enum {
  TASK_HEARTBEAT,
  TASK_MEASURE,
  TASK_PROCESS,
  TASK_TRANSMIT,
  TASK_SAVECONFIG,
  TASK_LIMIT
};

static word schedBuf[TASK_LIMIT];
Scheduler scheduler (schedBuf, TASK_LIMIT);


#define CONFIG_START 32   // offset where config is stored in EEPROM

char cfg_app[4]    = {'W', 'T', 'R', 'B' };
int  cfg_version = 1;

struct Config {
  // header
  char app[4];    // identifier for this application
  int  version;   // version number for the configuration structure, update when struct changes

  // content
  float setpoint;
  float kp;
  float ki;
  float kd;
};




SerialCommand sCmd;     // The SerialCommand object

//Define Variables for connecting PID controller
double PID_Setpoint, PID_Input, PID_Output;

//Specify the links and initial tuning parameters
PID myPID(&PID_Input, &PID_Output, &PID_Setpoint,
          2, 5, 1,
          P_ON_M, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior



int PWM_WindowSize = 5000;    // PWM cycle in ms
unsigned long PWM_WindowStartTime;



unsigned long energyWhCount = 0;    // energy counter in WattHours
float energyWhFraction = 0.0;       // keep track of energy, once we have 'full' Wh, transfer it to the energyWhCount



void loadConfig(void)
{
  Config myConfig;
  EEPROM.get(CONFIG_START, myConfig);

  // validate header before accepting values
  if (memcmp(myConfig.app, cfg_app, sizeof(cfg_app)) != 0) {
    Serial.println(F("EEPROM invalid app id"));
    return;
  }

  if (myConfig.version != cfg_version) {
    Serial.println(F("EEPROM invalid version"));
    return;
  }

  // header checks passed successfully, we should be able to trust the values stored
  // for additional security ensure values are within sensible range
  if (isTemperatureInRange(myConfig.setpoint))
    PID_Setpoint = myConfig.setpoint;

  if (isKpInRange(myConfig.kp) && isKiInRange(myConfig.ki) && isKdInRange(myConfig.kd))
    myPID.SetTunings(myConfig.kp, myConfig.ki, myConfig.kd, P_ON_M);


  Serial.println(F("Config loaded from EEPROM"));
}

void saveConfig(void)
{
  Config myConfig;

  // init header
  memcpy(myConfig.app, cfg_app, sizeof(cfg_app));
  myConfig.version = cfg_version;

  // write content
  myConfig.setpoint = PID_Setpoint;
  myConfig.kp = myPID.GetKp();
  myConfig.ki = myPID.GetKi();
  myConfig.kp = myPID.GetKd();

  EEPROM.put(CONFIG_START, myConfig);
  Serial.println(F("Configuration stored in EEPROM"));
}


// to be called when configuration has been updated
// delay expressed in s
// default delay is 5 minutes before commiting change to eeprom to redyce wear
void configUpdated(int delay = 300)
{
  scheduler.timer(TASK_SAVECONFIG, delay * 10);
}


int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


// code to display the sketch name/compile date
// http://forum.arduino.cc/index.php?PHPSESSID=82046rhab9h76mt6q7p8fle0u4&topic=118605.msg894017#msg894017

int pgm_lastIndexOf(uint8_t c, const char * p)
{
  int last_index = -1; // -1 indicates no match
  uint8_t b;
  for (int i = 0; true; i++) {
    b = pgm_read_byte(p++);
    if (b == c)
      last_index = i;
    else if (b == 0) break;
  }
  return last_index;
}

// displays at startup the Sketch running in the Arduino
void display_srcfile_details(void) {
  const char *the_path = PSTR(__FILE__);           // save RAM, use flash to hold __FILE__ instead

  int slash_loc = pgm_lastIndexOf('/', the_path); // index of last '/'
  if (slash_loc < 0) slash_loc = pgm_lastIndexOf('\\', the_path); // or last '\' (windows, ugh)

  int dot_loc = pgm_lastIndexOf('.', the_path);  // index of last '.'
  if (dot_loc < 0) dot_loc = pgm_lastIndexOf(0, the_path); // if no dot, return end of string

  Serial.print(F("\n\nSketch: "));
  for (int i = slash_loc + 1; i < dot_loc; i++) {
    uint8_t b = pgm_read_byte(&the_path[i]);
    if (b != 0) Serial.print((char) b);
    else break;
  }

  Serial.print(F(", Compiled on: "));
  Serial.print(F(__DATE__));
  Serial.print(F(" at "));
  Serial.println(F(__TIME__));
}


void showHelp(void)
{
  display_srcfile_details();
  Serial.print(F("Free RAM:   "));
  Serial.println(freeRam());


  Serial.println(F("\n"
                   "Available commands:\n"
                   "\n"
                   " values - show values \n"
                   " auto  - set PID controller in auto mode\n"
                   " manual - set PID controller in manual mode\n"
                   " setpoint [sp] - set setpoint in degrees Celsius\n"
                   " output [out%] - set controller output 0..100%\n"
                   " tuning [Kp] [Ki] [Kd] - set PID controller tuning parameters\n"
                   " energy [Wh] - (re-)initialise the energy counter\n"
                   " outrange [min] [max] - set output range\n"
                   " \n"
                   " hang - test watchdog\n"
                   " scan - scan bus for DX18x20 sensors\n"
                   //      " hello [name]\n"
                   //      " p <arg1> <arg2>\n"
                   //      " d  - toggle debug messages\n"
                   //      " l  - send lifesign message\n"
                   " ?  - show this help\n"
                   "\n"));
}

void showValues(void)
{
  Serial.println(F("\nValues:"));

  Serial.print(F("Sensor 1:      "));
  Serial.println(temp_s1);
  Serial.print(F("Sensor 2:      "));
  Serial.println(temp_s2);
  Serial.print(F("Sensor 3:      "));
  Serial.println(temp_s3);

  Serial.print(F("PID Measured:  "));
  Serial.println(PID_Input);
  Serial.print(F("PID Setpoint:  "));
  Serial.println(PID_Setpoint);
  Serial.print(F("PID Output:    "));
  Serial.println(PID_Output, 1);

  Serial.print(F("PID Kp:        "));
  Serial.println(myPID.GetKp(), 5);
  Serial.print(F("PID Ki:        "));
  Serial.println(myPID.GetKi(), 5);
  Serial.print(F("PID Kd:        "));
  Serial.println(myPID.GetKd(), 5);
  Serial.print(F("PID Mode:      "));
  Serial.println(myPID.GetMode());
  //    Serial.print(F("PID Output Min:"));
  //    Serial.println(myPID.GetXXX());
  //    Serial.print(F("PID Output Max:"));
  //    Serial.println(myPID.GetXXX());

  Serial.print(F("Power:         ")); // W
  Serial.println(idlePower + (fullPower * PID_Output / 100.0), 1);
  Serial.print(F("Energy:        ")); // Wh
  Serial.println(energyWhCount);
  //   Serial.print(F("Energy fract:  ")); // Wh
  //   Serial.println(energyWhFraction);


  // keep track on health of the MCU/Application
  Serial.print(F("RAM:           ")); // bytes
  Serial.println(freeRam());
  Serial.print(F("Uptime:        ")); // seconds
  Serial.println(millis() / 1000);

}


void setup(void) {
  // start hardware watchdog
  wdt_enable(WDTO_8S);
  Serial.begin(57600);

  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.print(F("Hello, World!"));

  showHelp();


  // defined port A as output
  relay.digiWrite2(0);
  relay.mode2(OUTPUT);


  sCmd.addCommand("values", showValues);

  sCmd.addCommand("auto",   setPIDModeAuto);
  sCmd.addCommand("manual", setPIDModeManual);
  sCmd.addCommand("setpoint", setPIDSetpoint);
  sCmd.addCommand("output", setPIDOutput);
  sCmd.addCommand("tuning", setPIDTuning);
  sCmd.addCommand("energy", setEnergy);

  sCmd.addCommand("hang", Hang);
  sCmd.addCommand("scan",  DS1820_ScanBus);

  //  sCmd.addCommand("hello", sayHello);        // Echos the string argument back
  //  sCmd.addCommand("p",     processCommand);  // Converts two arguments to integers and echos them back
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")


  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(output_range_min, output_range_max);
  myPID.SetSampleTime(1000);


  // initialise setpoint and PID parameters from eeprom
  // provide some starting in case no (valid) configuration exists
  output_range_min = 0.0;
  output_range_max = 60.0;

  myPID.SetOutputLimits(output_range_min, output_range_max);   // limit output power of the heater (360W was size for single matrass, so now it is oversized)
  myPID.SetTunings(100, 0.02, 0.0, P_ON_M);
  PID_Setpoint = 27.7;
  PID_Output = 0.0;


  // retrieve configuration from EEPROM
  loadConfig();

  // and when setpoint is known controller can be set to automatic
  // assume we are on setpoint at startup

  PID_Input = PID_Setpoint;
  myPID.SetMode(AUTOMATIC);

  // start measuring values
  scheduler.timer(TASK_MEASURE, 1);

  // start transmission
  scheduler.timer(TASK_TRANSMIT, TRANSMIT_INTERVAL);


  // start heartbeat/watchdog reset
  scheduler.timer(TASK_HEARTBEAT, 1);
}



void loop(void) {
  sCmd.readSerial();     // We don't do much, just process serial commands
  myPID.Compute();



  // modulate the SSR output to achive proportional power output
  //
  // PID_Output range is 0...100%
  //
  if (millis() - PWM_WindowStartTime > PWM_WindowSize) { //time to shift the Relay Window
    PWM_WindowStartTime += PWM_WindowSize;

    // keep track of energy consumption
    //
    // power in W
    // energy in Wh
    //
    // WindowSize in ms -->
    //  1h --> E=P
    //  1s --> E=P/3600
    //  5s --> E=P/(3600/5)
    //  5000ms --> E=P/(3600*1000/5000)
    energyWhFraction = energyWhFraction + ((idlePower + (fullPower * PID_Output / 100.0)) / ((3600.0 * 1000.0) / PWM_WindowSize));

    // move any full watt hours over from the fraction to the main counter
    if (energyWhFraction > 1.0) {
      int Wh = int(energyWhFraction);
      energyWhCount = energyWhCount + Wh;
      energyWhFraction = energyWhFraction - Wh;
    }

  }
  if ((PWM_WindowSize * PID_Output / 100.0) < (millis() - PWM_WindowStartTime)) {
    //digitalWrite(RELAY_PIN, HIGH);
    relay.digiWrite2(0);
  } else {
    //digitalWrite(RELAY_PIN, LOW);
    relay.digiWrite2(1);
  }



  // if output is at _min for some time switch off the series relay; switch on if >_min
  // this also will serve as a safety/redundancy to cover failure mode of the SSR where triac will fail closed
  // TODO... some time is maybe 10m ???


  // if output is at _max for some time switch on the bypass relay; switch off if < _max
  // TODO... some time is maybe 10m ???




  switch (scheduler.poll()) {
    case TASK_HEARTBEAT:
      wdt_reset(); // reset the hardware watchdog timer

      scheduler.timer(TASK_HEARTBEAT, 20);
      break;


    case TASK_TRANSMIT:
      showValues();

      // reschedule every 15s
      scheduler.timer(TASK_TRANSMIT, TRANSMIT_INTERVAL);
      break;


    // cyclic task to measure temperature values
    case TASK_MEASURE:
      scheduler.timer(TASK_MEASURE, 50);  // repeat every 5 seconds

      ds18b20_start(addr_s1);
      ds18b20_start(addr_s2);
      ds18b20_start(addr_s3);
      scheduler.timer(TASK_PROCESS, 10);  // one second after starting measurement the DS18B20 sensors should have the value ready, and we can start processing it
      break;


    // triggered from the measurement task
    case TASK_PROCESS:
      processMeasurents();
      break;



    case TASK_SAVECONFIG:
      saveConfig();
      break;


  }

  delay(10);
}



void processMeasurents(void)
{
  temp_s1_raw = ds18b20_read(addr_s1, 0) + correction_s1;
  if (temp_s1 <= -9999.0 || temp_s1_raw < sensor_under || temp_s1_raw > sensor_over) {
    // initialise value
    temp_s1 = temp_s1_raw;
  } else {
    temp_s1 = (temp_s1 * 0.90) + (temp_s1_raw * 0.10);
  }

  /*
     Serial.print(F("  Temperature S1  = "));
     Serial.print(celsius);
     Serial.print(F(" 'C raw"));
     Serial.print(F("  Temperature S1  = "));
     Serial.print(temp_s1);
     Serial.println(F(" 'C"));
  */


  temp_s2_raw = ds18b20_read(addr_s2, 0) + correction_s2;
  if (temp_s2 <= -9999.0 || temp_s2_raw < sensor_under || temp_s2_raw > sensor_over) {
    // initialise value
    temp_s2 = temp_s2_raw;
  } else {
    temp_s2 = (temp_s2 * 0.90) + (temp_s2_raw * 0.10);
  }

  /*
     Serial.print(F("  Temperature S2  = "));
     Serial.print(celsius);
     Serial.print(F(" 'C raw"));
     Serial.print(F("  Temperature S2  = "));
     Serial.print(temp_s2);
     Serial.println(F(" 'C"));
  */

  temp_s3_raw = ds18b20_read(addr_s3, 0) + correction_s3;
  if (temp_s3 <= -9999.0 || temp_s3_raw < sensor_under || temp_s3_raw > sensor_over) {
    // initialise value
    temp_s3 = temp_s3_raw;
  } else {
    temp_s3 = (temp_s3 * 0.90) + (temp_s3_raw * 0.10);
  }

  /*
      Serial.print(F("  Temperature S3  = "));
      Serial.print(celsius);
      Serial.print(F(" 'C raw"));
      Serial.print(F("  Temperature S3  = "));
      Serial.print(temp_s3);
      Serial.println(F(" 'C"));
  */


  // calcaulte the PID_Input value based on the S1 and S2 sensors;
  // determine if sensor is returning a reasonable value...
  bool s1_valid = isTemperatureInRange(temp_s1);
  bool s2_valid = isTemperatureInRange(temp_s2);

  if (s1_valid && s2_valid) {
    // both sensors valid, use weighted average; favouring the indicator with highest reading
    // since one sensor is closer to the heater element and should show a faster response on
    // changes in output
    // value = 80% highest + 20% lowest sensor
    //
    if (temp_s1 > temp_s2)
      PID_Input = ((4.0 * temp_s1) +      temp_s2 ) / 5.0;
    else
      PID_Input = (     temp_s1  + (4.0 * temp_s2)) / 5.0;
  } else {
    if (s1_valid) {
      PID_Input = temp_s1;
      Serial.println(F("WARNING: Sensor2 not valid, using Sensor 1"));
    } else {
      if (s2_valid) {
        PID_Input = temp_s2;
        Serial.println(F("WARNING: Sensor1 not valid, using Sensor 2"));
      } else {
        // no valid sensor readings.... ERROR, switch off controller
        Serial.println(F("ERROR: No valid sensor readings"));
        myPID.SetMode(MANUAL);
        PID_Input = PID_Setpoint;
        PID_Output = 0.0;
        myPID.SetMode(AUTOMATIC);
      }
    }
  }
}



// start conversion of the dallas ds18b20 chip
void ds18b20_start(uint8_t *addr)
{
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
}



// read temperature (celcius) from the dallas ds18b20 chip
//
// type_s = 0  --> DS18B20 or DS1822
// type_s = 1  --> DS18S20 or old DS1820
float ds18b20_read(uint8_t *addr, byte type_s)
{
  byte data[12];
  byte i;

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  //  Serial.print("  Data = ");
  //  Serial.print(present,HEX);
  //  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //    Serial.print(data[i], HEX);
    //    Serial.print(" ");
  }
  //  Serial.print(" CRC=");
  //  Serial.print(OneWire::crc8(data, 8), HEX);
  //  Serial.println();

  // convert the data to actual temperature
  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }

  // return celcius temperature value
  return (float)raw / 16.0;
}


void DS1820_ScanBus()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;

  while (ds.search(addr)) {

    Serial.print("ROM =");
    for ( i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println(F("CRC is not valid!"));
      return;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        Serial.println(F("  Chip = DS18S20"));  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        Serial.println(F("  Chip = DS18B20"));
        type_s = 0;
        break;
      case 0x22:
        Serial.println(F("  Chip = DS1822"));
        type_s = 0;
        break;
      default:
        Serial.println(F("Device is not a DS18x20 family device."));
        return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    Serial.print(F("  Data = "));
    Serial.print(present, HEX);
    Serial.print(" ");
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.print(F(" CRC="));
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // convert the data to actual temperature

    unsigned int raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // count remain gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
      // default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    Serial.print(F("  Temperature = "));
    Serial.print(celsius);
    Serial.println(F(" 'C"));

  }
  Serial.println(F("No more addresses."));
  ds.reset_search();
  delay(250);

}



void setPIDModeAuto(void)
{
  // use SP=MV option to avoid bump
  PID_Setpoint = PID_Input;
  Serial.print(F("Controller Setpoint set to "));
  Serial.println(PID_Setpoint);

  myPID.SetMode(AUTOMATIC);
  Serial.println(F("Controller Mode set to automatic"));
  // should we use SP=MV option?
  PID_Setpoint = PID_Input;

  // trigger update of reported values
  scheduler.timer(TASK_TRANSMIT, 0);
}


void setPIDModeManual(void)
{
  myPID.SetMode(MANUAL);
  Serial.println(F("Controller Mode set to manual"));

  // trigger update of reported values
  scheduler.timer(TASK_TRANSMIT, 0);
}


// santity check on the temperature value, ensure value is sensible
bool isTemperatureInRange(const float temperature)
{
  return (temperature >= sensor_under && temperature <= sensor_over);
}

void setPIDSetpoint(void)
{
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    float sp = atof(arg);

    // check if in range
    if (isTemperatureInRange(sp)) {
      PID_Setpoint = sp;
      Serial.print(F("Controller Setpoint set to "));
      Serial.println(PID_Setpoint);

      // trigger update of reported values
      scheduler.timer(TASK_TRANSMIT, 0);

      // notify config that values have been updated, for store to EEPROM
      configUpdated();

    } else
      Serial.println(F("setpoint out of range"));
  }
  else {
    Serial.println(F("missing setpoint argument"));
  }
}


void setPIDOutput(void)
{
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    float out = atof(arg);

    // check if in range
    if (out >= 0.0  && out <= 100.0) {
      PID_Output = out;
      Serial.print(F("Controller Output set to "));
      Serial.println(PID_Output);

      // trigger update of reported values
      scheduler.timer(TASK_TRANSMIT, 0);

    } else
      Serial.println(F("output out of range"));
  } else {
    Serial.println(F("missing output argument"));
  }
}


// santity check on parameter value, ensure value is sensible
bool isKpInRange(const float Kp)
{
  return (Kp >= 0.0 && Kp <= 1000.0);
}

// santity check on parameter value, ensure value is sensible
bool isKiInRange(const float Ki)
{
  return (Ki >= 0.0 && Ki <= 1000.0);
}

// santity check on parameter value, ensure value is sensible
bool isKdInRange(const float Kd)
{
  return (Kd >= 0.0 && Kd <= 1000.0);
}


void setPIDTuning(void)
{
  char *arg;
  float kp = -1.0, ki = -1.0, kd = -1.0; ;

  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    kp = atof(arg);

    // check if in range
    if (!isKpInRange(kp)) {
      Serial.println(F("Kp output out of range"));
      return;
    }
  } else {
    Serial.println(F("missing tuning parameters"));
    return;
  }

  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    ki = atof(arg);

    // check if in range
    if (!isKiInRange(ki)) {
      Serial.println(F("Ki output out of range"));
      return;
    }
  } else {
    Serial.println(F("missing tuning parameters"));
    return;
  }

  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    kd = atof(arg);

    // check if in range
    if (!isKdInRange(kd)) {
      Serial.println(F("Kd output out of range"));
      return;
    }
  } else {
    Serial.println(F("missing tuning parameters"));
    return;
  }



  Serial.print(F("Controller Tuning Kp="));
  Serial.print(kp, 5);
  Serial.print(F(", Ki="));
  Serial.print(ki, 5);
  Serial.print(F(", Kd="));
  Serial.println(kd, 5);

  myPID.SetTunings(kp, ki, kd, P_ON_M);

  // notify config that values have been updated, for store to EEPROM
  configUpdated();

}


void setEnergy(void)
{
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    energyWhCount = atol(arg);
    Serial.print(F("Energy Wh Count set to "));
    Serial.println(energyWhCount);

    // trigger update of reported values
    scheduler.timer(TASK_TRANSMIT, 0);

  } else {
    Serial.println(F("missing output argument"));
  }
}


void Hang(void) {
  Serial.println(F("\nHangup... to test watchdog...\n"));
  for (int i = 1; i < 60; i++) {
    Serial.println(i);
    delay(1000);
  }
  Serial.println(F("Sleepy watchdog???"));
}


/*
  void sayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    Serial.print(F("Hello "));
    Serial.println(arg);
  }
  else {
    Serial.println(F("Hello, whoever you are"));
  }
  }


  void processCommand() {
  int aNumber;
  char *arg;

  Serial.println(F("We're in processCommand"));
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);    // Converts a char string to an integer
    Serial.print(F("First argument was: "));
    Serial.println(aNumber);
  }
  else {
    Serial.println(F("No arguments"));
  }

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atol(arg);
    Serial.print(F("Second argument was: "));
    Serial.println(aNumber);
  }
  else {
    Serial.println(F("No second argument"));
  }
  }
*/



// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  showHelp();
}



// LCD Layout (16x2)
//
// 1234567890123456
// xxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxx
//
// SP=xx.x  PV=xx.x
// #########  xxx %     >> bargraph with user defined characters
//
// Change Setpoint
// >> xx.x 'C
//
// **** ERROR ****
// errortext
//
//
