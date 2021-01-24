/**
 * FAR&FURTHER Trip Master
 * Version: 0.1
 * 
 * TODO
 *   - Use better display library implementation
 *     - https://github.com/hedgehogV/HT1621-lcd/blob/master/src/HT1621.cpp
 * 
 *   - Use PROGMEM for strings and other stuff
 *     - http://www.gammon.com.au/progmem
 * 
 *   - Menu needs to be implemented
 *     - When we start we will show Menu for 2 seconds
 *     - If user use buttons we enter menu, otherwise continue
 * 
 *   - Add dhop handling
 *     - (https://sites.google.com/site/wayneholder/self-driving-rc-car/getting-the-most-from-gps)
 * 
 *   - Compass has following problems:
 *     - If the module is not level it won't give good reading
 *     - If there is interference we may need to calibrate
 *     - https://www.best-microcontroller-projects.com/hmc5883l.html
 *     - https://www.best-microcontroller-projects.com/magnetometer-tilt-compensation.html
 *     - Alternative HW to what we have:
 *       - https://www.laskarduino.cz/arduino-kompas-qmc5883l/ <<< Better than HMC5883L which we have
 *       - https://www.laskarduino.cz/arduino-9dof-gyroskop-akcelerometr-magnetometr-mpu-9250-spi-iic/
 * 
 * Libraries
 *  HT1621 - https://github.com/pk/ht1621-7-seg
 *         - My fork until merged into the master
 * 
 *  PushButton - https://github.com/r89m/PushButton
 *             - Needs dependencies Button, Bounce2 (v 2.53)
 * 
 *  SdFat - https://github.com/greiman/SdFat
 * 
 *  NeoGPS - https://github.com/SlashDevin/NeoGPS
 * 
 *  NeoSWSerial - https://github.com/SlashDevin/NeoSWSerial
 * 
 *  TinyGPS++ - https://github.com/mikalhart/TinyGPSPlus
 * 
 * Compile & Deploy
 *  arduino-cli compile -b arduino:avr:nano -p /dev/cu.usbserial-A947ST0C --upload
 * 
 * Hardware
 *   https://www.laskarduino.cz/6-mistny-sedmisegmentovy-lcd-displej-2-4--ht1621--bily/
 *   https://www.laskarduino.cz/arduino-nano-r3--atmega328p-klon--pripajene-piny/
 *   https://www.laskarduino.cz/arduino-gps-modul-gy-neo6mv2/
 *   https://www.laskarduino.cz/microsd-card-modul-spi/
 *   https://www.laskarduino.cz/mikro-step-down-menic--nastavitelny/
 * 
 *   Final board: STM32 ARM for 32-bit precission
 *   https://www.laskarduino.cz/bluepill-arm-stm32-stm32f103c8-vyvojova-deska/
 */

// Configuration
#define DEBUG
//#define USE_WHEEL_SENSOR
//#define USE_COMPASS

// Buttons
#include <Button.h>
#include <Bounce2.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#define BUTTON_HOLD_DELAY 800
#define BUTTON_HOLD_REPEAT 30
PushButton buttonUp = PushButton(7, ENABLE_INTERNAL_PULLUP);
PushButton buttonDown = PushButton(8, ENABLE_INTERNAL_PULLUP);

// LCD display
#include <HT1621.h>
#define LCD_CS_PIN 14
#define LCD_WR_PIN 15
#define LCD_DATA_PIN 16
#define LCD_LED_PIN 9
HT1621 lcd;

// Compass sensor
#ifdef USE_COMPASS
  #include <Adafruit_HMC5883_U.h>
  #include <Adafruit_Sensor.h>
  #include <Wire.h>
  Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
#endif

// Speed
#ifdef USE_WHEEL_SENSOR
#define WHEEL_PIN 2
volatile byte wheelRevs = 0;
#endif

// GPS Unit
const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off (Has HDOP we may not need to process it)
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

  // Baud rate
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E // 115200
};
#define gpsPort Serial1
#include <TinyGPS++.h>
TinyGPSPlus gps;

// SDCard
#define SD_CS_PIN 10
#define SD_GPS_FILE (char*)"gps.txt"
#include <SdFat.h>
SdFat SD;

// UI
#define UI_REFRESH_INTERVAL    500
#define UI_REFRESH_INTERVAL_2X 1000 
#define UI_REFRESH_INTERVAL_3X 1500
#define MAX_TRIP_VALUE 99999.0
#define MIN_TRIP_VALUE 0.0
unsigned long uiMillis = 0;

struct Coordinate {
  double lat;
  double lng;
};

enum Screen {
  SCREEN_TOTAL,
  SCREEN_PARTIAL,
  SCREEN_HEADING,
  SCREEN_SPEED,
  SCREEN_MENU,
};

struct State {
  byte activeScreen = SCREEN_TOTAL;

  // How many decimal points we want to display
  // (this influences increment 10 or 100 m when adjusting distance)
  byte decimals = 2;

  #ifdef USE_COMPASS
  float compassDeclinationAngle = 0.0698132;
  word compassHeading = 0;
  #endif

  #ifdef USE_WHEEL_SENSOR
           double wheelCircumference = 2.165186;
             word wheelSpeed = 0;
           double wheelPartial = 0;
           double wheelPreviousTotal = 0;
           double wheelTotal = 0;
  unsigned long wheelPreviousMillisSpeed = 0;
  #endif

  Coordinate gpsLastLocation = {};

  byte gpsPrecision = 0;

  // CAP heading in DEG 0-360
  word gpsHeading = 0;

  // Our current speed
  word gpsSpeed = 0;

  // Total distance (eg day/stage)
  double tripTotal = 0;

  // Partial distance (point - to point)
  double tripPartial = 0;
} state;

//
// Arduino
//

void setup(void)  {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  buttonUp.onPress(onButtonPressed);
  buttonUp.onRelease(150, onButtonReleased);
  buttonUp.onHoldRepeat(BUTTON_HOLD_DELAY, BUTTON_HOLD_REPEAT, onButtonHeld);

  buttonDown.onPress(onButtonPressed);
  buttonDown.onRelease(150, onButtonReleased);
  buttonDown.onHoldRepeat(BUTTON_HOLD_DELAY, BUTTON_HOLD_REPEAT, onButtonHeld);

  lcd.begin(LCD_CS_PIN, LCD_WR_PIN, LCD_DATA_PIN, LCD_LED_PIN);
  lcd.clear();
  lcd.setBatteryLevel(0);
  //lcd.backlight();
  // This controlls PWM for adjustable backlight
  analogWrite(LCD_LED_PIN, 256);
  
  if (!SD.begin(SD_CS_PIN) || !sdPrepare(SD_GPS_FILE)) {
    lcd.print((char*)"Sd  E1");
    while(true);
  }

  #ifdef USE_WHEEL_SENSOR
  pinMode(WHEEL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelRevsCounter, FALLING);
  #endif

  #ifdef USE_COMPASS
  if (!compass.begin()) {
    lcd.print((char*)"CPS Er");
    while(true);
  }
  #endif

  gpsSetup();
}

void loop(void) {
  buttonUp.update();
  buttonDown.update();

  #ifdef USE_WHEEL_SENSOR
  wheelSensorUpdate();
  wheelCalculateSpeed();
  #endif

  #ifdef USE_COMPASS
  state.compassHeading = calculateCompassHeading(compass, state.compassDeclinationAngle);
  #endif

  gpsUpdate(state);
  if (millis() > 10000 && gps.charsProcessed() < 10) {
    lcd.print((char*)"GPS E1");
    while(true);
  }

  // Update data and display every display interval
  unsigned long currentMillis = millis();
  if (currentMillis - uiMillis >= UI_REFRESH_INTERVAL) {
    uiMillis = currentMillis;
    screenUpdate(state);
    screenUpdateBatteryIndicator(state);
  }
}

//
// Utilities
//

unsigned long calculateTripValue(double current,
                                 byte increment,
                                 bool isIncrement,
                                 bool reset) {
  if (reset) return 0;
  long value = current + (isIncrement ? 1.0 : -1.0) * increment;
  if (value <= MIN_TRIP_VALUE) { return MIN_TRIP_VALUE; }
  if (value >= MAX_TRIP_VALUE) { return MAX_TRIP_VALUE; }
  return value;
}

//
// UI
//

void screenUpdate(State& state) {
  char lcdStr[7];
  switch (state.activeScreen) {
  case SCREEN_TOTAL:
    lcd.print(state.tripTotal / 1000.0, 1);
    break;
  case SCREEN_PARTIAL:
    lcd.print(state.tripPartial / 1000.0, state.decimals);
    break;
  case SCREEN_HEADING:
    snprintf(lcdStr, sizeof(lcdStr), "%u*", state.gpsHeading);
    lcd.print(lcdStr, true);
    break;
  case SCREEN_SPEED:
    lcd.print((long)state.gpsSpeed, (char*)"%6li", 0);
    break;
  case SCREEN_MENU:
    lcd.print((char*)"Menu");
    break;
  }
}

void onButtonReleased(Button& btn, uint16_t duration) {
  if (btn.is(buttonUp) && !buttonDown.isPressed()) { return; }
  if (btn.is(buttonDown) && !buttonUp.isPressed()) { return; }
  if (duration > BUTTON_HOLD_DELAY - 100) { return; }

  state.activeScreen == SCREEN_SPEED
    ? state.activeScreen = SCREEN_TOTAL
    : state.activeScreen += 1;
}

void onButtonPressed(Button& btn) {
  switch (state.activeScreen) {
  case SCREEN_TOTAL:
    state.tripTotal = calculateTripValue(state.tripTotal, 100.0, btn.is(buttonUp), false);
    break;
  case SCREEN_PARTIAL:
    state.tripPartial = calculateTripValue(state.tripPartial, 10.0, btn.is(buttonUp), false);
    break;
  case SCREEN_MENU:
    break;
  default:
    break;
  }
  screenUpdate(state);
}

void onButtonHeld(Button& btn, uint16_t duration, uint8_t repeat_count) {
  bool isReset = buttonUp.isPressed() && buttonDown.isPressed();

  switch (state.activeScreen) {
  case SCREEN_TOTAL:
    state.tripTotal = calculateTripValue(state.tripTotal, 100.0, btn.is(buttonUp), isReset);
    break;
  case SCREEN_PARTIAL:
    state.tripPartial = calculateTripValue(state.tripPartial, 10.0, btn.is(buttonUp), isReset);
    break;
  case SCREEN_MENU:
    break;
  default:
    break;
  }
  screenUpdate(state);
}

/**
 * Display GPS "accuracy" information in Battery level
 * 
 * When there are vere fiew satelites it shows no bars otherwise certain
 * number of bars based on the GPS Satelites at the moment.
 */
void screenUpdateBatteryIndicator(State& state) {
  byte level = 0;
  if (state.gpsPrecision <= 1) {
    level = 0;
  } else if (state.gpsPrecision >= 2 && state.gpsPrecision < 4) {
    level = 1;
  } else if (state.gpsPrecision >= 4 && state.gpsPrecision < 6) {
    level = 2;
  } else {
    level = 3;
  }
  lcd.setBatteryLevel(level);
}

//
// SD Card
//

bool sdPrepare(char *path) {
  SdFile file;
  if(!file.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
    #ifdef DEBUG
    Serial.print("SD: Unable to prepare '"); Serial.print(path); Serial.println("' file!");
    #endif
    return false;
  }
  file.print("SAT,LAT,LNG,GDSTHW,GDSTAPX,GDSTTOT,GCAP,GSPD");
  #ifdef USE_WHEEL_SENSOR
  file.print(",TWDST,WSPD");
  #endif
  file.println("");
  file.close();
  return true;
}

bool sdGPSLogWrite(State& state,
                   TinyGPSPlus& fix,
                   double dHarvesine,
                   double dAprox,
                   double dTotal) {
  SdFile file;
  if(!file.open(SD_GPS_FILE, O_WRONLY | O_AT_END | O_APPEND)) {
    #ifdef DEBUG
    Serial.println("SD: Unable to write to GPS Log!'");
    #endif
    return false;
  }

  char latString[10];
  dtostrf(fix.location.lat(), 9, 6, latString);

  char lngString[10];
  dtostrf(fix.location.lng(), 9, 6, lngString);

  char dHarvesineStr[7];
  dtostrf(dHarvesine, 6, 3, dHarvesineStr);

  char dAproxStr[7];
  dtostrf(dAprox, 6, 3, dAproxStr);

  char line[61];
  snprintf(line,
           sizeof(line),
           "%u,%s,%s,%s,%s,%u,%u,%u",
           (uint8_t)state.gpsPrecision,
           latString,
           lngString,
           dHarvesineStr,
           dAproxStr,
           (unsigned int)(dTotal + 0.5),
           (uint8_t)state.gpsHeading,
           (uint8_t)state.gpsSpeed);
  file.println(line);
  file.close();

  #ifdef DEBUG
  Serial.println(line);
  #endif

  return true;
}

//
// GPS
//

void gpsSetup() {
  gpsPort.begin(9600);

  // Send configuration data in UBX protocol
  for(unsigned long i = 0; i < sizeof(UBLOX_INIT); i++) {
    gpsPort.write(pgm_read_byte(UBLOX_INIT + i));
    // Simulating a 38400baud pace (or less),
    // otherwise commands are not accepted by the device.
    delay(5); 
  }

  gpsPort.end();
  gpsPort.begin(115200); 
}

/**
 * Simple Equirectangular aproximation gives very good results comparing to
 * calculations on the PC where there are no rounding errors. This turns out to
 * be closer to PC Harvesine or Equirectangular.
 */
double equirectangularDistance(double lat1, double long1, double lat2, double long2) {
  double x = radians(long2 - long1) * cos((radians(lat1 + lat2)/2.0));
  double y = radians(lat2 - lat1);
  double z = sqrt(sq(x) + sq(y));
  return z * 6371000.0;
}

void gpsUpdate(State& state) {
  while (gpsPort.available() > 0) {
    if (gps.encode(gpsPort.read())) {
      if (gps.satellites.isValid() && gps.satellites.isUpdated()) {
        state.gpsPrecision = (byte)gps.satellites.value();
      }

      if (gps.course.isValid() && gps.course.isUpdated()) {
        state.gpsHeading = (word)round(gps.course.deg());
      }

      if (gps.speed.isValid() && gps.speed.isUpdated()) {
        word speed = (word)round(gps.speed.kmph());
        if (speed < (word)250) {
          state.gpsSpeed = speed;
        }
      }

      if (gps.location.isValid() && gps.location.isUpdated()) {
        if (state.gpsLastLocation.lat == 0.0) {
          state.gpsLastLocation.lat = gps.location.lat();
          state.gpsLastLocation.lng = gps.location.lng();
        } else {
          // Should we update or not?
          bool update = true;

          // We need to have enough satelites, which is 4 for reasonable fix
          if (update)
            update = state.gpsPrecision >= (byte)4;

          // If the location age is older than 1500ms we most likely don't have
          // gps fix anymore and we need to wait for new fix.
          if (update)
            update = gps.location.age() < 1500UL;

          // When speed is very low we risk having loads of error due to
          // coordinates being too close together
          if (update)
            update = state.gpsSpeed > (word)5 && state.gpsSpeed < (word)300;

          double dHarvesine = TinyGPSPlus::distanceBetween(gps.location.lat(),
                                                           gps.location.lng(),
                                                           state.gpsLastLocation.lat,
                                                           state.gpsLastLocation.lng);
          double dAprox = equirectangularDistance(gps.location.lat(),
                                                  gps.location.lng(),
                                                  state.gpsLastLocation.lat,
                                                  state.gpsLastLocation.lng);

          double distance = dHarvesine;
          // Don't update distance when the distance is not within reasonable
          // difference. GPS has +- 2-3 meter possible error/noise...
          if(update)
            update = distance < 200.0;

          // If we think we should update distance, lets do it
          if (update) {
            state.gpsLastLocation.lat = gps.location.lat();
            state.gpsLastLocation.lng = gps.location.lng();
            state.tripPartial += distance;
            state.tripTotal += distance;
          }
          
          // Log data to SD card
          sdGPSLogWrite(state, gps, dHarvesine, dAprox, state.tripTotal);
        }
      }
    }
  }
}

//
// Speed sensor
//

#ifdef USE_WHEEL_SENSOR
// Interrupt Service Routine (ISR)
void wheelRevsCounter() {
  wheelRevs++;
}

void wheelSensorUpdate() {
  byte revsTmp = wheelRevs;
  if(revsTmp > 0) {
    state.wheelPartial += (state.wheelCircumference * (double)revsTmp);
    state.wheelTotal += (state.wheelCircumference * (double)revsTmp);
    wheelRevs -= revsTmp;  
  }
}

void wheelCalculateSpeed() {
  unsigned long currentMillis = millis();
  if (currentMillis - state.wheelPreviousMillisSpeed >= 1000) {
    state.wheelPreviousMillisSpeed = currentMillis;

    // m/s -> km/h
    float speed = (float)(state.wheelTotal - state.wheelPreviousTotal) * 3.6; 
    // moving average
    speed = (float)state.wheelSpeed + (speed - (float)state.wheelSpeed) / (float)3.0;
    // Round down < 1 or round normal values
    state.wheelSpeed = speed < 1.0 ? 0 : speed + 0.5; // Adding 0.5 is the easiest way to round up (int to float trims)

    state.wheelPreviousTotal = state.wheelTotal;
  }
}
#endif

//
// Compass
//
#ifdef USE_COMPASS
float calculateCompassHeading(Adafruit_HMC5883_Unified &mag, float declinationAngle) {
  // Get a new sensor event
  sensors_event_t event; 
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  return heading * 180/M_PI; 
}
#endif
