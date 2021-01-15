/**
 * FAR&FURTHER Trip Master
 * Version: 0.1
 * 
 * TODO
 *   - Menu needs to be implemented
 *   - Add dhop handling (https://sites.google.com/site/wayneholder/self-driving-rc-car/getting-the-most-from-gps)
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
 *  TinyGPS++ - https://github.com/mikalhart/TinyGPSPlus
 * 
 * Compile & Deploy
 *  arduino-cli compile -b arduino:avr:nano -p /dev/cu.usbserial-A947ST0C --upload
 */
#define DEBUG
#define USE_WHEEL_SENSOR
// #define USE_COMPASS

// Buttons
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>
#define BUTTON_HOLD_DELAY 800
#define BUTTON_HOLD_REPEAT 30
PushButton buttonUp = PushButton(A0, ENABLE_INTERNAL_PULLUP);
PushButton buttonDown = PushButton(A1, ENABLE_INTERNAL_PULLUP);

// LCD display
#include <HT1621.h>
#define LCD_CS_PIN 6
#define LCD_WR_PIN 7
#define LCD_DATA_PIN 8
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
#define GPS_USE_TINYGPS
#ifdef GPS_USE_TINYGPS
  #include <TinyGPS++.h>
  #include <NeoSWSerial.h>
  #define GPS_BAULD_RATE 9600
  #define GPS_RX_PIN 4
  #define GPS_TX_PIN 5
  TinyGPSPlus gps;
  NeoSWSerial gpsPort(GPS_RX_PIN, GPS_RX_PIN);
#else
  #define GPS_USE_NEOGPS
  #include <NMEAGPS.h>
  #include <GPSport.h>
  NMEAGPS gps;
#endif

// SDCard
#define SD_CS_PIN SS
#define SD_GPS_FILE "gps.txt"
#include <SdFat.h>
SdFat SD;

// UI
#define UI_REFRESH_INTERVAL 500
#define MAX_TRIP_VALUE 99999
#define MIN_TRIP_VALUE 0
unsigned long uiMillis = 0;
unsigned long sdCardMillis = 0;

struct Coordinate {
  double lat;
  double lng;
};

enum Screen {
  SCREEN_TOTAL,
  SCREEN_PARTIAL,
  SCREEN_HEADING,
  SCREEN_SPEED,
  SCREEN_MENU
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
           word wheelCircumference = 2045;
           word wheelSpeed = 0;
  unsigned long wheelPartial = 0;
  unsigned long wheelPreviousTotal = 0;
  unsigned long wheelTotal = 0;
  unsigned long wheelPreviousMillisSpeed = 0;
  #endif

  bool useGPS = true;
  Coordinate gpsLastLocation = {};
  byte gpsPrecision = 0;
  // CAP heading in DEG 0-360
  word gpsHeading = 0;
  // Our current speed
  word gpsSpeed = 0;

  // Total distance (eg day/stage)
  unsigned long tripTotal = 0;

  // Partial distance (point - to point)
  unsigned long tripPartial = 0;
} state;

//
// Arduino
//

void setup(void)  {
  buttonUp.onPress(onButtonPressed);
  buttonUp.onRelease(150, onButtonReleased);
  buttonUp.onHoldRepeat(BUTTON_HOLD_DELAY, BUTTON_HOLD_REPEAT, onButtonHeld);

  buttonDown.onPress(onButtonPressed);
  buttonDown.onRelease(150, onButtonReleased);
  buttonDown.onHoldRepeat(BUTTON_HOLD_DELAY, BUTTON_HOLD_REPEAT, onButtonHeld);

  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  lcd.begin(LCD_CS_PIN, LCD_WR_PIN, LCD_DATA_PIN);
  lcd.clear();
  lcd.setBatteryLevel(0);

  if (!SD.begin(SD_CS_PIN) || !sdPrepare(SD_GPS_FILE)) {
    lcd.print("Sd Err");
    while(true);
  }
  lcd.print("Sd  On");
  delay(500);

  #ifdef USE_WHEEL_SENSOR
  pinMode(WHEEL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelRevsCounter, FALLING);
  lcd.print("SPd On");
  delay(500);
  #endif

  #ifdef USE_COMPASS
  // compass.begin();
  // delay(500);
  #endif

  gpsPort.begin(GPS_BAULD_RATE);
  lcd.print("GPS On");
  delay(500);

  lcd.print("-ridE-");
  delay(1500);
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

  gpsUpdate(state, 4.0);

  // Update data and display every display interval
  unsigned long currentMillis = millis();
  if (currentMillis - uiMillis >= UI_REFRESH_INTERVAL) {
    uiMillis = currentMillis;
    screenUpdate(state);
    screenUpdateBatteryIndicator(state);
  }

  /*
  if (currentMillis - sdCardMillis >= 3 * UI_REFRESH_INTERVAL) {
    sdCardMillis = currentMillis;
    //sdWrite(dataString);
  }
  */

}

//
// Utilities
//

unsigned long calculateTripValue(unsigned long current,
                                 byte increment,
                                 bool isIncrement,
                                 bool reset) {
  if (reset) return 0;
  long value = current + (isIncrement ? 1 : -1) * increment;
  if (value <= (long)MIN_TRIP_VALUE) { return MIN_TRIP_VALUE; }
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
    #ifdef USE_WHEEL_SENSOR
    lcd.print(state.wheelPartial / 1000.0, state.decimals);
    #else
    lcd.print(state.tripPartial / 1000.0, state.decimals);
    #endif
    break;
  case SCREEN_HEADING:
    snprintf(lcdStr, sizeof(lcdStr), "%u*", state.gpsHeading);
    lcd.print(lcdStr, true);
    break;
  case SCREEN_SPEED:
    #ifdef USE_WHEEL_SENSOR
    lcd.print((long)state.wheelSpeed);
    #else
    lcd.print((long)state.gpsSpeed);
    #endif
    break;
  case SCREEN_MENU:
    lcd.print("MENU");
    break;
  }
}

void onButtonReleased(Button& btn, uint16_t duration) {
  if (btn.is(buttonUp) && !buttonDown.isPressed()) { return; }
  if (btn.is(buttonDown) && !buttonUp.isPressed()) { return; }
  if (duration > BUTTON_HOLD_DELAY - 100) { return; }

  state.activeScreen == SCREEN_MENU
    ? state.activeScreen = SCREEN_TOTAL
    : state.activeScreen += 1;
}

void onButtonPressed(Button& btn) {
  switch (state.activeScreen) {
  case SCREEN_TOTAL:
    state.tripTotal = calculateTripValue(state.tripTotal, 100, btn.is(buttonUp), false);
    break;
  case SCREEN_PARTIAL:
    state.tripPartial = calculateTripValue(state.tripPartial, 10, btn.is(buttonUp), false);
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
    state.tripTotal = calculateTripValue(state.tripTotal, 100, btn.is(buttonUp), isReset);
    break;
  case SCREEN_PARTIAL:
    state.tripPartial = calculateTripValue(state.tripPartial, 10, btn.is(buttonUp), isReset);
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
 * This method should be run at UI_REFRESH_INTERVAL frequency.
 * It blinks battery indicator if there is no GPS and it shows certain number
 * of bars based on the GPS Satelites at the moment.
 */
void screenUpdateBatteryIndicator(State& state) {
  byte level = 0;
  if (state.gpsPrecision == UINT8_MAX) {
    state.gpsPrecision = 0;
    level = 0;
  } else if (state.gpsPrecision == 0) {
    state.gpsPrecision = UINT8_MAX;
    level = 3;
  } else {
    if (state.gpsPrecision < 3) {
      level = 1;
    } else if (state.gpsPrecision >= 3 && state.gpsPrecision <= 5) {
      level = 2;
    } else {
      level = 3;
    }
  }
  lcd.setBatteryLevel(level);
}

//
// SD Card
//

bool sdPrepare(char *path) {
  File file = SD.open(path, O_RDWR | O_CREAT | O_TRUNC);
  if (!file) { return false; }
  file.println("UPD,SAT,-,LAT,LON,DIST,CAP,SPD,-,LATERR,LONERR");
  file.close();
  return true;
}

bool sdGPSLogWrite(State& state, TinyGPSPlus& fix, bool update, unsigned long distance) {
  File file = SD.open(SD_GPS_FILE, O_RDWR);
  if (!file) { return false; }

  char latString[11];
  dtostrf(fix.location.lat(), 10, 7, latString);

  char lonString[11];
  dtostrf(fix.location.lng(), 10, 7, lonString);

  char line[50];
  snprintf(line,
           sizeof(line),
           "%s,%u,%s,%s,%u,%u,%u,%u,%u",
           update ? "Y" : "N",
           state.gpsPrecision,
           latString,
           lonString,
           distance,
           state.gpsHeading,
           state.gpsSpeed,
           0,
           0
           );
  file.println(line);
  file.close();

  #ifdef DEBUG
  Serial.println(line);
  #endif
}

//
// GPS
//
#ifdef GPS_USE_TINYGPS
void gpsUpdate(State& state, double minDistanceTreshold) {
  while (gpsPort.available() > 0) {
    if (gps.encode(gpsPort.read())) {
      if (gps.satellites.isValid() && gps.satellites.isUpdated()) {
        state.gpsPrecision = (byte)gps.satellites.value();
      }
      if (gps.course.isValid() && gps.course.isUpdated()) {
        state.gpsHeading = (word)round(gps.course.deg());
      }
      if (gps.speed.isValid() && gps.speed.isUpdated()) {
        state.gpsSpeed = (word)round(gps.speed.kmph());
      }

      // This needs to be verified for accuracy, so far I'm getting about 5%
      // under reading comparing to the 2 different apps on the iPhone.
      // I think some more filering or cleverness is in order.
      //
      // Also as Arduino is only 8-bit arithmetic... we may have error at low
      // speed, but that would not explain the under reading.
      //
      // Possibility would be to switch to NeoGPS as it seems to be more
      // accurate than TinyGPSPlus
      // https://github.com/SlashDevin/NeoGPS
      //
      // Another distance calculation may come from here, that avoids loads of
      // trigonometry but need to verify the accuracy.
      // https://www.instructables.com/Distance-measuring-and-more-device-using-Arduino-a/
      //
      if (gps.location.isValid() && gps.location.isUpdated()
          && gps.date.isValid() && gps.time.isValid()) {
        if (state.gpsLastLocation.lat == 0.0) {
          state.gpsLastLocation = { gps.location.lat(), gps.location.lng() };
        } else {
          // Should we update or not?
          bool update = true;

          // If the location age is older than 1500ms we most likely don't have
          // gps fix anymore and we need to wait for new fix.
          if (update)
            update = gps.location.age() < 1500;

          // When speed is very low we risk having loads of error due to
          // coordinates being too close together
          if (update)
            update = state.gpsSpeed > 5 && state.gpsSpeed < 300;

          unsigned long distance =
            (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),
                                                        gps.location.lng(),
                                                        state.gpsLastLocation.lat,
                                                        state.gpsLastLocation.lng);

          // Don't update distance when the distance is not within reasonable
          // difference. GPS has +- 2-3 meter possible error/noise...
          if(update)
            update = distance > 4 && distance < 200;

          // If we think we should update distance, lets do it
          if (update) {
            state.gpsLastLocation = { gps.location.lat(), gps.location.lng() };
            state.tripPartial += distance;
            state.tripTotal += distance;
          }
          
          // Log data to SD card
          sdGPSLogWrite(state, gps, update, distance);
        }
      }
    }
  }
}
#endif

#ifdef GPS_USE_NEOGPS
void gpsUpdate(State& state, double minDistanceTreshold) {
   while (gps.available(gpsPort)) {
    gps_fix fix = gps.read();

    if (fix.valid.satellites) {
      state.gpsPrecision = fix.satellites;
    }
    if (fix.valid.speed) {
      // Too slow, zero out the speed
      if(fix.speed_mkn() < 1000) {
        fix.spd.whole = 0;
        fix.spd.frac  = 0;
      }
      state.gpsSpeed = round(fix.speed_kph());
    }
    if (fix.valid.heading) {
      state.gpsHeading = fix.heading_cd();
    }
    if (fix.valid.location) {
      if (state.gpsLastLocation.lat() == 0 && state.gpsLastLocation.lon() == 0) {
        state.gpsLastLocation = fix.location;
      } else {
        float distanceKM = fix.location.DistanceKm(state.gpsLastLocation);
        unsigned long distanceM = round(distanceKM * (float)1000.0);

        // Should we update or not?
        bool update = true;

        // If the location age is older than 1500ms we most likely don't have
        // gps fix anymore and we need to wait for new fix.
        // if (update)
        //   update = fix.location.age < (unsigned long)1500;

        // When speed is very low we risk having loads of error due to
        // coordinates being too close together
        if (update)
          update = state.gpsSpeed > (byte)5;

        // Don't update distance when the distance is not within reasonable
        // difference. GPS has +- 2-3 meter possible error/noise...
        if(update)
          update = distanceM > 5 && distanceM < 200;

        // If we think we should update distance, lets do it
        if (update) {
          state.gpsLastLocation = fix.location;
          state.tripLifetime += distanceM;
          state.tripPartial += distanceM;
          state.tripTotal += distanceM;
        }

        // Log data to SD card
        sdGPSLogWrite(state, fix, update, distanceM);
      }
    }
  }
}
#endif

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
    state.wheelPartial += ((state.wheelCircumference / 1000.0) * revsTmp) + 0.5;
    state.wheelTotal += ((state.wheelCircumference  / 1000.0) * revsTmp) + 0.5;
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