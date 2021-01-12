/**
 * FAR&FURTHER Trip Master
 * Version: 0.1
 * 
 * TODO
 *   - Menu needs to be implemented
 *   - That needs changes to HT1621 to display characters
 *   - SD Card storage
 *   - Distance & Speed from Hall sensor, this just needs some storage & interrupt
 * 
 * Libraries
 *  HT1621 - https://github.com/5N44P/ht1621-7-seg
 *         - Unable to display characters which will be needed for Menu
 *           but it seems it could be easy to add display(char c, int poistion)
 *           reasonably easy
 *  PushButton - https://github.com/r89m/PushButton
 *             - Needs dependencies Button, Bounce2 (v 2.53)
 * 
 * Compile & Deploy
 *  arduino-cli compile -b arduino:avr:nano -p /dev/cu.usbserial-A947ST0C --upload
 */
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <HT1621.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
// Buttons
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>

// Buttons
#define BUTTON_HOLD_DELAY 800
#define BUTTON_HOLD_REPEAT 30
PushButton buttonUp = PushButton(A0, ENABLE_INTERNAL_PULLUP);
PushButton buttonDown = PushButton(A1, ENABLE_INTERNAL_PULLUP);

// LCD display
#define LCD_CS_PIN 6
#define LCD_WR_PIN 7
#define LCD_DATA_PIN 8
HT1621 lcd;

// Magnetometer sensor
// Adafruit_HMC5883_Unified magneto = Adafruit_HMC5883_Unified(12345);

// Speed
#define SPEED_PIN A2

// GPS Unit
#define GPS_RX_PIN 4
#define GPS_TX_PIN 5
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// UI
unsigned long uiMillis = 0;
unsigned long sdCardMillis = 0;
const unsigned long UI_REFRESH_INTERVAL = 500;
const unsigned long MAX_TRIP_VALUE = 99999;
const unsigned long MIN_TRIP_VALUE = 0;

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

  bool useGPS = true;
  Coordinate gpsLastLocation = { 0.0, 0.0 };
  char gpsPrecision = 0;

  float magnetoDeclinationAngle = 0.0698132;
  float magnetoHeading = 0.0;

  // How many decimal points we want to display
  // (this influences increment 10 or 100 m when adjusting distance)
  byte decimals = 2;

  // CAP heading in DEG 0-360
  word tripHeading = 0;

  // Our current speed
  byte tripSpeed = 0;

  // Our maximum speed
  byte tripMaxSpeed = 0;

  // Lifetime distance since last device reset
  unsigned long tripLifetime = 0;

  // Total distance (eg day/stage)
  unsigned long tripTotal = 0;

  // Partial distance (point - to point)
  unsigned long tripPartial = 0;
} state;

//
// Arduino
//

void setup(void)  {
  Serial.begin(9600);

  buttonUp.onPress(onButtonPressed);
  buttonUp.onRelease(150, onButtonReleased);
  buttonUp.onHoldRepeat(BUTTON_HOLD_DELAY, BUTTON_HOLD_REPEAT, onButtonHeld);

  buttonDown.onPress(onButtonPressed);
  buttonDown.onRelease(150, onButtonReleased);
  buttonDown.onHoldRepeat(BUTTON_HOLD_DELAY, BUTTON_HOLD_REPEAT, onButtonHeld);

  pinMode(SPEED_PIN, INPUT);

  // magneto.begin();

  gpsSerial.begin(9600);

  lcd.begin(LCD_CS_PIN, LCD_WR_PIN, LCD_DATA_PIN);
  lcd.clear();
  lcd.setBatteryLevel(0);
}

void loop(void) {
  buttonUp.update();
  buttonDown.update();

  // state.magnetoHeading = calculateMagnetoHeading(magneto, state.magnetoDeclinationAngle);
  gpsUpdate(state, 4.0);

  // Update data and display every display interval
  unsigned long currentMillis = millis();
  if (currentMillis - uiMillis >= UI_REFRESH_INTERVAL) {
    uiMillis = currentMillis;
    screenUpdate(state);
    screenUpdateBatteryIndicator(state);
  }

  if (currentMillis - sdCardMillis >= 3 * UI_REFRESH_INTERVAL) {
    sdCardMillis = currentMillis;
    String dataString = "SD Millis: " + String(sdCardMillis);
    sdWrite(dataString);
  }

  // WIP: This will need to be interrupt for the wheel sensor, placeholder
  //      for now to verify the idea...
  int val = analogRead(SPEED_PIN);
  if(val < 100) {
     Serial.print(F("Magnet close!"));
  }
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
  lcd.begin(LCD_CS_PIN, LCD_WR_PIN, LCD_DATA_PIN);
  switch (state.activeScreen) {
  case SCREEN_TOTAL:
    lcd.print(state.tripTotal / 1000.0, 1);
    break;
  case SCREEN_PARTIAL:
    lcd.print(state.tripPartial / 1000.0, state.decimals);
    break;
  case SCREEN_HEADING:
    // I don't like to check the gps here...
    gps.course.isValid()
      ? lcd.print(state.tripHeading, 0)
      : lcd.print(state.magnetoHeading, 0);
    break;
  case SCREEN_SPEED:
    lcd.print(state.tripSpeed, 0);
    break;
  case SCREEN_MENU:
    lcd.print(9999, 0);
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
    state.tripTotal = calculateTripValue(state.tripTotal, 10, btn.is(buttonUp), false);
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
  bool reset = buttonUp.isPressed() && buttonDown.isPressed();
  switch (state.activeScreen) {
  case SCREEN_TOTAL:
    state.tripTotal = reset ? 0 : calculateTripValue(state.tripTotal, 10, btn.is(buttonUp), reset);
    break;
  case SCREEN_PARTIAL:
    state.tripPartial = reset ? 0 : calculateTripValue(state.tripPartial, 10, btn.is(buttonUp), reset);
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
  byte level;
  if (state.gpsPrecision == -1) {
    state.gpsPrecision = 0;
    level = 0;
  } else if (state.gpsPrecision == 0) {
    state.gpsPrecision = -1;
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
// GPS
//
void gpsUpdate(State& state, double minDistanceTreshold) {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.satellites.isValid() && gps.satellites.isUpdated()) {
        state.gpsPrecision = gps.satellites.value();
      }
      if (gps.course.isValid() && gps.course.isUpdated()) {
        state.tripHeading = gps.course.deg();
      }
      if (gps.speed.isValid() && gps.speed.isUpdated()) {
        state.tripSpeed = gps.speed.kmph();
        if (state.tripSpeed > state.tripMaxSpeed) {
          state.tripMaxSpeed = state.tripSpeed;
        }
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
      // One thing also is to stop counting distance at low speeds like < 5kmh
      //
      // Another distance calculation may come from here, that avoids loads of
      // trigonometry but need to verify the accuracy.
      // https://www.instructables.com/Distance-measuring-and-more-device-using-Arduino-a/
      //
      if (gps.location.isValid() && gps.location.isUpdated()) {
        if (state.gpsLastLocation.lat == 0.0 && state.gpsLastLocation.lng == 0.0) {
          state.gpsLastLocation = { gps.location.lat(), gps.location.lng() };
        } else {
          unsigned long distance =
            (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),
                                                        gps.location.lng(),
                                                        state.gpsLastLocation.lat,
                                                        state.gpsLastLocation.lng);
          // Should we update or not?
          bool update = true;

          // If the location age is older than 1500ms we most likely don't have
          // gps fix anymore and we need to wait for new fix.
          if (update)
            update = gps.location.age() < (unsigned long)1500;

          // When speed is very low we risk having loads of error due to
          // coordinates being too close together
          if (update)
            update = state.tripSpeed > (byte)5;

          // Don't update distance when the distance is not within reasonable
          // difference. GPS has +- 2-3 meter possible error/noise...
          if(update)
            update = distance > (unsigned long)4;

          // If we think we should update distance, lets do it
          if (update) {
            state.gpsLastLocation = { gps.location.lat(), gps.location.lng() };
            state.tripLifetime += distance;
            state.tripPartial += distance;
            state.tripTotal += distance;
          }
        }
      }
    }
  }
}

//
// Speed
//

// TODO....

//
// Magnetometer
//
float calculateMagnetoHeading(Adafruit_HMC5883_Unified &mag, float declinationAngle) {
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

void sdWrite(String& data) {
  if (!SD.begin(9)) {
    Serial.println(F("Can't initialize SD Card..."));
    return; 
  }

  File file = SD.open("data.txt", FILE_WRITE);
  if (!file) {
    Serial.println(F("Can't open the data.txt"));
    file.close();
    return; 
  }
  file.println(data);
  file.close();
}