#include <Arduino.h>
#include <Wire.h>
#include <TMC5160.h> // https://github.com/tommag/TMC5160_Arduino/blob/master/examples/TMC5160_SPI/TMC5160_SPI.ino
#include <LiquidCrystal_I2C.h>
#include <avr/eeprom.h>
#include <TFLI2C.h> // https://github.com/budryerson/TFLuna-I2C

// comment out to disable log/debug/serial commands and reduce sketch size
#define DEBUG_ON

/*
	Comment out if uploading the sketch for the first time so
	it writes default settings to the Arduino's eeprom
*/
// #define RESTORE_DEFAULT_SETTINGS

/*
	Arduino pin-out. SDA (white wire) - A4, SCL (yellow/green wire) - A5
*/
const uint8_t DRIVER_ENABLE = 9,
			  DRIVER_MOSI = 11,
			  DRIVER_SCK = 13,
			  DRIVER_CS = 6,
			  DRIVER_MISO = 12;

const uint8_t BUTTON1 = 7,
			  BUTTON2 = 8,
			  BUTTON3 = A1,
			  BUTTON4 = A2;

const uint8_t HALL_SENSOR = 4, // sensor that detects the center position of the headlight, HIGH when no magnet, LOW when magnet nearby
	LCD_BRIGHTNESS = 3,		   // must be a PWM pin, used to set custom LCD brightness that extends the build-in on/off
	VOLTAGE_SENSE = A0;		   // connected to voltage divider and 12V input

const uint8_t LUNA_ENABLE_RELAY = 2, // pin that disables one of Luna modules so the address of the other one can be changed
	LUNA_ADDRESS_1 = 0x10,			 // this must be the default luna address
	LUNA_ADDRESS_2 = 0x11;			 // luna that is not connected through relay will have address changed to this value

/*
	Speeds to use for calibration function.
	Calibration function doesn't use deceleration right now so keep max speed low.
*/
#define DRIVER_MAX_SPEED_CALIBRATION 500
#define DRIVER_MAX_ACC_CALIBRATION 1000

/*
	With 72:8 gear ratio (9:1),
	stepper motor having 1.8 degrees per step
	and microsteps set to 16
	it takes 360 * 9 * 16 / 1.8 = 28800 steps for full rev.

	With 130:10 / 13:1 gear ratio and 2 micro steps = 5200
	With 120:12 and 2 micro steps = 4800
*/
#define DRIVER_MICROSTEPS 2						  // to disable microsteps change this to 0
#define STEPS_PER_REVOLUTION 4800				  // used by the centering function
#define STEPS_LIMIT STEPS_PER_REVOLUTION / 2 - 10 // How many steps it takes to reach end of travel from the center. Limits the headlight's maximum angle.

TMC5160_SPI tmc = TMC5160_SPI(DRIVER_CS);
LiquidCrystal_I2C lcd(0x27, 16, 2);
TFLI2C tflI2C;

/*
	The idea to store/save settings like this it taken from
	catalkn's post at https://forum.arduino.cc/t/how-to-save-configuration/45314/6
*/
struct SETTINGS_S
{
	int DRIVER_MAX_SPEED = STEPS_PER_REVOLUTION / 4; // in steps per second
	int DRIVER_MAX_ACC = STEPS_PER_REVOLUTION / 4;
	int DRIVER_CURRENT = 8;		  // 0-31 range, hold current will be half of this
	int DISPLAY_BRIGHTNESS = 100; // 0-255
	int SENSOR_UPDATE_TIME = 25;  // in ms, how often to read data from the gyro
	int STEPS_LIMIT_ALLOWED = STEPS_LIMIT;
	int MOVE_THRESHOLD = 2;					 // how many steps off "correct" position before is starts moving
	int MOVE_THRESHOLD_CENTER = 5;			 // how many steps off center before it starts moving
	char STARTUP_MODE = -1;					 // what should be the mode after first start, -1 default, 0 or 1 will call calibration function
	float VOLTAGE_WEIGHTING_PARAMETER = 0.1; // to filter voltage readings
	float SENSOR_WEIGHTING_PARAMETER = 0.1;	 // to filter LUNA readings
} const DEFAULT_SETTINGS;

SETTINGS_S SETTINGS = DEFAULT_SETTINGS;
// void eeprom_update_block(const void*, void*, size_t); //defined so VS code doesn't complain
// void eeprom_read_block(void*, const void*, size_t);

void loadSettings()
{
	eeprom_read_block((void *)&SETTINGS, (void *)0, sizeof(SETTINGS));

	// TMCdriver.rms_current(SETTINGS.DRIVER_CURRENT);
	tmc.setMaxSpeed(SETTINGS.DRIVER_MAX_SPEED);
	tmc.setAcceleration(SETTINGS.DRIVER_MAX_ACC);
	analogWrite(LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS);
}

void saveSettings()
{
	eeprom_update_block((const void *)&SETTINGS, (void *)0, sizeof(SETTINGS));
}

/*
	If uploading the sketch first time call this function!
	Don't enable the stepper driver or bad things might happen.
*/
void restoreDefaultSettings()
{
	SETTINGS = DEFAULT_SETTINGS;
	tmc.setMaxSpeed(SETTINGS.DRIVER_MAX_SPEED);
	tmc.setAcceleration(SETTINGS.DRIVER_MAX_ACC);
	analogWrite(LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS);

	saveSettings();
}

char mode = -1; // used in main loop to turn on/off gyro function

// debouncing class for simple buttons, default for them is high (internal pull up)
unsigned long lastButtonEvent = 0; // keeps time when was the button pressed last time
class button
{
public:
	// constructor that sets all variables to default
	button(byte buttonID_c)
	{
		pinMode(buttonID_c, INPUT_PULLUP);

		// update all variables
		buttonID = buttonID_c;
		buttonDebounce = 0;
		buttonLastState = HIGH;
		buttonCurrentState = HIGH;
		buttonPressUsed = true;
	}

	// returns 1 if pressed, returns 2 if being hold
	int state()
	{
		// read actual button state
		buttonCurrentState = digitalRead(buttonID);

		// check if it changed from last time, if it did take note of the time
		if (buttonCurrentState != buttonLastState)
			buttonDebounce = millis();

		// update the last state variable
		buttonLastState = buttonCurrentState;

		// check if button is being held down
		if (millis() - buttonDebounce > buttonHoldTimer && buttonCurrentState == LOW)
		{
			lastButtonEvent = millis();

			// prevent from setting the "isPressed" and "isHeld" signal simultaneously
			buttonPressUsed = true;

			return 2;
		}

		// now check if button is in its current state for long enough, but trigger only if button is let go
		else if ((millis() - buttonDebounce) > buttonDebounceDelay && millis() - buttonDebounce < buttonHoldTimer)
		{
			// remember to return true only once per button press
			// to prevent value increase when button is being hold
			if (!buttonPressUsed && buttonCurrentState == HIGH)
			{
				buttonPressUsed = true;

				// if so, update the optional variable, if set
				lastButtonEvent = millis();

				return 1;
			}

			// if not pressed, reset the 'use' variable
			if (buttonCurrentState == LOW)
				buttonPressUsed = false;
		}

		return 0;
	}

private:
	byte buttonID;								  // holds button id, like A1
	unsigned long buttonDebounce;				  // holds time when button changed last time
	int buttonLastState;						  // what was last button state
	int buttonCurrentState;						  // what is current state
	bool buttonPressUsed;						  // if button was determined to be pressed, use this to return true only once
	const unsigned long buttonDebounceDelay = 20; // minimum delay between change of signals
	const unsigned long buttonHoldTimer = 400;	  // how long to hold the button before it's considered being hold
};
button buttonOK(BUTTON4);
button buttonUp(BUTTON3);
button buttonDown(BUTTON1);
button buttonESC(BUTTON2);

// to compare floats
bool areEqual(float a, float b, float epsilon = 1e-6f)
{
	return abs(a - b) < epsilon;
}

#ifdef DEBUG_ON
/*
	Taken from https://learn.adafruit.com/scanning-i2c-addresses/arduino
*/
void findDevices()
{
	byte error, address;
	int nDevices;

	Serial.println(F("Scanning..."));

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmission to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print(F("I2C device found at address 0x"));
			if (address < 16)
				Serial.print(F("0"));
			Serial.println(address, HEX);

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print(F("Unknown error at address 0x"));
			if (address < 16)
				Serial.print(F("0"));
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println(F("No I2C devices found\n"));
	else
		Serial.println(F("done\n"));
}

/*
	Processes the serial commands for debugging
*/
// void serialCommands()
// {
// 	if (Serial.available())
// 	{
// 		String data = Serial.readString();

// 		switch (data.charAt(0))
// 		{
// 		case 's':
// 		{
// 			int speed = data.substring(1).toInt();

// 			if (speed == 0)
// 			{
// 				tmc.disable();
// 				Serial.println(F("Stepper disabled"));
// 			}
// 			else
// 			{
// 				Serial.print(F("Speed set to "));
// 				Serial.println(speed);
// 				tmc.enable();
// 				tmc.setMaxSpeed(speed);
// 			}
// 			break;
// 		}

// 		case 'p':
// 		{
// 			int position = data.substring(1).toInt();
// 			Serial.print(F("Moving stepper to position "));
// 			Serial.println(position);
// 			tmc.setTargetPosition(position);
// 			break;
// 		}

// 		case 'a':
// 		{
// 			int speed = data.substring(1).toInt();

// 			Serial.print(F("Acceleration set to "));
// 			Serial.println(speed);
// 			stepper.enableOutputs();
// 			stepper.setAcceleration(speed);
// 			break;
// 		}

// 		case 'c':
// 		{
// 			calibratePosition();
// 			break;
// 		}
// 		case 'd':
// 		{
// 			Serial.println(F("Stepper disabled."));
// 			stepper.disableOutputs();
// 			break;
// 		}
// 		default:
// 			Serial.println(F("Unknown command!"));
// 		}
// 	}
// }

#endif

/*
	Used for calibration only.
	Blocks the execution until stepper reached the target.
*/
void tmcBlockingMove(float targetPosition, bool stopWhenReachedHallSensor = true, bool stopWhenPastHallSensor = false)
{
	while (true)
	{
		bool hallSensor = digitalRead(HALL_SENSOR);
		if ((stopWhenPastHallSensor && !hallSensor) || (stopWhenReachedHallSensor && hallSensor) || areEqual(tmc.getCurrentPosition(), targetPosition))
		{
			tmc.stop();
			return;
		}
	}
}

/*
	Finds the center / calibrates the stepper using the hall sensor.
*/
const float HALL_RANGE_GUESS = 0.04f; // ~4% of a full revolution
const float BACKTRACK_IF_PAST_HALL = 0.3f;

inline bool hallDetected()
{
	return digitalRead(HALL_SENSOR) == LOW;
}
void moveUntilHallDetected(float targetPos)
{
	tmc.setTargetPosition(targetPos);
	while (!hallDetected() && !tmc.isTargetPositionReached())
	{
		// wait for motion
	}
	tmc.stop();
}
void moveUntilHallCleared(float targetPos)
{
	tmc.setTargetPosition(targetPos);
	while (hallDetected() && !tmc.isTargetPositionReached())
	{
		// wait for motion
	}
	tmc.stop();
}

void calibratePosition()
{
	int direction = 1;
	float position = 0;

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("Calibrating"));
	lcd.setCursor(4, 1);
	lcd.print(F("position..."));

	tmc.enable();
	delay(300);

	tmc.setMaxSpeed(DRIVER_MAX_SPEED_CALIBRATION);
	tmc.setAcceleration(DRIVER_MAX_ACC_CALIBRATION);

	// Move to 0 if not already there
	if (areEqual(tmc.getCurrentPosition(), 0))
	{
		tmcBlockingMove(0, false);
		if (!hallDetected())
		{
			tmcBlockingMove(STEPS_PER_REVOLUTION * HALL_RANGE_GUESS);
		}
	}

	// Sweep to find Hall sensor or end
	position = direction * STEPS_PER_REVOLUTION * 0.25f;
	moveUntilHallDetected(position);
	tmc.setCurrentPosition(0); // define as 0 temporarily

	// If still no detection, backtrack
	if (!hallDetected())
	{
		direction *= -1;
		tmcBlockingMove(STEPS_PER_REVOLUTION * BACKTRACK_IF_PAST_HALL);
	}

	tmc.setCurrentPosition(0); // Ensure zeroed

	// Lower speed for fine calibration
	tmc.setMaxSpeed(DRIVER_MAX_SPEED_CALIBRATION / 4);
	tmc.setAcceleration(DRIVER_MAX_ACC_CALIBRATION / 2);

	// Step 1: Move until Hall clears
	float travel = STEPS_PER_REVOLUTION * HALL_RANGE_GUESS * direction;
	moveUntilHallCleared(travel);

	// Step 2: Reverse and move until Hall is detected again
	direction *= -1;
	tmc.setCurrentPosition(0);
	travel = STEPS_PER_REVOLUTION * HALL_RANGE_GUESS * direction;
	moveUntilHallDetected(travel);

	float firstEdge = tmc.getCurrentPosition();

	// Step 3: Continue until Hall clears again
	moveUntilHallCleared(firstEdge + STEPS_PER_REVOLUTION * HALL_RANGE_GUESS * direction);

	// Step 4: Reverse again and detect second edge
	direction *= -1;
	moveUntilHallDetected(tmc.getCurrentPosition() + STEPS_PER_REVOLUTION * HALL_RANGE_GUESS * direction);

	float secondEdge = tmc.getCurrentPosition();

	// Step 5: Move to center between edges
	float centerPos = (firstEdge + secondEdge) / 2.0f;
	tmcBlockingMove(centerPos, false, false);

	// Finalize
	tmc.setCurrentPosition(0);
	tmc.setMaxSpeed(SETTINGS.DRIVER_MAX_SPEED);
	tmc.setAcceleration(SETTINGS.DRIVER_MAX_ACC);

	lcd.clear();
}

/*
	Those values were obtained by gathering data on different voltage levels and doing
	the linear curve fit in LoggerPro. More accurate than calculating from resistor's values used and requires less brain power.
*/
float voltMeter()
{
	// static byte voltageHistoryCounter = 0;
	// static int voltageHistory[VOLTAGE_HISTORY_SIZE] = {0};

	// voltageHistory[voltageHistoryCounter] = analogRead(VOLTAGE_SENSE);
	// voltageHistoryCounter++;
	// if (voltageHistoryCounter >= VOLTAGE_HISTORY_SIZE)
	// 	voltageHistoryCounter = 0;

	// unsigned long historySum = 0;
	// for (int i = 0; i < VOLTAGE_HISTORY_SIZE; i++)
	// {
	// 	historySum += voltageHistory[i];
	// }
	static float previousValue = 0;
	float currentValue = (float)analogRead(VOLTAGE_SENSE) * 0.02764 + 0.0708;
	previousValue = previousValue * (1 - SETTINGS.VOLTAGE_WEIGHTING_PARAMETER) + currentValue * SETTINGS.VOLTAGE_WEIGHTING_PARAMETER;

	return currentValue;
}

void setup()
{
#ifdef RESTORE_DEFAULT_SETTINGS
	restoreDefaultSettings();
#endif

	eeprom_read_block((void *)&SETTINGS, (void *)0, sizeof(SETTINGS));

#ifdef DEBUG_ON
	Serial.begin(115200);
	Serial.print(F("Setup begin...  "));
#endif
	Wire.begin();
	SPI.begin();

	digitalWrite(DRIVER_ENABLE, HIGH);
	pinMode(DRIVER_ENABLE, OUTPUT);

	pinMode(HALL_SENSOR, INPUT_PULLUP);
	pinMode(VOLTAGE_SENSE, INPUT);

	/*
		disable one of the luna sensors to change other luna's address
		luna power connected to normally closed relay, so writing 1 opens the relay
	*/
	pinMode(LUNA_ENABLE_RELAY, OUTPUT);
	digitalWrite(LUNA_ENABLE_RELAY, 1);

	// TMC driver - setup follows the SPI example from github
	TMC5160::PowerStageParameters powerStageParams; // defaults.
	TMC5160::MotorParameters motorParams;
	motorParams.globalScaler = 98; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
	motorParams.irun = SETTINGS.DRIVER_CURRENT;
	motorParams.ihold = SETTINGS.DRIVER_CURRENT / 2;

	tmc.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

	tmc.setRampMode(TMC5160::POSITIONING_MODE);
	tmc.setMaxSpeed(SETTINGS.DRIVER_MAX_SPEED);
	tmc.setAcceleration(SETTINGS.DRIVER_MAX_ACC);

	lcd.init();
	lcd.noBacklight(); // as brightness is controlled by arduino, LCD is modified to allow that
	analogWrite(LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS);
	lcd.clear();

	if (SETTINGS.STARTUP_MODE != -1)
	{
		calibratePosition();
		mode = SETTINGS.STARTUP_MODE;
	}

	// if luna found on the default address, change it
	// if arduino rebooted without power cycling luna, it won't respond to this address
	Wire.beginTransmission(LUNA_ADDRESS_1);
	if (Wire.endTransmission() == 0)
	{
		tflI2C.Set_I2C_Addr(LUNA_ADDRESS_2, LUNA_ADDRESS_1);
	}
	digitalWrite(LUNA_ENABLE_RELAY, 0); // enable the second luna

#ifdef DEBUG_ON
	Serial.println(F(" done."));
#endif
}

/*
	Used to set numerical setting values.

	Input:
		- startValue - current setting/value
		- min, max - allowed range for that setting
		- multiplier - how much value should change per single button event

	Returns:
		- if ESC button pressed - original/unchanged startValue
		- if OK pressed - currently set/displayed value
*/
int menuInner(int startValue, int min, int max, int y, int multiplier = 1)
{
	int setValue = startValue;
	int displayedValue = startValue + 1; // to make sure display updates this time

	// figure out maximum number of characters needed to display the value
	int maxLength = String(max).length();
	int maxLength2 = String(min).length() + 1;
	if (maxLength2 > maxLength)
		maxLength = maxLength2;

	while (1)
	{
		if (displayedValue != setValue)
		{
			// clear the fields
			for (int i = 16 - maxLength; i <= 15; i++)
			{
				lcd.setCursor(i, y);
				lcd.print(F(" "));
			}

			// figure out how many characters we need
			lcd.setCursor(16 - String(setValue).length(), y);

			lcd.print(setValue);

			displayedValue = setValue;
		}

		if (buttonUp.state())
		{
			setValue += multiplier;
			if (setValue > max)
				setValue = max;
		}
		else if (buttonDown.state())
		{
			setValue -= multiplier;
			if (setValue < min)
				setValue = min;
		}
		else if (buttonESC.state())
		{
			return startValue;
		}
		else if (buttonOK.state() == 1)
		{
			return setValue;
		}
	}
}

void menu_motorDriver()
{
	int menuCurrentItem = 0;
	const int menuItemsCount = 5; // increase when adding menu options
	bool displayUpdate = true;

	while (1)
	{
		if (displayUpdate)
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			int counter = 0;

			// max string length = 13 because we also need 2 fields to print the arrow
			// 0
			if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Max Speed"));
				lcd.setCursor(3, 1);
				lcd.print(F("Acceleration"));
			}
			// 2
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Current/Amps"));
				lcd.setCursor(3, 1);
				lcd.print(F("Move thr."));
			}
			// 4
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Center thr."));
				lcd.setCursor(3, 1);
				lcd.print(F("Steps limit"));
			}

			// print selection arrow
			lcd.setCursor(0, menuCurrentItem % 2);
			lcd.print(F("->"));

			displayUpdate = false;
		}

		if (buttonESC.state())
			return;
		else if (buttonUp.state())
		{
			menuCurrentItem--;
			if (menuCurrentItem < 0)
				menuCurrentItem = menuItemsCount;
			displayUpdate = 1;
		}
		else if (buttonDown.state())
		{
			menuCurrentItem++;
			if (menuCurrentItem > menuItemsCount)
				menuCurrentItem = 0;
			displayUpdate = 1;
		}
		else if (buttonOK.state())
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			displayUpdate = true; // so display updates when we exit the sub-menu

			if (menuCurrentItem == 0)
			{
				lcd.print(F("Max speed"));
				SETTINGS.DRIVER_MAX_SPEED = menuInner(SETTINGS.DRIVER_MAX_SPEED, 0, 5000, 1, 10);
				tmc.setMaxSpeed(SETTINGS.DRIVER_MAX_SPEED);
				continue;
			}
			else if (menuCurrentItem == 1)
			{
				lcd.print(F("Acceleration"));
				SETTINGS.DRIVER_MAX_ACC = menuInner(SETTINGS.DRIVER_MAX_ACC, 0, 8000, 1, 10);
				tmc.setAcceleration(SETTINGS.DRIVER_MAX_ACC);
				continue;
			}
			else if (menuCurrentItem == 2)
			{
				lcd.print(F("I. Restart req!"));
				SETTINGS.DRIVER_CURRENT = menuInner(SETTINGS.DRIVER_CURRENT, 0, 2000, 1, 10);
				continue;
			}
			else if (menuCurrentItem == 3)
			{
				lcd.print(F("Move threshold"));
				SETTINGS.MOVE_THRESHOLD = menuInner(SETTINGS.MOVE_THRESHOLD, 0, STEPS_LIMIT, 1);
				continue;
			}
			else if (menuCurrentItem == 4)
			{
				lcd.print(F("Center thr."));
				SETTINGS.MOVE_THRESHOLD_CENTER = menuInner(SETTINGS.MOVE_THRESHOLD_CENTER, 0, STEPS_LIMIT, 1);
				continue;
			}
			else if (menuCurrentItem == 5)
			{
				lcd.print(F("Rotation limit"));
				SETTINGS.STEPS_LIMIT_ALLOWED = menuInner(SETTINGS.STEPS_LIMIT_ALLOWED, 0, STEPS_LIMIT, 1);
				continue;
			}
		}
	}
}

/*
	Gets raw distance from the sensors.
	Distance returned as reference parameters.
	If distance measurement was successful returns true.
*/
bool getRawDistance(float &distance)
{
	int16_t tfDist1 = 0, tfDist2 = 0;
	bool result = tflI2C.getData(tfDist1, LUNA_ADDRESS_1) && tflI2C.getData(tfDist2, LUNA_ADDRESS_2);

	if (result)
	{
		distance = tfDist1 - tfDist2;
	}

	return result;
}

/*
	Calculates the angle of the bike.
	Returns true if angle reading was successful.
	Returns the angle by the reference parameter.
*/
bool getBikeAngle(float &angle)
{
	static float lastAngle = 0;
	float tempAngle = 0;

	if (getRawDistance(tempAngle))
	{
		// got this function by measuring angle and the readout and doing the
		// best function fit in Logger Pro
		tempAngle = 38.43 * sin(0.01974 * tempAngle + 6.271) + 0.4739;
		lastAngle = tempAngle * SETTINGS.SENSOR_WEIGHTING_PARAMETER + lastAngle * (1 - SETTINGS.SENSOR_WEIGHTING_PARAMETER);
		angle = lastAngle;

		return 1;
	}

	return 0;
}

void menu_sensor()
{
	int menuCurrentItem = 0;
	const int menuItemsCount = 1; // increase when adding menu options
	bool displayUpdate = true;

	while (1)
	{
		if (displayUpdate)
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			int counter = 0;

			// max string length = 13 because we also need 2 fields to print the arrow
			// 0
			if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Raw distance"));
				lcd.setCursor(3, 1);
				lcd.print(F("Update freq."));
			}

			// print selection arrow
			lcd.setCursor(0, menuCurrentItem % 2);
			lcd.print(F("->"));

			displayUpdate = false;
		}

		if (buttonESC.state())
			return;
		else if (buttonUp.state())
		{
			menuCurrentItem--;
			if (menuCurrentItem < 0)
				menuCurrentItem = menuItemsCount;
			displayUpdate = 1;
		}
		else if (buttonDown.state())
		{
			menuCurrentItem++;
			if (menuCurrentItem > menuItemsCount)
				menuCurrentItem = 0;
			displayUpdate = 1;
		}
		else if (buttonOK.state())
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			displayUpdate = true; // so display updates when we exit the sub-menu

			if (menuCurrentItem == 0)
			{
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Distance: ");

				unsigned long lastUpd = 0;
				while (1)
				{
					if (millis() - lastUpd > 500)
					{
						lcd.setCursor(0, 1);
						lcd.print(F("          "));
						lcd.setCursor(0, 1);
						float dist = 0;
						if (getRawDistance(dist))
							lcd.print(dist);

						lastUpd = millis();
					}

					if (buttonESC.state())
						break;
				}
			}
			else if (menuCurrentItem == 1)
			{
				lcd.print(F("Update delay"));
				SETTINGS.SENSOR_UPDATE_TIME = menuInner(SETTINGS.SENSOR_UPDATE_TIME, 0, 500, 1, 1);
				continue;
			}
		}
	}
}

/*
	Test menu is used to move the headlight to test for binding /
	acceleration / speed problems.
*/
void menu_test()
{

	calibratePosition();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("Press OK to test"));
	int sweepSteps = STEPS_LIMIT;
	int sweepStepsDisp = -1;

	while (1)
	{
		if (sweepStepsDisp != sweepSteps)
		{
			lcd.setCursor(0, 1);
			for (int i = 0; i < 16; i++)
				lcd.print(F(" "));
			lcd.setCursor(0, 1);
			lcd.print(F("Steps: "));
			lcd.print(sweepSteps);

			sweepStepsDisp = sweepSteps;
		}

		if (buttonESC.state())
		{
			tmc.stop();
			break;
		}
		else if (buttonOK.state() == 1)
		{
			tmcBlockingMove(sweepSteps, false, false);
			tmcBlockingMove(-sweepSteps, false, false);
			tmcBlockingMove(0, false, false);
		}
		else if (buttonUp.state())
		{
			sweepSteps += 10;
			if (sweepSteps > STEPS_LIMIT)
				sweepSteps = STEPS_LIMIT;
		}
		else if (buttonDown.state())
		{
			sweepSteps -= 10;
			if (sweepSteps < 0)
				sweepSteps = 0;
		}
	}
}

void menu_main()
{
	int menuCurrentItem = 0;
	const int menuItemsCount = 7; // increase when adding menu options
	bool displayUpdate = true;

	while (1)
	{
		if (displayUpdate)
		{
			lcd.clear();
			lcd.setCursor(3, 0);
			int counter = 0;

			// Up to 13 characters to make room for the arrow
			// 0
			if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Sensors")); // 0
				lcd.setCursor(3, 1);
				lcd.print(F("Motor driver")); // 1
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Brightness")); // 2
				lcd.setCursor(3, 1);
				lcd.print(F("Save settings")); // 3
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Load settings")); // 4
				lcd.setCursor(3, 1);
				lcd.print(F("Restore def.")); // 5
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print(F("Startup mode")); // 6
				lcd.setCursor(3, 1);
				lcd.print(F("Test")); // 7
			}

			// print selection arrow
			lcd.setCursor(0, menuCurrentItem % 2);
			lcd.print(F("->"));

			displayUpdate = false;
		}

		if (buttonESC.state())
			return;
		else if (buttonUp.state())
		{
			menuCurrentItem--;
			if (menuCurrentItem < 0)
				menuCurrentItem = menuItemsCount;
			displayUpdate = 1;
		}
		else if (buttonDown.state())
		{
			menuCurrentItem++;
			if (menuCurrentItem > menuItemsCount)
				menuCurrentItem = 0;
			displayUpdate = 1;
		}
		else if (buttonOK.state())
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			displayUpdate = true; // so display updates when we exit the sub-menu

			if (menuCurrentItem == 0)
			{
				menu_sensor();
				continue;
			}
			else if (menuCurrentItem == 1)
			{
				menu_motorDriver();
				continue;
			}
			else if (menuCurrentItem == 2) // screen brightness
			{
				// could use menuInner function but I want the brightness to update as
				// I change it.
				lcd.print(F("Display"));
				lcd.setCursor(0, 1);
				lcd.print(F("brightness"));
				int DISPLAY_BRIGHTNESS_local = SETTINGS.DISPLAY_BRIGHTNESS;
				int DISPLAY_BRIGHTNESS_local_displayed = -1;

				while (1)
				{
					if (DISPLAY_BRIGHTNESS_local_displayed != DISPLAY_BRIGHTNESS_local)
					{
						lcd.setCursor(13, 0);
						lcd.print(F("   "));
						lcd.setCursor(13, 0);
						lcd.print(DISPLAY_BRIGHTNESS_local);
						DISPLAY_BRIGHTNESS_local_displayed = DISPLAY_BRIGHTNESS_local;
					}

					int bupState = buttonUp.state();
					int bdownState = buttonDown.state();

					if (bupState)
					{
						DISPLAY_BRIGHTNESS_local += 1 + 4 * (bupState - 1); // state == 1 when pressed, inc by 1, state == 2 if held down, inc by more
						if (DISPLAY_BRIGHTNESS_local > 255)
							DISPLAY_BRIGHTNESS_local = 255;
						analogWrite(LCD_BRIGHTNESS, DISPLAY_BRIGHTNESS_local);
					}
					else if (bdownState)
					{
						DISPLAY_BRIGHTNESS_local -= 1 + 4 * (bdownState - 1);
						if (DISPLAY_BRIGHTNESS_local < 0)
							DISPLAY_BRIGHTNESS_local = 0;
						analogWrite(LCD_BRIGHTNESS, DISPLAY_BRIGHTNESS_local);
					}
					else if (buttonESC.state())
					{
						analogWrite(LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS);
						break;
					}
					else if (buttonOK.state())
					{
						SETTINGS.DISPLAY_BRIGHTNESS = DISPLAY_BRIGHTNESS_local;
						break;
					}
				}
			}
			else if (menuCurrentItem == 3) // save
			{
				saveSettings();
				continue;
			}
			else if (menuCurrentItem == 4) // load
			{
				loadSettings();
				continue;
			}
			else if (menuCurrentItem == 5) // default
			{
				restoreDefaultSettings();
				continue;
			}
			else if (menuCurrentItem == 6) // startup mode
			{
				char defaultMode = SETTINGS.STARTUP_MODE;
				char modeDisplayed = defaultMode + 1; // so it updates the display on the first run

				while (1)
				{
					if (defaultMode > 1)
						defaultMode = -1;
					if (defaultMode < -1)
						defaultMode = 1;

					if (defaultMode != modeDisplayed)
					{
						lcd.clear();
						lcd.setCursor(0, 0);
						lcd.print(F("Startup mode: "));
						lcd.setCursor(0, 1);

						if (defaultMode == -1)
							lcd.print(F("Req.calibration"));
						else if (defaultMode == 0)
							lcd.print(F("Calib then OFF"));
						else if (defaultMode == 1)
							lcd.print(F("Calib then ON"));

						modeDisplayed = defaultMode;
					}

					if (buttonUp.state() == 1)
					{
						defaultMode++;
					}
					if (buttonDown.state() == 1)
					{
						defaultMode--;
					}
					if (buttonESC.state())
					{
						break;
					}
					if (buttonOK.state() == 1)
					{
						SETTINGS.STARTUP_MODE = defaultMode;
						break;
					}
				}

				continue;
			}
			else if (menuCurrentItem == 7) // Test menu
			{
				menu_test();
				continue;
			}
		}
	}
}

/*
	Is stepper.distanceToGo() > 0 then only gyro and stepper.run part is running.
	Display and user input not checked to make stepper run function have less delay.

	unsigned long variables usually used to keep track of time
*/
void loop()
{
	static float bikeLeanAngle = 0;
	static int previousTarget = 0; // what was the target stepper position

	/*
		Update the sensors.
	*/
	static unsigned long lastSensorUpdate = 0;
	if (millis() - lastSensorUpdate > (unsigned long)SETTINGS.SENSOR_UPDATE_TIME)
	{
		float angle = 0;
		if (getBikeAngle(angle))
		{
			bikeLeanAngle = angle;
		}
		lastSensorUpdate = millis();
	}

	/*
		Stepper motor control.
	*/
	if (mode == 1)
	{
		// bikeLeanAngle is in degrees off center
		int newTarget = bikeLeanAngle * STEPS_PER_REVOLUTION / 4 / 90;

		if (abs(newTarget) < SETTINGS.MOVE_THRESHOLD_CENTER)
		{
			newTarget = 0;
			previousTarget = 0;
			tmc.setTargetPosition(0);
		}
		else if (abs(newTarget - previousTarget) > SETTINGS.MOVE_THRESHOLD)
		{
			if (newTarget > SETTINGS.STEPS_LIMIT_ALLOWED)
				newTarget = SETTINGS.STEPS_LIMIT_ALLOWED;
			else if (newTarget < -SETTINGS.STEPS_LIMIT_ALLOWED)
				newTarget = -SETTINGS.STEPS_LIMIT_ALLOWED;

			tmc.setTargetPosition(newTarget);

			previousTarget = newTarget;
		}
	}

	/*
		Check for user input.
		Only very important button checks here as it's possibly
		done when stepper driver is running.
	*/
	switch (buttonOK.state())
	{
	case 1:
		if (mode == -1)
		{
			calibratePosition();
			mode = 0;
		}
		else
		{
			if (mode == 0)
				mode = 1;
			else
				mode = 0;

			if (!mode)
			{
				tmc.setTargetPosition(0);
				previousTarget = 0;
			}
		}
		break;

		// if button is held down, recalibrate without changing the mode
	case 2:
		int modeSave = mode;

		calibratePosition();

		if (modeSave == 1)
			mode = 1;
		else
			mode = 0;

		break;
	}

	if (buttonESC.state())
	{
		// if we were calibrated and possibly running, go back to the center before entering the menu
		if (mode != -1)
		{
			tmcBlockingMove(0, false, false);
		}

		tmc.stop();
		mode = -1;
		menu_main();
		return;
	}
	else if (buttonDown.state() == 2)
	{
		mode = -1;
		tmc.stop();
	}

	/*
		Update the display.
	*/
	static unsigned long lastDisplayUpdate = 0;
	if (millis() - lastDisplayUpdate > 300)
	{
		lcd.clear();
		lcd.setCursor(0, 0);

		lcd.print(F("Status:"));
		lcd.setCursor(0, 1);
		if (mode == 0)
			lcd.print(F("OFF"));
		else if (mode == 1)
			lcd.print(F("ON"));
		else if (mode == -1)
		{
			static byte msg_counter = 0;

			if (msg_counter < 3)
				lcd.print(F("press OK"));
			else if (msg_counter < 6)
				lcd.print(F("to"));
			else
				lcd.print(F("calibrate"));

			msg_counter++;
			if (msg_counter > 9)
				msg_counter = 0;
		}
		else
			lcd.print(F("err"));

		float volts = voltMeter();
		if (volts >= 10)
			lcd.setCursor(10, 1);
		else
			lcd.setCursor(11, 1);
		lcd.print(volts);
		lcd.print(F("V"));

		int printOffset = 0;
		if (bikeLeanAngle < 0)
			printOffset--;
		if (abs(bikeLeanAngle) >= 10)
			printOffset--;
		if (abs(bikeLeanAngle) >= 100)
			printOffset--;

		lcd.setCursor(printOffset + 12, 0);
		lcd.print(bikeLeanAngle);

		lastDisplayUpdate = millis();
	}

#ifdef DEBUG_ON
	// serialCommands();
	static unsigned long lastLogEvent = 0;
	if (millis() - lastLogEvent > 500)
	{

		lastLogEvent = millis();
	}

#endif
}