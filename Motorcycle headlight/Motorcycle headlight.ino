﻿#include <Wire.h>
#include <TMCStepper.h>         // https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>		// https://github.com/teemuatlut/TMCStepper/blob/master/examples/TMC_AccelStepper/TMC_AccelStepper.ino
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/eeprom.h>

//#define debug //comment out to disable debug/serial commands

/*
	Arduino pin-out. SDA (white) - A4, SCL (yellow) - A5
*/
#define DRIVER_ENABLE 9
#define DRIVER_DIRECTION 10
#define DRIVER_STEP 3
#define DRIVER_SWCLCK 4
#define DRIVER_TX 6	//TX and RX must be swapped around
#define DRIVER_RX 5
#define HALL_SENSOR 12
#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 A1
#define BUTTON4 A2
#define LCD_BRIGHTNESS 11 //must be a PWM pin
#define VOLTAGE_SENSE A0

int mode = -1; //used in main loop to turn on/off gyro function

#define DRIVER_ADDRESS 0b00
#define DRIVER_RSENSE 0.11f
#define MPU 0x68

/*
	Speeds to use for calibration function.
*/
#define DRIVER_MAX_SPEED_CALIBRATION 500
#define DRIVER_MAX_ACC_CALIBRATION 500

/*
	With 72:8 gear ratio (9:1),
	stepper motor having 1.8 degrees per step
	and microsteps set to 16
	it takes 360 * 9 * 16 / 1.8 = 28800 steps for full rev.

	With microsteps set to 0, it takes 1800 steps for full rev.
*/
#define DRIVER_MICROSTEPS 0
#define STEPS_PER_REVOLUTION 1800 //used by the centering function
#define STEPS_LIMIT 400	//How many steps it takes to reach end of travel from the center. Limits the headlight's maximum angle.

#define DEFAULT_SETTINGS { 1200,2000,1600,50,25,-7, STEPS_LIMIT, 1,10,false }

struct
{
	int DRIVER_MAX_SPEED;	//in steps per second?
	int DRIVER_MAX_ACC;
	int DRIVER_CURRENT;		//in mA?
	int DISPLAY_BRIGHTNESS;	//0-255
	int GYRO_UPDATE_TIME;		//in ms, how often to read data from the gyro
	float POSITION_OFFSET;		//used to calibrate the center gyro position
	int STEPS_LIMIT_ALLOWED;
	int MOVE_THRESHOLD;			//how many steps off "correct" position before is starts moving
	int MOVE_THRESHOLD_CENTER; //how many steps off center before it starts moving
	bool AUTO_POSITION_OFFSET;
} SETTINGS = DEFAULT_SETTINGS;

SoftwareSerial SoftSerial( DRIVER_RX, DRIVER_TX );
TMC2209Stepper TMCdriver( &SoftSerial, DRIVER_RSENSE, DRIVER_ADDRESS );
AccelStepper stepper = AccelStepper( stepper.DRIVER, DRIVER_STEP, DRIVER_DIRECTION );
LiquidCrystal_I2C lcd( 0x27, 16, 2 );
Adafruit_BNO055 bno = Adafruit_BNO055( 55, 0x28, &Wire );

//debouncing class for simple buttons, default for them is high (internal pull up)
unsigned long lastButtonEvent;	//keeps time when was the button pressed last time
class button
{
public:
	//constructor that sets all variables to default
	button( byte buttonID_c )
	{
		pinMode( buttonID_c, INPUT_PULLUP );

		//update all variables
		buttonID = buttonID_c;
		buttonDebounce = 0;
		buttonLastState = HIGH;
		buttonCurrentState = HIGH;
		buttonPressUsed = true;
	}


	//returns 1 if pressed, returns 2 if being hold
	int state()
	{
		//read actual button state
		buttonCurrentState = digitalRead( buttonID );

		//check if it changed from last time, if it did take note of the time
		if (buttonCurrentState != buttonLastState)
			buttonDebounce = millis();

		//update the last state variable
		buttonLastState = buttonCurrentState;

		//check if button is being held down
		if (millis() - buttonDebounce > buttonHoldTimer && buttonCurrentState == LOW)
		{
			lastButtonEvent = millis();

			//prevent from setting the "isPressed" and "isHeld" signal simultaneously 
			buttonPressUsed = true;

			return 2;
		}

		//now check if button is in its current state for long enough, but trigger only if button is let go
		else if ((millis() - buttonDebounce) > buttonDebounceDelay && millis() - buttonDebounce < buttonHoldTimer)
		{
			//remember to return true only once per button press
			//to prevent value increase when button is being hold
			if (!buttonPressUsed && buttonCurrentState == HIGH)
			{
				buttonPressUsed = true;

				//if so, update the optional variable, if set
				lastButtonEvent = millis();

				return 1;
			}

			//if not pressed, reset the 'use' variable
			if (buttonCurrentState == LOW)
				buttonPressUsed = false;

		}

		return 0;
	}

private:
	byte buttonID;					//holds button id, like A1
	unsigned long buttonDebounce;	//holds time when button changed last time
	int buttonLastState;			//what was last button state
	int buttonCurrentState;			//what is current state
	bool buttonPressUsed;			//if button was determined to be pressed, use this to return true only once
	const unsigned long buttonDebounceDelay = 20;	//minimum delay between change of signals
	const unsigned long buttonHoldTimer = 400;		//how long to hold the button before it's considered being hold
};
button buttonOK( BUTTON4 );
button buttonUp( BUTTON3 );
button buttonDown( BUTTON1 );
button buttonESC( BUTTON2 );


#ifdef debug

/*
	Taken from https://learn.adafruit.com/scanning-i2c-addresses/arduino
*/
void findDevices()
{
	byte error, address;
	int nDevices;

	Serial.println( F( "Scanning..." ) );

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission( address );
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print( F( "I2C device found at address 0x" ) );
			if (address < 16)
				Serial.print( F( "0" ) );
			Serial.println( address, HEX );

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print( F( "Unknown error at address 0x" ) );
			if (address < 16)
				Serial.print( F( "0" ) );
			Serial.println( address, HEX );
		}
	}
	if (nDevices == 0)
		Serial.println( F( "No I2C devices found\n" ) );
	else
		Serial.println( F( "done\n" ) );
}


/*
	Processes the serial commands
*/
void serialCommands()
{
	if (Serial.available())
	{
		String data = Serial.readString();

		switch (data.charAt( 0 ))
		{
		case 's':
		{
			int speed = data.substring( 1 ).toInt();

			if (speed == 0)
			{
				stepper.disableOutputs();
				Serial.println( F( "Stepper disabled" ) );
			}
			else
			{
				Serial.print( F( "Speed set to " ) );
				Serial.println( speed );
				stepper.enableOutputs();
				stepper.setMaxSpeed( speed );
			}
			break;
		}

		case 'p':
		{
			int position = data.substring( 1 ).toInt();
			Serial.print( F( "Moving stepper to position " ) );
			Serial.println( position );
			stepper.moveTo( position );
			break;
		}

		case 'a':
		{
			int speed = data.substring( 1 ).toInt();

			Serial.print( F( "Acceleration set to " ) );
			Serial.println( speed );
			stepper.enableOutputs();
			stepper.setAcceleration( speed );
			break;
		}

		case 'c':
		{
			calibratePosition();
			break;
		}
		case 'd':
		{
			Serial.println( F( "Stepper disabled." ) );
			stepper.disableOutputs();
			break;
		}

		default:
			Serial.println( F( "Unknown command!" ) );
		}
	}
}

#endif

/*
	Finds the center / calibrates the stepper
	using the hall sensor.
*/
void calibratePosition()
{
	//setup
	int direction = 1;
	lcd.clear();
	lcd.setCursor( 0, 0 );
	lcd.print( F( "Calibrating" ) );
	lcd.setCursor( 4, 1 );
	lcd.print( F( "position..." ) );
	if (digitalRead( DRIVER_ENABLE ))
	{
		stepper.enableOutputs();
		delay( 300 );
	}
	stepper.setMaxSpeed( DRIVER_MAX_SPEED_CALIBRATION );
	stepper.setAcceleration( DRIVER_MAX_ACC_CALIBRATION );


	//if already calibrated before, move to 0 to reduce time
	//if just started or already on 0 this function returns 0
	if (stepper.currentPosition())
	{
		stepper.moveTo( 0 );
		while (stepper.distanceToGo()) stepper.run();
		//if moved to 0 but still didn't reach HALL (due to some missed steps??) go a bit further
		if (digitalRead( HALL_SENSOR ))
			stepper.moveTo( STEPS_PER_REVOLUTION * 0.04 );
		while (stepper.distanceToGo()) stepper.run();
	}

	//arbitrary multiplier - want to move not more than less than 45 degrees
	stepper.move( -STEPS_PER_REVOLUTION * 0.22 );

	while (stepper.distanceToGo() != 0 && digitalRead( HALL_SENSOR ))
	{
		stepper.run();
	}

	stepper.setCurrentPosition( 0 );

	//	if hall still reads high, we must be past it (and stalled on the end stop), so rotate back to the middle
	if (digitalRead( HALL_SENSOR ))
	{
		direction = -1; //set so next steps of finding the center accelerate in the same direction

		stepper.move( STEPS_PER_REVOLUTION * 0.22 );

		while (stepper.distanceToGo() != 0 && digitalRead( HALL_SENSOR ))
		{
			stepper.run();
		}
	}

	stepper.setCurrentPosition( 0 );

	//at this point we more or less should be in the middle
	//hall should read 0
	//so do small steps to actually find the middle
	stepper.setMaxSpeed( DRIVER_MAX_SPEED_CALIBRATION / 4 );
	stepper.setAcceleration( DRIVER_MAX_ACC_CALIBRATION / 2 );

	//again arbitrary multiplier - too small and we don't reach other side of the magnet
	stepper.move( -STEPS_PER_REVOLUTION * 0.04 * direction );

	while (stepper.distanceToGo() != 0 && !digitalRead( HALL_SENSOR ))
	{
		stepper.run();
	}

	//reached some point where hall no longer reads 0
	//reset position to 0
	//move in other direction until hall reads 0 and then reads 1 again

	stepper.setCurrentPosition( 0 );
	stepper.move( STEPS_PER_REVOLUTION * 0.04 * direction );

	while (stepper.distanceToGo() != 0 && digitalRead( HALL_SENSOR ))
	{
		stepper.run();
	}
	while (stepper.distanceToGo() != 0 && !digitalRead( HALL_SENSOR ))
	{
		stepper.run();
	}


	//now reached other side of the hall range, so actual middle position is somewhere in between
	stepper.moveTo( stepper.currentPosition() / 2 );

	while (stepper.distanceToGo() != 0)
		stepper.run();

	//now we are in the actual middle position
	stepper.setCurrentPosition( 0 );

	//go back to the previous settings
	stepper.setMaxSpeed( SETTINGS.DRIVER_MAX_SPEED );
	stepper.setAcceleration( SETTINGS.DRIVER_MAX_ACC );
	lcd.clear();
}

/*
	Those values were obtained by gathering data on different voltage levels and doing
	the linear curve fit in LoggerPro. More accurate than calculating from resistor's values used and requires less brain power.
*/
float voltMeter() { return (float) analogRead( VOLTAGE_SENSE ) * 0.02764 + 0.07088; }

/*
	Returns the angle from the gyro. In degrees off center.
*/
double readSensorData()
{
	sensors_event_t orientationData;
	bno.getEvent( &orientationData, Adafruit_BNO055::VECTOR_EULER );

	return orientationData.orientation.y - SETTINGS.POSITION_OFFSET;
}


void setup()
{
	eeprom_read_block( (void*) &SETTINGS, (void*) 0, sizeof( SETTINGS ) );

#ifdef debug
	Serial.begin( 9600 );
	Serial.print( F( "Setup begin...  " ) );
#endif
	SoftSerial.begin( 9600 );
	TMCdriver.beginSerial( 9600 );

	Wire.begin();

	digitalWrite( DRIVER_ENABLE, HIGH );
	pinMode( DRIVER_ENABLE, OUTPUT );
	pinMode( DRIVER_DIRECTION, OUTPUT );
	pinMode( DRIVER_STEP, OUTPUT );
	pinMode( HALL_SENSOR, INPUT_PULLUP );
	pinMode( VOLTAGE_SENSE, INPUT );

	TMCdriver.begin();
	TMCdriver.rms_current( SETTINGS.DRIVER_CURRENT );
	TMCdriver.pwm_autoscale( 1 );
	TMCdriver.microsteps( DRIVER_MICROSTEPS );
	stepper.setMaxSpeed( SETTINGS.DRIVER_MAX_SPEED );
	stepper.setAcceleration( SETTINGS.DRIVER_MAX_ACC );
	stepper.setEnablePin( DRIVER_ENABLE );
	stepper.setPinsInverted( false, false, true );
	stepper.disableOutputs();

	lcd.begin();
	lcd.noBacklight(); //as brightness is controlled by arduino
	analogWrite( LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS );
	lcd.clear();

	if (!bno.begin())
	{
		digitalWrite( 13, HIGH );
	}

	//findDevices();

	calibratePosition();
#ifdef debug
	Serial.println( F( " done." ) );
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
int menuInner( int startValue, int min, int max, int y, int multiplier = 1 )
{
	int setValue = startValue;
	int displayedValue = startValue + 1; //to make sure display updates this time

	//figure out maximum number of characters needed to display the value
	int maxLength = String( max ).length();
	int maxLength2 = String( min ).length() + 1;
	if (maxLength2 > maxLength) maxLength = maxLength2;

	while (1)
	{
		if (displayedValue != setValue)
		{
			//clear the fields
			for (int i = 16 - maxLength; i <= 15; i++)
			{
				lcd.setCursor( i, y );
				lcd.print( F( " " ) );
			}

			//figure out how many characters we need
			lcd.setCursor( 16 - String( setValue ).length(), y );

			lcd.print( setValue );

			displayedValue = setValue;
		}

		if (buttonUp.state())
		{
			setValue += multiplier;
			if (setValue > max) setValue = max;
		}
		else if (buttonDown.state())
		{
			setValue -= multiplier;
			if (setValue < min) setValue = min;
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


/*
	Settings menu
*/
void menu()
{
	int menuCurrentItem = 0;
	const int menuItemsCount = 12; //increase when adding menu options
	bool displayUpdate = true;


	while (1)
	{
		if (displayUpdate)
		{
			lcd.clear();
			lcd.setCursor( 3, 0 );
			int counter = 0;

			//max string length = 13 because we also need 2 fields to print the arrow
			//0
			if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Calib. light" ) );
				lcd.setCursor( 3, 1 );
				lcd.print( F( "Calib. gyro" ) );
			}
			//2
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Driver speed" ) );
				lcd.setCursor( 3, 1 );
				lcd.print( F( "Driver accel" ) );
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Driver Amps" ) );
				lcd.setCursor( 3, 1 );
				lcd.print( F( "Move thr." ) );
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Center thr." ) );
				lcd.setCursor( 3, 1 );
				lcd.print( F( "Steps limit" ) );
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Brightness" ) );
				lcd.setCursor( 3, 1 );
				lcd.print( F( "Auto calib." ) );
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Save setting" ) );
				lcd.setCursor( 3, 1 );
				lcd.print( F( "Load setting" ) );
			}
			else if (menuCurrentItem == counter++ || menuCurrentItem == counter++)
			{
				lcd.print( F( "Restore def." ) );
				//lcd.setCursor( 3, 1 );
				//lcd.print( F( "Load setting" ) );
			}

			//print selection arrow
			lcd.setCursor( 0, menuCurrentItem % 2 );
			lcd.print( F( "->" ) );

			displayUpdate = false;
		}


		if (buttonESC.state())
			return;
		else if (buttonUp.state())
		{
			menuCurrentItem--;
			if (menuCurrentItem < 0) menuCurrentItem = menuItemsCount;
			displayUpdate = 1;
		}
		else if (buttonDown.state())
		{
			menuCurrentItem++;
			if (menuCurrentItem > menuItemsCount) menuCurrentItem = 0;
			displayUpdate = 1;
		}
		else if (buttonOK.state())
		{
			lcd.clear();
			lcd.setCursor( 0, 0 );
			displayUpdate = true; //so display updates when we exit the sub-menu

			if (menuCurrentItem == 0)
			{
				calibratePosition();
				mode = 0;
				return;
			}
			else if (menuCurrentItem == 1)
			{
				lcd.print( F( "Gyro: " ) );
				lcd.setCursor( 0, 1 );
				lcd.print( F( "OK to reset" ) );

				while (1)
				{
					lcd.setCursor( 8, 0 );
					double gyroData = readSensorData();
					lcd.print( gyroData );


					if (buttonOK.state() == 1)
					{
						SETTINGS.POSITION_OFFSET = gyroData + SETTINGS.POSITION_OFFSET;
					}
					else if (buttonESC.state())
					{
						break;
					}
				}
			}
			else if (menuCurrentItem == 2)
			{
				lcd.print( F( "Max speed" ) );
				SETTINGS.DRIVER_MAX_SPEED = menuInner( SETTINGS.DRIVER_MAX_SPEED, 0, 4000, 1, 10 );
				stepper.setMaxSpeed( SETTINGS.DRIVER_MAX_SPEED );
				continue;

			}
			else if (menuCurrentItem == 3)
			{
				lcd.print( F( "Acceleration" ) );
				SETTINGS.DRIVER_MAX_ACC = menuInner( SETTINGS.DRIVER_MAX_ACC, 0, 4000, 1, 10 );
				stepper.setAcceleration( SETTINGS.DRIVER_MAX_ACC );
				continue;

			}
			else if (menuCurrentItem == 4)
			{
				lcd.print( F( "Driver current" ) );
				SETTINGS.DRIVER_CURRENT = menuInner( SETTINGS.DRIVER_CURRENT, 0, 2000, 1, 10 );
				TMCdriver.rms_current( SETTINGS.DRIVER_CURRENT );
				continue;

			}
			else if (menuCurrentItem == 5)
			{
				lcd.print( F( "Move threshold" ) );
				SETTINGS.MOVE_THRESHOLD = menuInner( SETTINGS.MOVE_THRESHOLD, 0, STEPS_LIMIT, 1 );
				continue;

			}
			else if (menuCurrentItem == 6)
			{
				lcd.print( F( "Center thr." ) );
				SETTINGS.MOVE_THRESHOLD_CENTER = menuInner( SETTINGS.MOVE_THRESHOLD_CENTER, 0, STEPS_LIMIT, 1 );
				continue;
			}
			else if (menuCurrentItem == 7)
			{
				lcd.print( F( "Rotation limit" ) );
				SETTINGS.STEPS_LIMIT_ALLOWED = menuInner( SETTINGS.STEPS_LIMIT_ALLOWED, 0, STEPS_LIMIT, 1 );
				continue;
			}
			else if (menuCurrentItem == 8)
			{
				//could use manuInner function but I want the brightness to update as
				//I change it.
				lcd.print( F( "Display" ) );
				lcd.setCursor( 0, 1 );
				lcd.print( F( "brightness" ) );
				int DISPLAY_BRIGHTNESS_local = SETTINGS.DISPLAY_BRIGHTNESS;
				int DISPLAY_BRIGHTNESS_local_displayed = -1;

				while (1)
				{
					if (DISPLAY_BRIGHTNESS_local_displayed != DISPLAY_BRIGHTNESS_local)
					{
						lcd.setCursor( 13, 0 );
						lcd.print( F( "   " ) );
						lcd.setCursor( 13, 0 );
						lcd.print( DISPLAY_BRIGHTNESS_local );
						DISPLAY_BRIGHTNESS_local_displayed = DISPLAY_BRIGHTNESS_local;
					}

					if (buttonUp.state())
					{
						DISPLAY_BRIGHTNESS_local++;
						if (DISPLAY_BRIGHTNESS_local > 255) DISPLAY_BRIGHTNESS_local = 255;
						analogWrite( LCD_BRIGHTNESS, DISPLAY_BRIGHTNESS_local );
					}
					else if (buttonDown.state())
					{
						DISPLAY_BRIGHTNESS_local--;
						if (DISPLAY_BRIGHTNESS_local < 0) DISPLAY_BRIGHTNESS_local = 0;
						analogWrite( LCD_BRIGHTNESS, DISPLAY_BRIGHTNESS_local );
					}
					else if (buttonESC.state())
					{
						analogWrite( LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS );
						break;
					}
					else if (buttonOK.state())
					{
						SETTINGS.DISPLAY_BRIGHTNESS = DISPLAY_BRIGHTNESS_local;
						break;
					}
				}
			}
			else if (menuCurrentItem == 9)
			{
				lcd.print( F( "Auto calib." ) );
				SETTINGS.AUTO_POSITION_OFFSET = menuInner( SETTINGS.AUTO_POSITION_OFFSET, 0, 1, 1 );
				continue;
			}
			else if (menuCurrentItem == 10)
			{
				eeprom_write_block( (const void*) &SETTINGS, (void*) 0, sizeof( SETTINGS ) );
				continue;
			}
			else if (menuCurrentItem == 11)
			{
				eeprom_read_block( (void*) &SETTINGS, (void*) 0, sizeof( SETTINGS ) );

				TMCdriver.rms_current( SETTINGS.DRIVER_CURRENT );
				stepper.setMaxSpeed( SETTINGS.DRIVER_MAX_SPEED );
				stepper.setAcceleration( SETTINGS.DRIVER_MAX_ACC );
				analogWrite( LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS );
				continue;
			}
			else if (menuCurrentItem == 12)
			{
				SETTINGS = DEFAULT_SETTINGS;
				TMCdriver.rms_current( SETTINGS.DRIVER_CURRENT );
				stepper.setMaxSpeed( SETTINGS.DRIVER_MAX_SPEED );
				stepper.setAcceleration( SETTINGS.DRIVER_MAX_ACC );
				analogWrite( LCD_BRIGHTNESS, SETTINGS.DISPLAY_BRIGHTNESS );
				eeprom_write_block( (const void*) &SETTINGS, (void*) 0, sizeof( SETTINGS ) );
				continue;
			}
			else
			{
#ifdef debug
				Serial.println( F( "Trying to enter unknown menu item !" ) );
#endif
				//restart menu
				menu();
				return;
			}

			displayUpdate = 1;
		}

	}
}


/*
	Is stepper.distanceToGo() > 0 then only gyro and stepper.run part is running.
	Display and user input not checked to make stepper run function have less delay.
*/
void loop()
{
	static unsigned long gyroUpdate = 0;
	static double gyroVal = 0;
	static int previousTarget = 0;


	/*
		Update the gyro value.
	*/
	if (millis() - gyroUpdate > SETTINGS.GYRO_UPDATE_TIME)
	{
		gyroVal = readSensorData();

		if (SETTINGS.AUTO_POSITION_OFFSET)
		{
			if (gyroVal > 0) SETTINGS.POSITION_OFFSET += 0.001;
			else SETTINGS.POSITION_OFFSET -= 0.001;
		}


		gyroUpdate = millis();
	}

	/*
		Gyro motor control.
	*/
	if (mode == 1)
	{
		//gyroVal is in degrees.
		int newTarget = gyroVal * 5;	//gyroVal / 90*450 as gyroVal is degrees of center and it takes 450 steps to rotate 90 degrees

		if (abs( newTarget ) < SETTINGS.MOVE_THRESHOLD_CENTER)
		{
			newTarget = 0;
			previousTarget = 0;
			stepper.moveTo( 0 );
		}
		else if (abs( newTarget - previousTarget ) > SETTINGS.MOVE_THRESHOLD)
		{
			if (newTarget > SETTINGS.STEPS_LIMIT_ALLOWED) newTarget = SETTINGS.STEPS_LIMIT_ALLOWED;
			else if (newTarget < -SETTINGS.STEPS_LIMIT_ALLOWED) newTarget = -SETTINGS.STEPS_LIMIT_ALLOWED;

			stepper.moveTo( newTarget );

			previousTarget = newTarget;
		}
	}

	stepper.run();

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
			mode = !mode;

			if (!mode)
			{
				stepper.moveTo( 0 );
				previousTarget = 0;
			}
		}
		break;

		//if button is held down
	case 2:
		int modeSave = mode;

		calibratePosition();

		if (modeSave == 1)
			mode = 1;
		else
			mode = 0;

		break;
	}

	//if stepper not in target position don't run rest of the function.
	//less important parts after this step
	if (stepper.distanceToGo())
	{
		return;
	}

	//allow to go to the menu only if gyro is not running
	//as if gyro is running we don't reach this part of the program
	if (buttonESC.state())
	{
		//if we were calibrated and possibly running, go back to the center before entering the menu
		if (mode != -1)
			stepper.moveTo( 0 );
		while (stepper.distanceToGo())
			stepper.run();

		stepper.disableOutputs();
		mode = -1;
		menu();
		return;
	}
	else if (buttonDown.state() == 2)
	{
		mode = -1;
		stepper.disableOutputs();
	}


	/*
		Update the display.
	*/
	static unsigned long displayUpdate = 0;
	if (millis() - displayUpdate > 300)
	{
		lcd.clear();
		lcd.setCursor( 0, 0 );

		lcd.print( F( "Gyro: " ) );
		lcd.setCursor( 0, 1 );
		if (mode == 0)
			lcd.print( F( "OFF" ) );
		else if (mode == 1)
			lcd.print( F( "ON" ) );
		else if (mode == -1)
		{
			static byte msg_counter = 0;

			if (msg_counter < 3)
				lcd.print( F( "press OK" ) );
			else if (msg_counter < 6)
				lcd.print( F( "to" ) );
			else
				lcd.print( F( "calibrate" ) );

			msg_counter++;
			if (msg_counter > 9) msg_counter = 0;
		}
		else
			lcd.print( F( "err" ) );

		float volts = voltMeter();
		if (volts >= 10)
			lcd.setCursor( 10, 1 );
		else
			lcd.setCursor( 11, 1 );
		lcd.print( volts );
		lcd.print( F( "V" ) );

		int printOffset = 0;
		if (gyroVal < 0) printOffset--;
		if (abs( gyroVal ) >= 10) printOffset--;
		if (abs( gyroVal ) >= 100) printOffset--;

		lcd.setCursor( printOffset + 12, 0 );
		lcd.print( gyroVal );

		displayUpdate = millis();
	}
}









