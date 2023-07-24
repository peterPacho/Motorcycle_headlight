/*
	Gyro portion is heavly based on Kristian Lauszus, TKJ Electronics work.
*/

#include <Wire.h>
#include <TMCStepper.h>         // https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>		// https://github.com/teemuatlut/TMCStepper/blob/master/examples/TMC_AccelStepper/TMC_AccelStepper.ino
#include <LiquidCrystal_I2C.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

/*
	Arduino pin-out. SDA - A4, SCL - A5
*/
#define DRIVER_ENABLE 9
#define DRIVER_DIRECTION 10
#define DRIVER_STEP 3
#define DRIVER_SWCLCK 4
#define DRIVER_TX 6	//TX and RX must be swapped around
#define DRIVER_RX 5
#define HALL_SENSOR 11
#define ENCODER_BUTTON 7
#define ENCODER_CLK 2
#define ENCODER_DT 8
#define VOLTAGE_SENSE A0

#define DRIVER_MAX_SPEED 1200
#define DRIVER_MAX_ACC 2500
#define DRIVER_CURRENT 1500

#define DRIVER_ADDRESS 0b00
#define DRIVER_RSENSE 0.11f
#define MPU 0x68

/*
	Speeds to use for calibration function.
*/
#define DRIVER_MAX_SPEED_CALIBRATION 400
#define DRIVER_MAX_ACC_CALIBRATION 300

#define MOVE_THRESHOLD 5		//how many steps off "correct" position before is starts moving
#define MOVE_THRESHOLD_CENTER 35 //how many steps off center before it starts moving

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

SoftwareSerial SoftSerial( DRIVER_RX, DRIVER_TX );
TMC2209Stepper TMCdriver( &SoftSerial, DRIVER_RSENSE, DRIVER_ADDRESS );
AccelStepper stepper = AccelStepper( stepper.DRIVER, DRIVER_STEP, DRIVER_DIRECTION );
LiquidCrystal_I2C lcd( 0x27, 16, 2 );

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX;
int16_t tempRaw;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


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
button encoderButton( ENCODER_BUTTON );

/*
	Found this somewhere. Don't know who is the original author.
*/
void findDevices()
{
	byte err, adr;       /*variable error is defined with address of I2C*/
	int number_of_devices;
	Serial.println( "Scanning." );
	number_of_devices = 0;
	for (adr = 1; adr < 127; adr++)
	{
		Wire.beginTransmission( adr );
		err = Wire.endTransmission();

		if (err == 0)
		{
			Serial.print( "I2C device at address 0x" );
			if (adr < 16)
				Serial.print( "0" );
			Serial.print( adr, HEX );
			Serial.println( "  !" );
			number_of_devices++;
		}
		else if (err == 4)
		{
			Serial.print( "Unknown error at address 0x" );
			if (adr < 16)
				Serial.print( "0" );
			Serial.println( adr, HEX );
		}
	}
	if (number_of_devices == 0)
		Serial.println( "No I2C devices attached\n" );
	else
		Serial.println( "done\n" );
}


/*
	Finds the center / calibrates the stepper
	using the hall sensor.
*/
void calibratePosition()
{
	//setup
	lcd.clear();
	lcd.setCursor( 0, 0 );
	lcd.print( "Calibrating..." );
	if (digitalRead( DRIVER_ENABLE ))
	{
		stepper.enableOutputs();
		delay( 300 );
	}
	stepper.setMaxSpeed( DRIVER_MAX_SPEED_CALIBRATION );
	stepper.setAcceleration( DRIVER_MAX_ACC_CALIBRATION );


	//if already calibrated before, move to 0 to reduce time
	if (stepper.currentPosition())
	{
		stepper.moveTo( 0 );
		while (stepper.distanceToGo()) stepper.run();
		//if moved to 0 but still didn't reach HALL (due to some missed steps??) go a bit further
		if (digitalRead( HALL_SENSOR ))
			stepper.moveTo( STEPS_PER_REVOLUTION * 0.04 );
		while (stepper.distanceToGo()) stepper.run();
	}

	stepper.move( -STEPS_PER_REVOLUTION * 0.22 );

	while (stepper.distanceToGo() != 0 && digitalRead( HALL_SENSOR ))
	{
		stepper.run();
	}

	stepper.setCurrentPosition( 0 );

	//	if hall still reads high, we must be past it (and stalled on the end stop), so rotate back to the middle
	if (digitalRead( HALL_SENSOR ))
	{
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

	stepper.move( -STEPS_PER_REVOLUTION * 0.04 );

	while (stepper.distanceToGo() != 0 && !digitalRead( HALL_SENSOR ))
	{
		stepper.run();
	}

	//reached some point where hall no longer reads 0
	//reset position to 0
	//move in other direction until hall reads 0 and then reads 1 again

	stepper.setCurrentPosition( 0 );
	stepper.move( STEPS_PER_REVOLUTION * 0.04 );

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
	stepper.setMaxSpeed( DRIVER_MAX_SPEED );
	stepper.setAcceleration( DRIVER_MAX_ACC );
	lcd.clear();
}



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
				Serial.println( "Stepper disabled" );
			}
			else
			{
				Serial.print( "Speed set to " );
				Serial.println( speed );
				stepper.enableOutputs();
				stepper.setMaxSpeed( speed );
			}
			break;
		}

		case 'p':
		{
			int position = data.substring( 1 ).toInt();
			Serial.print( "Moving stepper to position " );
			Serial.println( position );
			stepper.moveTo( position );
			break;
		}

		case 'a':
		{
			int speed = data.substring( 1 ).toInt();

			Serial.print( "Acceleration set to " );
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
			Serial.println( "Stepper disabled." );
			stepper.disableOutputs();
			break;
		}

		default:
			Serial.println( "Unknown command!" );
		}
	}
}

/*
	More of Kristian Lauszus code.
*/
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
uint8_t i2cWrite( uint8_t registerAddress, uint8_t data, bool sendStop )
{
	return i2cWrite( registerAddress, &data, 1, sendStop ); // Returns 0 on success
}
uint8_t i2cWrite( uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop )
{
	Wire.beginTransmission( IMUAddress );
	Wire.write( registerAddress );
	Wire.write( data, length );
	uint8_t rcode = Wire.endTransmission( sendStop ); // Returns 0 on success
	if (rcode)
	{
		Serial.print( F( "i2cWrite failed: " ) );
		Serial.println( rcode );
	}
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}
uint8_t i2cRead( uint8_t registerAddress, uint8_t* data, uint8_t nbytes )
{
	uint32_t timeOutTimer;
	Wire.beginTransmission( IMUAddress );
	Wire.write( registerAddress );
	uint8_t rcode = Wire.endTransmission( false ); // Don't release the bus
	if (rcode)
	{
		Serial.print( F( "i2cRead failed: " ) );
		Serial.println( rcode );
		return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
	}
	Wire.requestFrom( IMUAddress, nbytes, (uint8_t) true ); // Send a repeated start and then release the bus after reading
	for (uint8_t i = 0; i < nbytes; i++)
	{
		if (Wire.available())
			data[i] = Wire.read();
		else
		{
			timeOutTimer = micros();
			while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
			if (Wire.available())
				data[i] = Wire.read();
			else
			{
				Serial.println( F( "i2cRead timeout" ) );
				return 5; // This error value is not already taken by endTransmission
			}
		}
	}
	return 0; // Success
}


/*
	Kristian Lauszus work
*/
double readSensorData()
{
	while (i2cRead( 0x3B, i2cData, 10 ));
	accX = ((i2cData[0] << 8) | i2cData[1]);
	accY = ((i2cData[2] << 8) | i2cData[3]);
	accZ = ((i2cData[4] << 8) | i2cData[5]);
	tempRaw = (i2cData[6] << 8) | i2cData[7];
	gyroX = (i2cData[8] << 8) | i2cData[9];

	double dt = (double) (micros() - timer) / 1000000; // Calculate delta time
	timer = micros();

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
	double roll = atan2( accY, accZ ) * RAD_TO_DEG;
	double pitch = atan( -accX / sqrt( accY * accY + accZ * accZ ) ) * RAD_TO_DEG;

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
	{
		kalmanX.setAngle( roll );
		kalAngleX = roll;
		gyroXangle = roll;
	}
	else
		kalAngleX = kalmanX.getAngle( roll, gyroXrate, dt ); // Calculate the angle using a Kalman filter

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;

	return kalAngleX;
}


void setup()
{
	Serial.begin( 9600 );
	SoftSerial.begin( 9600 );
	TMCdriver.beginSerial( 9600 );


	/*
		https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
	*/
	Wire.begin();

	digitalWrite( DRIVER_ENABLE, HIGH );
	pinMode( DRIVER_ENABLE, OUTPUT );
	pinMode( DRIVER_DIRECTION, OUTPUT );
	pinMode( DRIVER_STEP, OUTPUT );
	pinMode( HALL_SENSOR, INPUT_PULLUP );
	pinMode( ENCODER_BUTTON, INPUT );
	pinMode( ENCODER_CLK, INPUT );
	pinMode( ENCODER_DT, INPUT );
	pinMode( VOLTAGE_SENSE, INPUT );

	TMCdriver.begin();
	TMCdriver.rms_current( DRIVER_CURRENT );
	TMCdriver.pwm_autoscale( 1 );
	TMCdriver.microsteps( DRIVER_MICROSTEPS );
	stepper.setMaxSpeed( DRIVER_MAX_SPEED );
	stepper.setAcceleration( DRIVER_MAX_ACC );
	stepper.setEnablePin( DRIVER_ENABLE );
	stepper.setPinsInverted( false, false, true );

	lcd.begin();
	lcd.clear();

	/*
		Setup copied from Kristian Lauszus's work.
	*/
	TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

	i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
	while (i2cWrite( 0x19, i2cData, 4, false )); // Write to all four registers at once
	while (i2cWrite( 0x6B, 0x01, true )); // PLL with X axis gyroscope reference and disable sleep mode

	while (i2cRead( 0x75, i2cData, 1 ));
	if (i2cData[0] != 0x68)
	{ // Read "WHO_AM_I" register
		Serial.print( F( "Error reading sensor" ) );
		while (1);
	}

	delay( 100 ); // Wait for sensor to stabilize

	/* Set kalman and gyro starting angle */
	while (i2cRead( 0x3B, i2cData, 6 ));
	accX = (i2cData[0] << 8) | i2cData[1];
	accY = (i2cData[2] << 8) | i2cData[3];
	accZ = (i2cData[4] << 8) | i2cData[5];

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
	double roll = atan2( accY, accZ ) * RAD_TO_DEG;
	double pitch = atan( -accX / sqrt( accY * accY + accZ * accZ ) ) * RAD_TO_DEG;


	kalmanX.setAngle( roll ); // Set starting angle
	kalmanY.setAngle( pitch );
	gyroXangle = roll;
	gyroYangle = pitch;

	timer = micros();

	calibratePosition();
}


/*
	Is stepper.distanceToGo() > 0 then only gyro and stepper.run part is running.
	Display and user input not checked to make stepper run function have less delay.

*/
void loop()
{
	static bool enabled = false;
	static unsigned long gyroUpdate = 0;
	static double gyroVal = 0;

	/*
		Update the gyro value.
	*/
	if (millis() - gyroUpdate > 100)
	{
		gyroVal = readSensorData();

		if (gyroVal > 0)
			gyroVal -= 180;
		else
			gyroVal += 180;


		gyroUpdate = millis();
	}

	/*
		Gyro motor control.
	*/
	if (enabled)
	{
		static int previousTarget = 0;

		//gyroVal is in degrees.
		int newTarget = gyroVal * 5;	//gyroVal / 90*450 as gyroVal is degrees of center and it takes 450 steps to rotate 90 degrees

		if (abs(newTarget) < MOVE_THRESHOLD_CENTER)
		{
			newTarget = 0;
			previousTarget = 0;

			stepper.moveTo( 0 );
		}
		else if (abs( newTarget - previousTarget ) > MOVE_THRESHOLD)
		{
			if (newTarget > STEPS_LIMIT) newTarget = STEPS_LIMIT;
			else if (newTarget < -STEPS_LIMIT) newTarget = -STEPS_LIMIT;

			stepper.moveTo( newTarget );

			previousTarget = newTarget;
		}
	}

	stepper.run();


	/*
		Check for user input.
	*/
	if (encoderButton.state() == 1)
	{
		enabled = !enabled;

		if (!enabled)
		{
			stepper.moveTo( 0 );
		}
	}


	//if stepper not in target position don't run rest of the function.
	if (stepper.distanceToGo())
	{
		return;
	}

	if (encoderButton.state() == 2)
		calibratePosition();


	serialCommands();

	/*
		Update the display.
	*/
	static unsigned long displayUpdate = 0;
	if (millis() - displayUpdate > 300)
	{
		lcd.clear();
		lcd.setCursor( 0, 0 );
		lcd.print( "Gyro: " );
		lcd.print( gyroVal );

		lcd.setCursor( 0, 1 );
		lcd.print( "T: " );
		lcd.print( stepper.currentPosition() );

		displayUpdate = millis();
	}
}









