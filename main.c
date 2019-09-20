/*
=========================================================================================================================================================
	 ____  ___  ____    _____                ____               _                _  _             
	|  _ \|_ _||  _ \  |  ___|__ _  _ __    / ___| ___   _ __  | |_  _ __  ___  | || |  ___  _ __ 
	| |_) || | | | | | | |_  / _` || '_ \  | |    / _ \ | '_ \ | __|| '__|/ _ \ | || | / _ \| '__|
	|  __/ | | | |_| | |  _|| (_| || | | | | |___| (_) || | | || |_ | |  | (_) || || ||  __/| |   
	|_|   |___||____/  |_|   \__,_||_| |_|  \____|\___/ |_| |_| \__||_|   \___/ |_||_| \___||_|  


	This software control system provides a user with methods of controlling a variable-speed DC fan via an FPGA.
		- The controller drives the fan using a PWM signal, and measures its speed via its tachometer signal.
		- Open-loop or closed-loop PID feedback control can be toggled between via a switch.
		- The target speed / duty cycle can be adjusted via a rotary encoder.
		- System parameters (e.g. fan speed, controller error) are displayed on a set of seven-segment displays,
		  and toggled between using push buttons.
	
	This is the code submission for EE30186 coursework: Fan Controller. EE30186 is the Integrated Engineering
	module in the Department of Electronic & Electrical Engineering at the University of Bath.
		- Note that this code requires additional resources (such as "EE30186.h") to compile. As these are property 
		  of the University of Bath, their inclusion in this GitHub upload has been excluded. Therefore, the purpose
		  of this upload is solely to illustrate the controller code.

	Version: 		1.0
	Last updated: 	13-Dec-2018
	Author:			Matt Cotton
 */

//=======================================================================================================================================================
//------------------------------------------------------------USER SETTINGS------------------------------------------------------------------------------
//=======================================================================================================================================================

#define K_PROP 					5.0		// Proportional error gain
#define K_INT 					4.0		// Integral error gain
#define K_DERIV 				0.2		// Derivative error gain
#define K_DERIV_DEADZONE 		100		// Derivative error dead-zone, RPM/s

#define TACH_FILTER_TIME 		0.004	// Required amount of time for tachometer signal to remain stable after an edge to register, s
#define TACH_SMOOTHING_FACTOR 	10		// Maximum # of speed measurements to average across when calculating fan speed

#define PWM_FREQ 				300		// PWM frequency, Hz
#define SWITCH_REFRESH_FREQ 	10		// Switch read frequency, Hz
#define BUTTON_REFRESH_FREQ 	20		// Button read frequency, Hz
#define ROTARY_REFRESH_FREQ 	150		// Rotary encoder read frequency, Hz
#define SPEED_REFRESH_FREQ		10		// Speed measurement and closed-loop controller update frequency, Hz
#define SEVENSD_REFRESH_FREQ 	10		// 7-segment display update frequency, Hz

#define MESSAGE_DISPLAY_TIME_S  0.3		// Display time for system messages, s

#define TARGET_MIN 				500		// Minimum target speed, RPM
#define TARGET_MAX 				2500	// Maximum target speed, RPM
#define TARGET_INITIAL 			2000	// Target speed at switch-on, RPM
#define TARGET_INCREMENT_STEP 	25		// Target speed increase per 1/4 increment of rotary encoder, RPM

#define DUTY_MIN_OPEN			0		// Minimum Duty Cycle, % (open-loop)
#define DUTY_MIN_CLOSED			1		// Minimum Duty Cycle, % (closed-loop)
#define DUTY_MAX 				100		// Maximum Duty Cycle, %
#define DUTY_INITIAL 			50		// Duty Cycle at switch-on, %
#define DUTY_INCREMENT_STEP 	1		// Duty Cycle increase per 1/4 increment of rotary encoder, %

//=======================================================================================================================================================
//------------------------------------------------------------SYSTEM SETTINGS----------------------------------------------------------------------------
//=======================================================================================================================================================

//System settings
#define CLOCK_SPEED 			50000000 	// 50MHz clock
#define CYCLES_IN_60_SECS 		3000000000 	// 50MHz * 60s
#define TACH_WAVES_PER_REV		2			// 2 tachometer square-waves per fan revolution
#define DUTY_RESOLUTION 		10000 		// 10000 levels of duty available, i.e. 0.01% steps

// GPIO port pin definitions
#define TACH_PIN 				1
#define PWM_PIN 				3
#define ROTENC_PIN_A 			17
#define ROTENC_PIN_B 			19

// Toggle switch definitions
#define POWER_SWITCH 			9
#define CLOSED_LOOP_SWITCH 		0

// Button value definitions
#define key0 					0xE
#define key1 					0xD
#define key2 					0xB
#define key3 					0x7

//=======================================================================================================================================================
//------------------------------------------------------------SYSTEM CALCULATIONS------------------------------------------------------------------------
//=======================================================================================================================================================

#define DUTY_S_MIN_OPEN			( DUTY_RESOLUTION * DUTY_MIN_OPEN / 100 )		// Duty cycle %'s scaled to the duty resolution
#define DUTY_S_MIN_CLOSED		( DUTY_RESOLUTION * DUTY_MIN_CLOSED / 100 )
#define DUTY_S_MAX 				( DUTY_RESOLUTION * DUTY_MAX / 100 )
#define DUTY_S_INITIAL			( DUTY_RESOLUTION * DUTY_INITIAL / 100 )
#define DUTY_S_INCREMENT_STEP 	( DUTY_RESOLUTION * DUTY_INCREMENT_STEP / 100 )

#define SWITCH_REFRESH_TIME 	( CLOCK_SPEED / SWITCH_REFRESH_FREQ )			// Frequencies converted to # of clock cycles per period
#define BUTTON_REFRESH_TIME 	( CLOCK_SPEED / BUTTON_REFRESH_FREQ )
#define ROTARY_REFRESH_TIME 	( CLOCK_SPEED / ROTARY_REFRESH_FREQ )
#define SPEED_REFRESH_TIME 		( CLOCK_SPEED / SPEED_REFRESH_FREQ )
#define SEVENSD_REFRESH_TIME 	( CLOCK_SPEED / SEVENSD_REFRESH_FREQ )
#define SPEED_CALC_TIME			( CLOCK_SPEED / SPEED_CALC_FREQ )
#define PWM_CYCLE_TIME 			( CLOCK_SPEED / PWM_FREQ )

#define MESSAGE_DISPLAY_TIME 	( CLOCK_SPEED * MESSAGE_DISPLAY_TIME_S )		// Times converted to # of clock cycles
#define TACH_FILTER_THRESHOLD	( CLOCK_SPEED * TACH_FILTER_TIME )

//=======================================================================================================================================================
//---------------------------------------------------------------------DEFINITIONS-----------------------------------------------------------------------
//=======================================================================================================================================================

// Libraries used
#include "EE30186.h"
#include "system.h"
#include "socal/socal.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

//Enumerations and custom types
typedef enum {FALSE = 0, TRUE = 1} Bool;
typedef enum {NA, CW, CCW} Direction;
typedef enum {DUTY, SPEED, TARGET, ERROR, NONE} DisplayOption;
typedef enum {FCN_SWITCHES = 0, FCN_BUTTONS = 1, FCN_ROTARYENCODER = 2, FCN_SPEED = 3, FCN_7SD = 4} Function;
typedef enum {MSG_OFF, MSG_OPEN, MSG_CLOSED, MSG_SPEED, MSG_TARGET, MSG_DUTY, MSG_ERROR, MSG_DASHES, MSG_BLANK} Message;

// I/O pointers (defined in system.h)
volatile int * LEDs     		= (volatile int *)ALT_LWFPGA_LED_BASE;
volatile int * Switches 		= (volatile int *)ALT_LWFPGA_SWITCH_BASE;
volatile int * Buttons 			= (volatile int *)ALT_LWFPGA_KEY_BASE;
volatile int * GpioPort			= (volatile int *)ALT_LWFPGA_GPIO_1A_BASE;
volatile int * HexA 			= (volatile int *)ALT_LWFPGA_HEXA_BASE;
volatile int * HexB 			= (volatile int *)ALT_LWFPGA_HEXB_BASE;
volatile unsigned int * Clock 	= (volatile unsigned int *)ALT_LWFPGA_COUNTER_BASE;

// Global variables
Bool 			PowerEnabled, ClosedLoopEnabled, MessageBeingDisplayed;
DisplayOption 	SelectedDisplay;
int 		 	SpeedError;
unsigned int 	TargetSpeed, FanSpeed, Duty, PWMHighPeriod, MessageDisplayTimestamp, SpeedUpdatedTimestamp,
				NumOfSpeedMeasurements, MeasuredSpeedIndex, MeasuredSpeeds[TACH_SMOOTHING_FACTOR];

//=======================================================================================================================================================
//------------------------------------------------------------INPUT READ FUNCTIONS-----------------------------------------------------------------------
//=======================================================================================================================================================

/* ReadSwitch function:
 * - Gets the digital value of the given switch number.
 */
Bool ReadSwitch(unsigned int SwitchNumber)
{
	return 0x1 & (*Switches >> SwitchNumber);
}

//=======================================================================================================================================================

/* ReadPin function:
 * - Gets the digital value of the given GPIO port pin number.
 */
Bool ReadPin(unsigned int PinNumber)
{
	return 0x1 & (*GpioPort >> PinNumber);
}

//=======================================================================================================================================================
//------------------------------------------------------------SET INPUT FUNCTIONS------------------------------------------------------------------------
//=======================================================================================================================================================

/* SetDuty function:
 * - Updates the Duty Cycle, ensuring it stays within the defined limits.
 * - Recalculates the on-time for the PWM signal.
 */
void SetDuty(int NewDuty)
{
	if 		 (NewDuty > DUTY_S_MAX) 								Duty = DUTY_S_MAX;			// Constrain Duty Cycle within its limits
	else if ((NewDuty < DUTY_S_MIN_OPEN)   && !ClosedLoopEnabled)	Duty = DUTY_S_MIN_OPEN;
	else if ((NewDuty < DUTY_S_MIN_CLOSED) &&  ClosedLoopEnabled)	Duty = DUTY_S_MIN_CLOSED;
	else 															Duty = NewDuty;

	PWMHighPeriod = PWM_CYCLE_TIME / DUTY_RESOLUTION * Duty;	// Recalculate the on-time for the PWM signal
}

//=======================================================================================================================================================

/* SetTargetSpeed function:
 * - Updates the Target Speed, ensuring it stays within the defined limits.
 */
void SetTargetSpeed(int NewTarget)
{
	if 		(NewTarget > TARGET_MAX)	TargetSpeed = TARGET_MAX;		// Constrain target speed within its limits
	else if (NewTarget < TARGET_MIN)	TargetSpeed = TARGET_MIN;
	else 					 			TargetSpeed = NewTarget;
}

//=======================================================================================================================================================
//------------------------------------------------------------CLOSED-LOOP FEEDBACK FUNCTION--------------------------------------------------------------
//=======================================================================================================================================================

/* ComputeFeedback function:
 * - Linearises the system (assuming a quadratic relationship between Duty Cycle and fan speed) to scale the feedback gain.
 * - Applies closed-loop feedback control using a Proportional-Integral-Derivative controller.
 */
void ComputeFeedback()
{
	static int ErrorInt = DUTY_S_INITIAL;
	static unsigned int PrevSpeed;
	int ErrorDeriv;

	SpeedError = TargetSpeed - FanSpeed;	// Recalculate speed error

	float LinearisationFactor;				// System linearisation

	if (TargetSpeed <= 800) LinearisationFactor = 0.0178;
	else					LinearisationFactor = 0.00002225 * TargetSpeed;

	/* Note: System is non-linear (Duty cycle vs. RPM approximates a quadratic relationship at high speed, linear at low speed).
	 * To improve closed-loop control performance, the closed-loop gain is scaled depending on the target speed.
	 * To achieve this, measurements were taken and a graph of duty cycle vs. fan speed was plotted.
	 * A 2nd order polynomial curve was fit to the higher end of the data, a linear curve fit to the lower.
	 * LinearisationFactor is the gradient of these curves.
	 */

	float GainNormalisation = LinearisationFactor * (DUTY_RESOLUTION / 100); 	// Normalise gain to duty resolution

	int ErrorProp = (int)(K_PROP * GainNormalisation * SpeedError);				// Scale proportional gain

	ErrorInt += (int)(K_INT * GainNormalisation * SpeedError / SPEED_REFRESH_FREQ);	// Update integral gain

	if 		(ErrorInt > DUTY_S_MAX)			ErrorInt = DUTY_S_MAX;				// Prevent integral wind-up
	else if (ErrorInt < DUTY_S_MIN_CLOSED)	ErrorInt = DUTY_S_MIN_CLOSED;

	int SpeedChange = (PrevSpeed - FanSpeed) * SPEED_REFRESH_FREQ; 				// Fan acceleration, units RPM/s

	if ( abs(SpeedChange) > K_DERIV_DEADZONE)									// Dead-zone in derivative error gain
		ErrorDeriv = (int)(K_DERIV * GainNormalisation * SpeedChange);			// Scale derivative gain
	else
		ErrorDeriv = 0;

	SetDuty( ErrorInt + ErrorProp + ErrorDeriv );		// Update the Duty Cycle

	PrevSpeed = FanSpeed;
}

//=======================================================================================================================================================
//------------------------------------------------------------TACHOMETER FUNCTIONS------------------------------------------------------------------------
//=======================================================================================================================================================

/* LogSpeedMeasurement function:
 * - Saves the new speed measurement in an array, overwriting the oldest measurement.
 */
void LogSpeedMeasurement(unsigned int NewSpeed)
{
	MeasuredSpeeds[MeasuredSpeedIndex] = NewSpeed; 		// Overwrite the oldest speed measurement with the new value

	SpeedUpdatedTimestamp = *Clock;						// Log the time-stamp when the measurement occurred

	if (NumOfSpeedMeasurements < TACH_SMOOTHING_FACTOR) NumOfSpeedMeasurements++;	// Cap the # of measurements at the max

	MeasuredSpeedIndex++;						 		// Move to the next array position, ready for the next measurement
	MeasuredSpeedIndex %= TACH_SMOOTHING_FACTOR; 		// Jump back to the start of the array if required

}

//=======================================================================================================================================================

/* CalculateFanSpeed function:
 * - Smoothes the measured fan speed by taking an average of the speed measurements made in the last 0.1s.
 * - Detects if the fan has stopped spinning.
 */
void CalculateFanSpeed()
{
	if (NumOfSpeedMeasurements <= 0)								// No new measurements were logged in the refresh period
	{
		unsigned int TimeElapsed = *Clock - SpeedUpdatedTimestamp;	// Measure the time elapsed since a measurement was logged

		if (TimeElapsed > (CLOCK_SPEED/2)) FanSpeed = 0;			// No new measurements were logged in 0.5s, set speed to 0
	}
	else
	{																// New measurements logged, we need to average them
		int sum = 0;

		for (int i = 0; i < NumOfSpeedMeasurements; i++)			// Sum all speed values
		{
			sum += MeasuredSpeeds[i];
		}
		FanSpeed = sum / NumOfSpeedMeasurements;					// Divide by the number of measurements to get average speed

		NumOfSpeedMeasurements = 0;									// Reset measurement counters
		MeasuredSpeedIndex = 0;
	}

	if (ClosedLoopEnabled) ComputeFeedback();						// Update closed-loop controller in response to this new speed
}

//=======================================================================================================================================================

/* ReadTachometer function:
 * Measures the fan speed:
 * - Detects positive edges in the tachometer signal.
 * - Filters out invalid edges.
 * - Measures the time elapsed since the positive edge one revolution ago.
 * - Converts this to an RPM value.
 */
void ReadTachometer()
{
	static Bool PrevTachHigh = FALSE,
				FilterEnabled = FALSE;

	static unsigned int PosEdgeTimestamp,
						PrevPosEdgeTimestamp[TACH_WAVES_PER_REV],
						EdgeNum = 0;


	Bool TachHigh = ReadPin(TACH_PIN);

	if ( TachHigh && !PrevTachHigh ) 	// Positive edge detected
	{
		PosEdgeTimestamp = *Clock;		// Log the time-stamp when the edge occurred
		FilterEnabled = TRUE;
	}

	if (FilterEnabled)					// Filtering mode
	{
		if (TachHigh)					 //The high signal is still stable
		{
			unsigned int TimeElapsed = *Clock - PosEdgeTimestamp;	//Calculate the time passed since the edge

			if (TimeElapsed >= TACH_FILTER_THRESHOLD) 		//Signal remained high for sufficient time, the edge was real
			{
				FilterEnabled = FALSE;						// Exit filtering mode

				EdgeNum++;									// Move to the next edge index
				EdgeNum %= TACH_WAVES_PER_REV;				// Wrap back to 0 if required

				unsigned int FanPeriod = PosEdgeTimestamp - PrevPosEdgeTimestamp[EdgeNum];	// Time elapsed since the positive edge one revolution ago
				unsigned int MeasuredSpeed = CYCLES_IN_60_SECS / FanPeriod;					// Convert to RPM
				LogSpeedMeasurement(MeasuredSpeed);

				PrevPosEdgeTimestamp[EdgeNum] = PosEdgeTimestamp;							// Store the edge time-stamp for comparison to the next
			}
		}
		else FilterEnabled = FALSE;		// Signal didn't remain stable, discard this edge
	}
	PrevTachHigh = TachHigh;
}

//=======================================================================================================================================================
//------------------------------------------------------------PWM MODULATION FUNCTION--------------------------------------------------------------------
//=======================================================================================================================================================

/* Modulation function:
 * Generates the PWM signal.
 */
void Modulation()
{
	static unsigned int PWMStartTime = 0;

	if (PowerEnabled)
	{
		unsigned int TimeSinceRise = *Clock - PWMStartTime;					// Calculate the time elapsed since the rising edge of the PWM signal

		while (TimeSinceRise >= PWM_CYCLE_TIME) 							// We are onto the next PWM cycle (while loop to prevent wind-up)
		{
			PWMStartTime  += PWM_CYCLE_TIME;								// Shift the PWM rising edge time-stamp forward one PWM period
			TimeSinceRise -= PWM_CYCLE_TIME;								// Update the time elapsed to reflect this change
		}

		if (TimeSinceRise >= PWMHighPeriod) *GpioPort &= ~(1 << PWM_PIN); 	//We are in the low excursion of the cycle, so set PWM low
		else 							    *GpioPort |=  (1 << PWM_PIN); 	//We are in the high excursion of the cycle, so set PWM high
	}
	else
		*GpioPort &= ~(1 << PWM_PIN);										// Power switch is off, set PWM low
}

//=======================================================================================================================================================
//------------------------------------------------------------LED FUNCTIONS------------------------------------------------------------------------------
//=======================================================================================================================================================

int RoundToOneTenth(int Value, int MaxValue)
{
	int Percentage = 100 * Value / MaxValue;		// Convert to %

	int ReturnValue = Percentage / 10;				// Truncate to nearest 10%

	if ( (Percentage % 10) >= 5 ) ReturnValue++;	// Increase if round-up was required

	return ReturnValue;
}

//=======================================================================================================================================================

/* WriteLEDs function:
 * - Displays the value of selected system parameter as a fraction out of 10 on the LEDs.
 */
void WriteLEDs(DisplayOption SelectedDisplay)
{
	int LEDsLit;

	switch (SelectedDisplay)			// Calculate the number of LEDs to light up
	{
	case DUTY:		LEDsLit = RoundToOneTenth(Duty, DUTY_RESOLUTION); 		break;
	case SPEED:		LEDsLit = RoundToOneTenth(FanSpeed, TARGET_MAX); 		break;
	case TARGET:	LEDsLit = RoundToOneTenth(TargetSpeed, TARGET_MAX); 	break;
	case ERROR:		LEDsLit = RoundToOneTenth(abs(SpeedError), 400); 		break;
	default:		LEDsLit = 0;
	}

	int LEDsCode = 0; 					// Default no lights lit

	for (int i = 0; i < 10; i++)		// For each LED
	{
		LEDsCode <<= 1;					// Shift previous values across
		LEDsCode |= (LEDsLit > i);		// Set current LED value to 1 if required
	}

	*LEDs = LEDsCode;
}

//=======================================================================================================================================================
//------------------------------------------------------------7 SEGMENT DISPLAY - MESSAGE FUNCTIONS------------------------------------------------------
//=======================================================================================================================================================

/* PrefixTo7SD function:
 * - Displays the chosen prefix on the left-side of the 7SDs.
 */
void PrefixTo7SD(Message msg)
{
	int hexCode;

	switch (msg)
	{
	case MSG_DUTY:		hexCode = 0xFFFF21FF; break;	// "d" for Duty Cycle
	case MSG_SPEED:		hexCode = 0xFFFF12FF; break;	// "S" for Speed
	case MSG_TARGET:	hexCode = 0xFFFF07FF; break;	// "t" for Target
	case MSG_ERROR:		hexCode = 0xFFFF06FF; break;	// "E" for Error
	default:			hexCode = 0xFFFFFFFF;			// For unknown inputs, return blank
	}

	*HexB = hexCode;	// Send to the 7SDs
}

//=======================================================================================================================================================

/* MessageTo7SDs function:
 * - Displays a predefined message on the 7SDs.
 */
void MessageTo7SD(Message msg)
{
	long long int hexCode;

	switch (msg)
	{
	case MSG_OFF:		hexCode = 0xFFFF400E0EFFFFFF; break;
	case MSG_OPEN:		hexCode = 0xFFFF400C0648FFFF; break;
	case MSG_CLOSED:	hexCode = 0xFFFF464740120621; break;
	case MSG_DUTY:		hexCode = 0xFFFF21630711FFFF; break;
	case MSG_SPEED:		hexCode = 0xFFFF120c060621FF; break;
	case MSG_TARGET:	hexCode = 0xFFFF07084E100607; break;
	case MSG_ERROR:		hexCode = 0xFFFF062F2F232FFF; break;
	case MSG_DASHES:	hexCode = 0xFFFF3F3F3F3F3F3F; break;
	case MSG_BLANK:		hexCode = 0xFFFFFFFFFFFFFFFF; break;
	default:			hexCode = 0xFFFFFFFFFFFFFFFF;			// For unknown inputs, return blank
	}

	*HexA = (int) hexCode;				// Send to the 7SDs
	*HexB = (int)(hexCode >> 32);

	WriteLEDs(NONE);					// Turn off LEDs

	MessageDisplayTimestamp = *Clock;	// Log the time-stamp when the message is put up
	MessageBeingDisplayed 	= TRUE;
}

//=======================================================================================================================================================
//------------------------------------------------------------7 SEGMENT DISPLAY - NUMERICAL FUNCTIONS----------------------------------------------------
//=======================================================================================================================================================

/* SevenSegmentDecoder function:
 * - Decodes a character to its corresponding 7SD code.
 */
char SevenSegmentDecoder(char Digit)
{
	switch (Digit)
	{
	case 0:		return 0x40;
	case 1:		return 0x79;
	case 2:		return 0x24;
	case 3:		return 0x30;
	case 4:		return 0x19;
	case 5:		return 0x12;
	case 6:		return 0x02;
	case 7:		return 0x78;
	case 8:		return 0x00;
	case 9:		return 0x18;
	case '-':	return 0x3F;
	default:	return 0xFF; 	// For unknown inputs, return blank
	}
}

//=======================================================================================================================================================

/* InsertDigit function:
 * - Gets the 7SEG code for the given digit.
 * - Inserts it into the specified position in the given memory block.
 */
void InsertDigit(volatile int * HexCode, char Digit, int Position)
{
	char SingleDigitCode = SevenSegmentDecoder(Digit);				// Get the 7SEG code for the given digit

	*HexCode = *HexCode & ~(0xFF << (Position * 8));				// Clear the space where the 7SEG code is to be inserted

	*HexCode = *HexCode |  (SingleDigitCode << (Position * 8));		// Shift the 7SEG code to the correct position and insert it
}

//=======================================================================================================================================================

/* NumberTo7SD function:
 * - Displays a multi-digit number on the 7SDs (maximum 4-digits).
 */
void NumberTo7SD(int Number)
{
	int CurrentDigit = 0;
	volatile int	HexCode = 0xFFFFFFFF;		// Default display code, no lights lit

	Bool NumberIsNegative = ( Number < 0 );		// Log whether the number is negative
	Number = abs(Number);						// Convert to positive

	do											// Loop through all digits in the number, at least once to ensure that a 0 is still displayed
	{
		InsertDigit(&HexCode, Number % 10, CurrentDigit);	// Decode the bottom digit and insert its code into the full code
		Number /= 10;										// Crop the bottom digit out of the number
		CurrentDigit++;										// Move to the next digit
	}
	while (Number != 0); 						// Repeat until all digits have been extracted

	if (NumberIsNegative)						// Insert a negative sign before the number if required
	{
		if ( CurrentDigit >= 4 ) InsertDigit(HexB, '-', 0);					// Special case when -ve sign needs to overflow into HexB
		else 					InsertDigit(&HexCode, '-', CurrentDigit);
	}

	*HexA = HexCode;	//Send to the 7SDs
}

//=======================================================================================================================================================
//------------------------------------------------------------7 SEGMENT DISPLAY - WRITE FUNCTION---------------------------------------------------------
//=======================================================================================================================================================

/* Write7SD function:
 * Controls the 7-segment display:
 * - Displays the value of selected system parameter.
 * - Holds system messages on display for the required time.
 */
void Write7SD()
{
	if (MessageBeingDisplayed)
	{
		unsigned int TimeElapsed = *Clock - MessageDisplayTimestamp;
		if (TimeElapsed > MESSAGE_DISPLAY_TIME) MessageBeingDisplayed = FALSE;		// Stop displaying a message if the required time has passed
	}

	if (!MessageBeingDisplayed)
	{
		unsigned int DutyPercent;

		switch (SelectedDisplay)
		{
		case DUTY:											// Display Duty Cycle %
			PrefixTo7SD(MSG_DUTY);
			DutyPercent = Duty * 100 / DUTY_RESOLUTION;		// Convert to %
			NumberTo7SD(DutyPercent);
			WriteLEDs(DUTY);
			break;
		case SPEED:											// Display measured fan speed
			PrefixTo7SD(MSG_SPEED);
			NumberTo7SD(FanSpeed);
			WriteLEDs(SPEED);
			break;
		case TARGET:										// Display target speed
			PrefixTo7SD(MSG_TARGET);
			NumberTo7SD(TargetSpeed);
			WriteLEDs(TARGET);
			break;
		case ERROR:											// Display speed error
			PrefixTo7SD(MSG_ERROR);
			NumberTo7SD(-SpeedError);
			WriteLEDs(ERROR);
			break;
		default:
			MessageTo7SD(MSG_BLANK);
		}
	}
}

//=======================================================================================================================================================
//------------------------------------------------------------READ USER INPUTS FUNCTIONS-----------------------------------------------------------------
//=======================================================================================================================================================

/* ReadButtons function:
 * - Detects button presses.
 * - Adjusts display options in response.
 */
void ReadButtons()
{
	switch (*Buttons)
	{
	case key3:								// Display Duty Cycle
		SelectedDisplay = DUTY;
		MessageTo7SD(MSG_DUTY);
		break;
	case key2:								// Display measured fan speed
		SelectedDisplay = SPEED;
		MessageTo7SD(MSG_SPEED);
		break;
	case key1:								// Display target speed if in closed-loop. Display dashes if in open-loop.
		if (ClosedLoopEnabled)
		{
			SelectedDisplay = TARGET;
			MessageTo7SD(MSG_TARGET);
		}
		else MessageTo7SD(MSG_DASHES);
		break;
	case key0:								// Display speed error if in closed-loop. Display dashes if in open-loop.
		if (ClosedLoopEnabled)
		{
			SelectedDisplay = ERROR;
			MessageTo7SD(MSG_ERROR);
		}
		else MessageTo7SD(MSG_DASHES);
		break;
	}
}

//=======================================================================================================================================================

/* ReadSwitches function:
 * - Detects changes in toggle switch values.
 * - Adjusts system parameters in response.
 */
void ReadSwitches()
{
	static Bool PrevPowerEnabled = FALSE, PrevClosedLoopEnabled = FALSE;
	static unsigned int DutyAtSwitchOff = DUTY_S_INITIAL;


	PowerEnabled 	  =	ReadSwitch(POWER_SWITCH);				// Read switch values
	ClosedLoopEnabled = ReadSwitch(CLOSED_LOOP_SWITCH);

	if (PowerEnabled && !PrevPowerEnabled) 						// Power was toggled on
	{
		if (ClosedLoopEnabled)	MessageTo7SD(MSG_CLOSED);
		else					MessageTo7SD(MSG_OPEN);
		SetDuty(DutyAtSwitchOff);
	}

	else if (!PowerEnabled && PrevPowerEnabled) 				// Power was toggled off
	{
		DutyAtSwitchOff = Duty;
		MessageTo7SD(MSG_OFF);
	}

	if ( ClosedLoopEnabled && !PrevClosedLoopEnabled ) 			// Closed-loop control was enabled
	{
		SelectedDisplay = TARGET;
		if (PowerEnabled) MessageTo7SD(MSG_CLOSED);
	}

	else if( !ClosedLoopEnabled && PrevClosedLoopEnabled ) 		// Closed-loop control was disabled
	{
		SelectedDisplay = DUTY;
		if (PowerEnabled) MessageTo7SD(MSG_OPEN);
	}

	PrevPowerEnabled = PowerEnabled;
	PrevClosedLoopEnabled = ClosedLoopEnabled;
}

//=======================================================================================================================================================

/* ReadRotaryEncoder function:
 * Detects changes in the rotary encoder position, and updates the Duty Cycle / target speed in response.
 * Movement detection is achieved by:
 * - Concatenating the previous and new 2-bit Gray code signals to create a 4-bit value.
 * - This value corresponds to an index in the direction table, which contains the direction of the movement that occurred.
 */
void ReadRotaryEncoder()
{
	static char PrevCode;
	static const Direction DirectionTable[] = {NA, CW, CCW, NA, CCW, NA, NA, CW, CW, NA, NA, CCW, NA, CCW, CW, NA};

	char NewCode = ( ReadPin(ROTENC_PIN_B) << 1 ) | ReadPin(ROTENC_PIN_A);	// Read rotary encoder pins and store as 2-bit code

	if (NewCode != PrevCode)												// Encoder position has changed
	{
		unsigned char directionIndex = (PrevCode << 2) | NewCode;			// Concatenate previous and new codes to a 4-bit value
		Direction DirectionMoved = DirectionTable[directionIndex];			// Get the corresponding movement direction

		if (ClosedLoopEnabled)		// Closed-loop, adjust the target speed
		{
			if 		(DirectionMoved == CW) 	SetTargetSpeed(TargetSpeed + TARGET_INCREMENT_STEP);
			else if (DirectionMoved == CCW) SetTargetSpeed(TargetSpeed - TARGET_INCREMENT_STEP);
		}
		else						// Open-loop, adjust the Duty Cycle
		{
			if 		(DirectionMoved == CW) 	SetDuty(Duty + DUTY_S_INCREMENT_STEP);
			else if (DirectionMoved == CCW) SetDuty(Duty - DUTY_S_INCREMENT_STEP);
		}

		PrevCode = NewCode;			// Store the new code for the next loop
	}
}

//=======================================================================================================================================================
//------------------------------------------------------------ARCHITECTURE FUNCTIONS---------------------------------------------------------------------
//=======================================================================================================================================================

/* ShouldItBeCalled function:
 * Checks if the specified function passes its criteria for being called.
 * - All functions have a criterion for a minimum time period to have elapsed since its previous call.
 * - All functions except ReadSwitches also have a criterion for the power to be on.
 */
Bool ShouldItBeCalled(Function fcn)
{
	unsigned int TimeElapsed;
	static unsigned int PrevCallTime[] = { 0, 0, 0, 0, 0 };					// Array of previous call time-stamps
	static const unsigned int MinTimeRequired[] = { SWITCH_REFRESH_TIME,	// Array of minimum time periods
													BUTTON_REFRESH_TIME,
													ROTARY_REFRESH_TIME,
													SPEED_REFRESH_TIME,
													SEVENSD_REFRESH_TIME };

	switch (fcn)	// Note: break statements omitted so subsequent criteria are also tested
	{
	case FCN_SPEED:
	case FCN_BUTTONS:
	case FCN_ROTARYENCODER:
	case FCN_7SD:
		if (!PowerEnabled) return FALSE;				// Criterion 1: Power must be on
	case FCN_SWITCHES:
		TimeElapsed = *Clock - PrevCallTime[fcn];		// Measure the time since previous call

		if ( TimeElapsed > MinTimeRequired[fcn] )		// Criterion 2: If the defined time has elapsed since the previous call
		{
			PrevCallTime[fcn] = *Clock;					// Update the call time-stamp for the next iteration
			return TRUE;								// Allow the function to be called
		}
		else
			return FALSE;								// Not enough time elapsed, don't call the function
	default:
		return FALSE;
	}
}

//=======================================================================================================================================================

/* Main function:
 * - Initialises the fan controller.
 * - Indefinitely calls each major function in turn.
 */
int main(int argc, char** argv)
{
	EE30186_Start();					// Initialise the FPGA configuration

	*(GpioPort + 1) = 1 << PWM_PIN;		// Set the GPIO port data direction register so the PWM pin is an O/P, all other pins are inputs

	PowerEnabled = FALSE;				// Set initial system parameter values
	ClosedLoopEnabled = FALSE;
	SelectedDisplay = DUTY;
	SetDuty(0);
	SetTargetSpeed(TARGET_INITIAL);
	MessageTo7SD(MSG_OFF);

	while (1)					// Main loop
	{
		ReadTachometer();		// Always call these functions
		Modulation();

		if (ShouldItBeCalled(FCN_SWITCHES)) 	 ReadSwitches();		// Only call these functions if their calling criteria is met
		if (ShouldItBeCalled(FCN_BUTTONS)) 		 ReadButtons();
		if (ShouldItBeCalled(FCN_ROTARYENCODER)) ReadRotaryEncoder();
		if (ShouldItBeCalled(FCN_SPEED))		 CalculateFanSpeed();
		if (ShouldItBeCalled(FCN_7SD))			 Write7SD();
	}

    EE30186_End();				// Clean up and close the FPGA configuration

    return 0;
}
