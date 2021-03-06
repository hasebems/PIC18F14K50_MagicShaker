/*
 * File:   main18.c
 * Author: hasebems
 *
 * Created on 2014/12/19, 22:57
 * Copied from magicFlute 2015/5/12
 */


#include <xc.h>

#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_midi.h"
#include "i2cdevice.h"

#include	"mfconfig.h"

/*----------------------------------------------------------------------------*/
#ifndef _XTAL_FREQ
    /* In case of 4MHz, Set 4000000 */
    #define _XTAL_FREQ 48000000
#endif
//#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000UL)))
//#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000UL)))

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config PLLEN = ON       // 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 19        // Brown-out Reset Voltage bits (VBOR set to 1.9 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RA3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)


#define	SW1			PORTCbits.RC7   //  Switch
#define	OUT2		PORTCbits.RC6
#define	OUT1		PORTCbits.RC5   //  Heartbeat
#define	DIPSW2		PORTCbits.RC4
#define DIPSW1      PORTCbits.RC3   //  1:Ocarina
#define LEDR        PORTCbits.RC2
#define LEDG        PORTCbits.RC1
#define LEDB        PORTCbits.RC0

/*----------------------------------------------------------------------------*/
//
//      Macros
//
/*----------------------------------------------------------------------------*/
#define SERIAL_MIDI_BUFF    32

#define	MIDI_BUF_MAX		8
#define	MIDI_BUF_MAX_MASK	0x07;

#define	NO_NOTE		12

#define	GRADIENT_MVG_AVE_CNT		8
#define GRADIENT_MVG_AVE_MASK		0x07

typedef enum {
	DETECT_SHAKING,	//	Accelerator to detect shaking
	DETECT_INCLINE,	//	Accelerator to detect incline
	MAX_DETECT
} ACCEL_TYPE;

typedef enum {
	FT_WAIT,
	INV_WAIT,
	MA_WAIT,
	STATE_MAX
} SHAKE_STATE_MACHINE;


/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static uint8_t ReceivedDataBuffer[64] @ DEVCE_AUDIO_MIDI_RX_DATA_BUFFER_ADDRESS;
static USB_AUDIO_MIDI_EVENT_PACKET midiData @ DEVCE_AUDIO_MIDI_EVENT_DATA_BUFFER_ADDRESS;

static USB_HANDLE	USBTxHandle;
static USB_HANDLE	USBRxHandle;

//	MIDI for serial (phisical MIDI)
static uint8_t		serialMidiBuffer[SERIAL_MIDI_BUFF];
static int			smbReadPtr;
static int			smbWritePtr;
static uint8_t		runningStatus;

static bool			sentNoteOff;

static uint8_t		midiEvent[MIDI_BUF_MAX][3];
static int			midiEventReadPointer;
static int			midiEventWritePointer;

static uint8_t		crntNote;
static uint8_t		midiExp;
static int			doremi;

static long			counter10msec;	//	one loop 243 days
static bool			event5msec;
static bool			event10msec;
static bool			event100msec;
static uint16_t		timerStock;
static uint8_t		tmr2Cnt;

static signed long			gravity[MAX_DETECT][3];
static signed short			shake[3];
static int					shakeCount;
static SHAKE_STATE_MACHINE	zaxisStateMachine;
static signed short			zaxisMaxStock;
static signed short			debugOld[3];

static int			kofCounter;
static int			dbgCounter;

static int			i2cComErr;
static bool			usbEnable;

/*----------------------------------------------------------------------------*/
//
//      Full Color LED by Interrupt
//
/*----------------------------------------------------------------------------*/
const unsigned char tColorTable[13][3] = {

	//	this Value * midiExp(0-127) / 16 = 0x00-0xfe : PWM count value
	// R	 G		B
	{ 0x20,  0x00,  0x00  },   //  red		C
	{ 0x1a,  0x06,  0x00  },   //  red		C#
	{ 0x16,  0x0a,  0x00  },   //  orange	D
	{ 0x14,  0x0c,  0x00  },   //  orange	D#
	{ 0x0c,  0x14,  0x00  },   //  yellow	E
	{ 0x00,  0x20,  0x00  },   //  green	F
	{ 0x00,  0x10,  0x10  },   //  green	F#
	{ 0x00,  0x00,  0x20  },   //  blue		G
	{ 0x04,  0x00,  0x1c  },   //  blue		G#
	{ 0x08,  0x00,  0x18  },   //  violet	A
	{ 0x0c,  0x00,  0x14  },   //  violet	A#
	{ 0x18,  0x00,  0x08  },   //  violet	B
	{ 0x00,  0x00,  0x00  },   //  none
};
//-------------------------------------------------------------------------
void interrupt lightFullColorLed( void )
{
    if (TMR2IF == 1) {          // Timer2 Interrupt?
		TMR2IF = 0 ;            // reset of Timer2 Interrupt
		tmr2Cnt += 0x04 ;       // PWM resolution

		//	PWM Full Color LED
		uint16_t ledCnt;
		ledCnt = ((uint16_t)tColorTable[doremi][0]*midiExp)>>4;
		LEDR = ((uint16_t)tmr2Cnt >= ledCnt)? 1:0;

		ledCnt = ((uint16_t)tColorTable[doremi][1]*midiExp)>>4;
		LEDG = ((uint16_t)tmr2Cnt >= ledCnt)? 1:0;

		ledCnt = ((uint16_t)tColorTable[doremi][2]*midiExp)>>4;
		LEDB = ((uint16_t)tmr2Cnt >= ledCnt)? 1:0;
	}
}


/*----------------------------------------------------------------------------*/
//
//      Common Initialize ( Power On / USB reset )
//
/*----------------------------------------------------------------------------*/
void initCommon( void )
{
	midiEventReadPointer = 0;
	midiEventWritePointer = 0;

    smbReadPtr = 0;
    smbWritePtr = 0;
    runningStatus = 0x00;

	crntNote = 0;
	doremi = NO_NOTE;

	i2cComErr = 0;
    usbEnable = true;

	int	i;
	for (i=0;i<3;i++){
		gravity[DETECT_SHAKING][i] = 0;
		gravity[DETECT_INCLINE][i] = 0;
		shake[i] = 0;
		debugOld[i] = 0;
	}

	shakeCount = 0;
	zaxisStateMachine = FT_WAIT;
	zaxisMaxStock = 0;

	kofCounter = 0;
	dbgCounter = 0;

	//AnalyseTouch_init();
}

/*----------------------------------------------------------------------------*/
//
//      Init I2C Hardware
//
/*----------------------------------------------------------------------------*/
void initAllI2cHw( void )
{
	initI2c();

#if USE_I2C_TOUCH_SENSOR
	MPR121_init();
#endif
#if USE_I2C_ACCELERATOR_SENSOR
	ADXL345_init(DETECT_SHAKING);
	ADXL345_init(DETECT_INCLINE);
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Initialize only for Power On
//
/*----------------------------------------------------------------------------*/
void initMain(void)
{
	int		i;

	//	PIC H/W registor
	INTCON	=	0b00000000;         //  Disable all Interrupt
	
	//	Set Port
	//    ADCON1  =	0b00001111;
    TRISA   =	0b00000000;			//D-,D+
    TRISB   =	0b11110000;			//I2C master mode, UART Tx/Rx(set INPUT)
    TRISC   =	0b10011000;			//SW1, DIPSW1, DIPSW2 for PROTO8
	ANSEL	=	0b00000000;			//not use ADC. use PORT
	ANSELH	=	0b00000000;
	T0CON	=	0b10010111;			// 1:256 of System Clock
									//	 48/4MHz -> 46875Hz 21.333..usec

//    LATA    =	0b00000000;
//    LATB    =	0b00000000;
//    LATC    =	0b01000000;

	//	TIMER0
	TMR0H	= 0;
	TMR0L	= 0;
	OUT1	= 0;
	OUT2	= 0;

	// Interrupt by TIMER2
	tmr2Cnt = 0 ;					// PWM Counter clear
	T2CON  = 0b00000111 ;			// TMR2 prescaler 1:16, postscaler 1:1 (48/4*16MHz) 1.333...usec
	PR2    = 187 ;					// TMR2 interrupt count Interval: 250usec
	TMR2   = 0 ;					// Initialize
	TMR2IF = 0 ;					// clear TMR2 Interrupt flag
	TMR2IE = 1 ;					// enable TMR2 interrupt

	//	UART
    RCSTA   = 0b10010000;           // enable UART Tx & Rx
    TXSTA   = 0b00100000;           // 8bit Asynclonous Tx/Rx
    BAUDCON = 0b00000000;           // HI-16bit baudrate=disable
    SPBRG   = 23;                   // 31250[bps]

	//	Initialize Variables only when the power turns on
	for ( i=0; i<MIDI_BUF_MAX; i++ ){
		midiEvent[i][0] = 0;
		midiEvent[i][1] = 0;
		midiEvent[i][2] = 0;
	}
	counter10msec = 0;
	event5msec = false;
	event10msec = false;
	event100msec = false;
	timerStock = 0;
	midiExp = 0;

	//	Iitialize other H/W
	initAllI2cHw();

	//	common Initialize
	initCommon();

	//	Enable All Interrupt
	PEIE   = 1 ;					// enable peripheral interrupt
	GIE    = 1 ;					// enable all interrupt
}

/*----------------------------------------------------------------------------*/
//
//	Function: void USBMIDIInitialize(void);
//
//	Overview: Initializes the demo code
//
/*----------------------------------------------------------------------------*/
void USBMIDIInitialize()
{
    USBTxHandle = NULL;
    USBRxHandle = NULL;
    sentNoteOff = true;

	initCommon();

    //enable the HID endpoint
    USBEnableEndpoint(USB_DEVICE_AUDIO_MIDI_ENDPOINT,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);
}

/*----------------------------------------------------------------------------*/
//
//	Function: void APP_DeviceAudioMIDISOFHandler(void);
//
/*----------------------------------------------------------------------------*/
void APP_DeviceAudioMIDISOFHandler()
{
}
/*----------------------------------------------------------------------------*/
//
//      USB Callback Function
//
/*----------------------------------------------------------------------------*/
bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch( (int) event )
    {
        case EVENT_CONFIGURED:
            /* When the device is configured, we can (re)initialize the demo
             * code. */
            USBMIDIInitialize();
            break;
#if 0
		case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            /* We are using the SOF as a timer to time the LED indicator.  Call
             * the LED update function here. */
			APP_DeviceAudioMIDISOFHandler();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;
#endif
        default:
            break;
    }
    return true;
}

/*----------------------------------------------------------------------------*/
//
//      Generate Counter
//
/*----------------------------------------------------------------------------*/
void generateCounter( void )
{
	uint16_t tmr;

	//	Make Master Counter
	tmr = (uint16_t)TMR0L;
	tmr |= (uint16_t)(TMR0H << 8);

	//	Generate Timer Event
	if (( tmr & 0x0080 ) && !( timerStock & 0x0080 )){
		//	5msec Event ( precise time : 5.46msec )
		event5msec = true;
	}
	else event5msec = false;

	if (( tmr & 0x0100 ) && !( timerStock & 0x0100 )){
		//	10msec Event ( precise time : 10.92msec )
		event10msec = true;
		counter10msec++;
	}
	else event10msec = false;

	if (( tmr & 0x0800 ) && !( timerStock & 0x0800 )){
		//	100msec Event ( precise time : 87.37msec )
		event100msec = true;
	}
	else event100msec = false;

	timerStock = tmr;
}

/*----------------------------------------------------------------------------*/
//
//      Set MIDI Buffer
//
/*----------------------------------------------------------------------------*/
void setMidiBuffer( uint8_t status, uint8_t dt1, uint8_t dt2 )
{
    //  for USB MIDI
    if ( usbEnable == true ){
    	midiEvent[midiEventWritePointer][0] = status;
        midiEvent[midiEventWritePointer][1] = dt1;
    	midiEvent[midiEventWritePointer][2] = dt2;
        midiEventWritePointer++;
        midiEventWritePointer &= MIDI_BUF_MAX_MASK;
    }

    //  for Serial MIDI
    if ( status != runningStatus ){
        runningStatus = status;
        serialMidiBuffer[smbWritePtr++] = status;
      if ( smbWritePtr >= SERIAL_MIDI_BUFF ){ smbWritePtr -= SERIAL_MIDI_BUFF;}
    }
    serialMidiBuffer[smbWritePtr++] = dt1;
    if ( smbWritePtr >= SERIAL_MIDI_BUFF ){ smbWritePtr -= SERIAL_MIDI_BUFF;}
    serialMidiBuffer[smbWritePtr++] = dt2;
    if ( smbWritePtr >= SERIAL_MIDI_BUFF ){ smbWritePtr -= SERIAL_MIDI_BUFF;}
}

/*----------------------------------------------------------------------------*/
//
//      MIDI Test Note Event
//
/*----------------------------------------------------------------------------*/
void testNoteEvent( void )
{
	/* If a button is pressed... */
    if ( SW1 == 0 ){    //(BUTTON_IsPressed(BUTTON_DEVICE_AUDIO_MIDI) == true
        if ( sentNoteOff == true ){
			setMidiBuffer( 0x90, 0x24+MF_FIRM_VERSION, 0x7f );
            sentNoteOff = false;
		}
    }
    else {
        if ( sentNoteOff == false ){
			setMidiBuffer( 0x90, 0x24+MF_FIRM_VERSION, 0x00 );
            sentNoteOff = true;
		}
    }
}

#if USE_I2C_ACCELERATOR_SENSOR
/*----------------------------------------------------------------------------*/
//
//      Detect shaking / incline
//
/*----------------------------------------------------------------------------*/
#define		LPF_COEF		230		//	230/256 = 0.9
/*----------------------------------------------------------------------------*/
void analyseAccelValue( ACCEL_TYPE tp, signed short* acl )
{
	int		i;
	signed long tmp;
	
	//	remove a gravity ingredient from acceleration
	for ( i=0; i<3; i++){
		tmp = (signed long)acl[i];
		if ( tmp > 32768 ) tmp -= 65536;	//	-32768 - +32767
		gravity[tp][i] = (LPF_COEF*gravity[tp][i] + (256-LPF_COEF)*tmp)/256;
		if ( tp == DETECT_SHAKING ){
			signed long shk = acl[i] - gravity[tp][i];
			shk /= 512;
			shake[i] = shk;
		}
	}
}

/*----------------------------------------------------------------------------*/
//
//      Generate MIDI by shaking
//
/*----------------------------------------------------------------------------*/
const uint8_t tCnvAccelToVel[128] = 
{
	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	2,	4,	6,	8,	10,	12,	14,	16,	18,	20,
	22,	24,	26,	28,	30,	32,	34,	36,	38,	40,
	42,	44,	46,	48,	50,	52,	54,	56,	58,	60,
	62,	64,	66,	68,	71,	74,	77,	80,	83,	86,
	88,	90,	92,	94,	96,	98,	100,	102,	104,	106,
	108,	110,	112,	114,	116,	118,	120,	122,	124,	126,
	127,127,127,127,127,127,127,127,127,127,
	127,127,127,127,127,127,127,127,127,127,
	127,127,127,127,127,127,127,127,127,127,
	127,127,127,127,127,127,127,127,127,127,
	127,127,127,127,127,127,127,127
};
/*----------------------------------------------------------------------------*/
void keyOnByShaking(void)
{
	uint8_t vel;

	if ( gravity[DETECT_INCLINE][0] > 0 ){
		crntNote = 0x3a;
	}
	else {
		crntNote = 0x37;
	}
	
	if ( zaxisMaxStock <= 0 ){ vel = 1; }
	else {
		vel = zaxisMaxStock;
		if ( vel > 127 ) vel = 127;
	}
	setMidiBuffer(0x99,crntNote,vel/*tCnvAccelToVel[vel]*/);
	doremi = crntNote%12;
	midiExp = vel;
}
/*----------------------------------------------------------------------------*/
void keyOffByShaking(void)
{
	setMidiBuffer(0x99,crntNote,0);
	doremi = NO_NOTE;
	midiExp = 0;	
}
/*----------------------------------------------------------------------------*/
#define		FT_LVL					(-10)
#define		MIN_MA_LVL				10
#define		INV_TIME				12	//	*10msec
#define		MA_TIME					10	//	*10msec
#define		KEY_OFF_CNT				10	//	*10msec
/*----------------------------------------------------------------------------*/
void generateZaxisMIDI( void )
{
	signed short shk = shake[2];	//	z-axis
	signed short* maxStock = &zaxisMaxStock;
	SHAKE_STATE_MACHINE* stateMachine = &zaxisStateMachine;

	switch( *stateMachine ){
		default:
		case FT_WAIT:
			if ( shk < FT_LVL ){
				*stateMachine = INV_WAIT;
				shakeCount = 0;
			}
			break;

		case INV_WAIT:
			//	check time-up
			if ( shakeCount > INV_TIME ){
				*stateMachine = FT_WAIT;
			}
			shakeCount++;

			if ( shk > 0 ){
				//	Invert
				*stateMachine = MA_WAIT;
				*maxStock = 0;
				shakeCount = 0;
			}
			break;

		case MA_WAIT:
			if ( shk < 0 ){
				*stateMachine = FT_WAIT;
			}
			//	Check time-up
			if ( shakeCount > MA_TIME ){
				*stateMachine = FT_WAIT;				
			}
			shakeCount++;

			if ( *maxStock < shk ){
				//	Update Max Value
				*maxStock = shk;
			}
			else {
				if ( *maxStock > MIN_MA_LVL ){
					//	KeyOn
					if ( kofCounter ){
						keyOffByShaking();
					}
					keyOnByShaking();
					shakeCount = 0;
					kofCounter = KEY_OFF_CNT;
				}
				*stateMachine = FT_WAIT;
				*maxStock = 0;
			}
			break;
	}

	//	Generate Key Off
	if ( kofCounter ){
		kofCounter--;
		if ( !kofCounter ){
			keyOffByShaking();
		}
	}
}
/*----------------------------------------------------------------------------*/
//
//      Accelerator Sensor Job
//
/*----------------------------------------------------------------------------*/
void acceleratorSensor( void )
{
	signed short aclShake[3] = {0,0,0};
	signed short aclIncline[3] = {0,0,0};
	int		err, i;

	//	once 10msec
	if ( event10msec == false ) return;

	err = ADXL345_getAccel(0,aclShake);
	if ( err != 0 ) i2cComErr = err;

	err = ADXL345_getAccel(1,aclIncline);
	if ( err != 0 ) i2cComErr = err + 40;
	
	//	Analyse & Generate MIDI
	analyseAccelValue( DETECT_SHAKING, aclShake );
	analyseAccelValue( DETECT_INCLINE, aclIncline );
	generateZaxisMIDI();

	//	for debug
#if 0	//	observe one sensor's x,y,z-axis
	for (int i=0;i<3;i++){
		signed long dbgDt = gravity[DETECT_SHAKING][i]/256;
		if ( dbgDt != debugOld[i] ){
			setMidiBuffer(0xb0,0x50+i,(uint8_t)(dbgDt+64));
			debugOld[i] = dbgDt;
		}
	}
#endif	
#if 1	//	observe two sensor's z-axis
	signed long dbgDt = gravity[DETECT_SHAKING][2]/256;
	if ( dbgDt != debugOld[0] ){
		setMidiBuffer(0xb0,0x50,(uint8_t)(dbgDt+64));
		debugOld[0] = dbgDt;
	}
	signed long dbgDt2 = gravity[DETECT_INCLINE][2]/256;
	if ( dbgDt2 != debugOld[1] ){
		setMidiBuffer(0xb0,0x51,(uint8_t)(dbgDt2+64));
		debugOld[1] = dbgDt2;
	}
#endif
}
#endif
/*----------------------------------------------------------------------------*/
//
//      Debug by using MIDI / extra LED
//
/*----------------------------------------------------------------------------*/
void midiOutDebugCode( void )
{
	//	Heartbeat
	OUT1 = ((counter10msec & 0x001e) == 0x0000)? 1:0;		//	350msec

	//	Debug by using LED
	if ( i2cErr == true ){
		if ( event100msec == true ){
			OUT2 = 0;
			i2cErr = false;
		}
		else OUT2 = 1;
	}

	//	Debug by using USB MIDI
	if ( event100msec == true ){
		if ( i2cComErr != 0 ){
			setMidiBuffer(0xb0,0x10,(unsigned char)i2cComErr);
		}
	}
}
/*----------------------------------------------------------------------------*/
//
//      Send one event to MIDI ( just only one event for each time )
//
/*----------------------------------------------------------------------------*/
void sendEventToUSBMIDI( void )
{
    /* If the device is not configured yet, or the device is suspended, then
     * we don't need to run the demo since we can't send any data.
     */
    if ( (USBGetDeviceState() < CONFIGURED_STATE) ||
         (USBIsDeviceSuspended() == true)){
        return;
    }

	//	USB MIDI In
	if ( !USBHandleBusy(USBRxHandle) ){
		//We have received a MIDI packet from the host, process it and then
		//  prepare to receive the next packet

        //INSERT MIDI PROCESSING CODE HERE		

		//Get ready for next packet (this will overwrite the old data)
		USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);
	}

	//	USB MIDI Out
	if ( USBHandleBusy(USBTxHandle) ){ return;}	

	if ( midiEventReadPointer != midiEventWritePointer ){
		uint8_t	statusByte = midiEvent[midiEventReadPointer][0];

		midiData.Val = 0;   //must set all unused values to 0 so go ahead
                            //  and set them all to 0

		midiData.CableNumber = 0;
		midiData.CodeIndexNumber = statusByte >> 4;

        midiData.DATA_0 = statusByte;								// Status Byte
        midiData.DATA_1 = midiEvent[midiEventReadPointer][1];		// Data Byte 1
        midiData.DATA_2 = midiEvent[midiEventReadPointer][2];		// Data Byte 2
        USBTxHandle = USBTxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&midiData,4);

		midiEventReadPointer++;
		midiEventReadPointer &= MIDI_BUF_MAX_MASK;
	}
}
/*----------------------------------------------------------------------------*/
void sendEventToRealMIDI( void ){
    //  Serial MIDI
    if ( smbReadPtr != smbWritePtr ){
        if (PIR1bits.TXIF){  // Tx is already finished
            TXREG = serialMidiBuffer[smbReadPtr++];
            if ( smbReadPtr >= SERIAL_MIDI_BUFF ){ smbReadPtr -= SERIAL_MIDI_BUFF; }
        }
    }
}

/*----------------------------------------------------------------------------*/
//
//      Main Function
//
/*----------------------------------------------------------------------------*/
void main(void)
{
    initMain();

	USBDeviceInit();

    while(1){
		//	Increment Counter & Make Timer Event
		generateCounter();

		//	USB
		usbEnable = false;
		USBDeviceTasks();
		usbEnable = true;

		//	Test by Tact Swtich
		testNoteEvent();

		//  accelerator sensor
#if USE_I2C_ACCELERATOR_SENSOR
		acceleratorSensor();
#endif
		//	Debug by using MIDI
		midiOutDebugCode();

		//	MIDI Out
		sendEventToUSBMIDI();
		sendEventToRealMIDI();
	}

	return;
}
