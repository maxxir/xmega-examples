/*
+ FREQ Generator ADJ via POTENTIOMETR on PA2 
* 4 диапазона перестройки от 4Mhz до 61Hz:
* 4Mhz-200kHz/200kHz-10kHz/10kHz-500Hz/500Hz-61Hz
* 
+ FREQ METER via TCC1 pin (PORTC1) && show result via USARTE0 every second
+ from 16Mhz till 1Hz (with SYSCLOCK=32Mhz) 
* PS. with OK overcloked for XMEGA***AU series (SYSCLOCK=64Mhz)
* we can measure from 32Mhz till 1Hz

+ FREQ GENERATOR via TCC0 CCA pin (PORTC.0) 
+ from 4mHz till 61Hz with step INC 250ns for FREQ generation

+ micros timing via event system
* Use timer TCD0 and FREQ Source from event system SYSCLOCK / 32 = 1us
* PS. прямой прескалер таймера считает с предделителями 1/2/4/8/64/256/1024
* при SYSCLK = 32Mhz надо предделитель 32 для получения шага 1us - мы
* получаем его через 0 канал EventSystem - там прескалер идет с шагом от 2^0=1,2^1=2...  до 2^^15=32768
 

+ Added redirect stdout (printf/printf_P..) to USARTE0
+ added printf for float
 PRINTF_LIB = $(PRINTF_LIB_FLOAT)

+ Usart Hello TX - polling & RX in IRQ mode
+ Used LOW LVL IRQ with Round-Robin Priority
+ Add Programm Metrics printout
Should be use USARTE0 9600bps
>>>PS. used code from BostonAndroid XMEGA code: <xmega-waveform-serial.c>

In XMEGAxxA4[U] 5 USARTs:
On PORTC: USARTC0/USARTC1
On PORTD: USARTD0/USARTD1
On PORTE: USARTE0

+ Timer IRQ Example
In XMEGAxxA4[U] 5 x 16-bit timers:
TCC0/TCC1/TCD0/TCD1/TCE0
prescalers: OFF/1/2/4/8/64/256/1024 and count from EventChannel
* 
* We can use 3 IRQ LVL at XMEGA:
* HI, MED, LO

Minimal template for XMEGA32A4U
(c) Ibragimov M. Russia Togliatty 26/09/2014

USED Internal RC for 32Mhz FREQ (NO PLL mode) or EXT XTAL 8/16Mhz
LED PIN PORTA6 to toggle via TIMER0 delay 1sec while not pressing SW1(PORTA7) && 100msec when pressing SW1(PORTA7)
PORTC7 used for check SYSTEM clockout (must be 32Mhz)

!!Check in real hardware!!
* 05.01.2015 - ALL CHECKED OK!! on [gcc4.8.1 from <<arduino-1.5.6-xmegaduino-beta5>>]
PS.
ALL peripherials check on:
<E:\working\arduino-1.5.6-xmegaduino-beta5\hardware\tools\avr\avr\include\avr\iox32a4u.h>
* 
*/
//!! To avoid poisoned errors (for old MEGA code)
//#define __AVR_LIBC_DEPRECATED_ENABLE__

#include <avr/io.h>
#include <util/delay.h> // for _delay_us[ms] macro
#include <avr/interrupt.h>
#include <stdio.h> // For sprintf etc..
#include <string.h> // For memset etc..
#include <avr/pgmspace.h>

//*********Program metrics
const char compile_date[] PROGMEM    = __DATE__;     // Mmm dd yyyy - Дата компиляции
const char compile_time[] PROGMEM    = __TIME__;     // hh:mm:ss - Время компиляции
const char PROGMEM str_prog_name[]    = "\r\n\r\nXMEGA32A4U Timer FREQ OUT as FREQ GENERATOR POT PA2 and FREQ COUNTER 16/01/2015..\r\n"; // Program name

//*********Board defines
#define LED 6 //PORTA.6
#define SW1 7 //PORTA.7


//*********VARs
volatile uint32_t uptime; // Uptime in seconds

//****Function declaration: BEGIN
void USARTE0_init(void);
unsigned char USARTE0_ReadChar(void);
unsigned char USARTE0_RX_Available(void);
void USARTE0_WriteChar(unsigned char data);
void USARTE0_WriteString(char *string);
void USARTE0_WriteLine(char *string);
void USARTE0_WriteString_P(PGM_P string);
int USARTE0_putch(char ch,FILE *stream);
void ADC_init_11bit_signed(void);
//****Function declaration: END

//****STDIO redirect (printf etc..)
FILE USARTE0_stdout = FDEV_SETUP_STREAM (USARTE0_putch, NULL, _FDEV_SETUP_WRITE); // Only STDOUT Stream use printf etc.. (no scanf etc..)
//FILE uart1_str = FDEV_SETUP_STREAM(uart1_putch, uart1_getch, _FDEV_SETUP_RW); // STDOUT/STDIN stream (use printf & scanf)

void Config32MHzClock_Internal(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};

//!! From Batsocks Example 16Mhz EXT XTAL to 32Mhz SYSCLOCK with PLLx2
// Not used here
/*
void init_SystemClock_External_32Mhz_from16Mhz_x_PLL2( void ){
// Use an external 16Mhz crystal and x 2 PLL to give a clock of 32Mhz

	// Enable the external oscillator
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc ;
	OSC.CTRL |= OSC_XOSCEN_bm ;
	while( (OSC.STATUS & OSC_XOSCRDY_bm) == 0 ){} // wait until stable

	// Now configure the PLL to be eXternal OSCillator * 2
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 2 ;
	OSC.CTRL |= OSC_PLLEN_bm ; // enable the PLL...
	while( (OSC.STATUS & OSC_PLLRDY_bm) == 0 ){} // wait until stable

	// And now switch to the PLL as the clocksource
	CCP = CCP_IOREG_gc; // protected write follows	 
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}
*/

void Config32MHzClock_External_XTAL_16Mhz_x_PLL2(void)
{
	// Use an external 16Mhz crystal and x 2 PLL to give a clock of 32Mhz
	// Enable the external oscillator
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc ;
	OSC.CTRL |= OSC_XOSCEN_bm ;
	while( (OSC.STATUS & OSC_XOSCRDY_bm) == 0 ){} // wait until stable

	// Now configure the PLL to be eXternal OSCillator * 2
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 2 ; 
	
	//!!PS
	//x4 Тоже работает для AU серии т.е 64Mhz-64MIPs!!!  (на AVRFreaks пишут что 64Mhz для AU - STABLE)
	
	OSC.CTRL |= OSC_PLLEN_bm ; // enable the PLL...
	while( (OSC.STATUS & OSC_PLLRDY_bm) == 0 ){} // wait until stable

	// And now switch to the PLL as the clocksource
	CCP = CCP_IOREG_gc; // protected write follows	 
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}

void Config32MHzClock_External_XTAL_8Mhz_x_PLL4(void)
{
	// Use an external 8Mhz crystal and x 4 PLL to give a clock of 32Mhz
	// Enable the external oscillator
	OSC.XOSCCTRL = OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc ;
	OSC.CTRL |= OSC_XOSCEN_bm ;
	while( (OSC.STATUS & OSC_XOSCRDY_bm) == 0 ){} // wait until stable

	// Now configure the PLL to be eXternal OSCillator * 4
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 4 ; 
	
	//!!PS
	//x8 Тоже работает для AU серии т.е 64Mhz-64MIPs!!!  (на AVRFreaks пишут что 64Mhz для AU - STABLE)
	
	OSC.CTRL |= OSC_PLLEN_bm ; // enable the PLL...
	while( (OSC.STATUS & OSC_PLLRDY_bm) == 0 ){} // wait until stable

	// And now switch to the PLL as the clocksource
	CCP = CCP_IOREG_gc; // protected write follows	 
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}

void TCE0_init(void)
{
  // configure timer/counter E0 to overflow periodically
  // and trigger interrupt handler which used to Blink LED

  //TCE0.CTRLA = TC_CLKSEL_DIV1_gc;   // clk wo division (32Mhz timer FREQ)
  TCE0.CTRLA = TC_CLKSEL_DIV1024_gc;  // clk with 1024 division (31_250Hz Timer FREQ)

  //TCE0.INTCTRLA = 0x3; // hi level interrupt enable - Bit manipulating
  
  //TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc; // hi level interrupt enable - group command
  //TCE0.INTCTRLA = TC_OVFINTLVL_MED_gc; //med  level interrupt enable
  TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc; //low  level interrupt enable
  //TCE0.INTCTRLA = TC_OVFINTLVL_OFF_gc; //IRQ interrupt disable

  
  TCE0.PER = 31250; // PERIOD register: used for Clear counter and count from <0> - So we have 1Hz IRQ
  //TCE0.PER = 31250/2; // PERIOD register: used for Clear counter and count from <0> - So we have 2Hz IRQ
  //TCE0.PER = 31250/10; // PERIOD register: used for Clear counter and count from <0> - So we have 10Hz IRQ
  //TCE0.PER = 31250/100; // PERIOD register: used for Clear counter and count from <0> - So we have 100Hz IRQ
  //TCE0.PER = 31250/1000; // PERIOD register: used for Clear counter and count from <0> - So we have 1000Hz IRQ
  TCE0.CNT = 0;
}


void TCD0_init_micros(void)
{

  // Event Channel 0 MUX from SYSCLK/64
  EVSYS.CH0MUX = EVSYS_CHMUX_PRESCALER_32_gc; // Prescaler (from SYSCLK) divide by 32 so - 1us(for 32Mhz SYSCLK)!!

  // configure timer/counter D0 to counting micros() (like Arduino)

  TCD0.CNT = 0; // Считаем максимум до 65535us = ~ 65ms (Счетчики XMEGA 16-ти битные)
  //TCE0.CTRLA = TC_CLKSEL_DIV1_gc;   // clk wo division (32Mhz timer FREQ)
  TCD0.CTRLA = TC_CLKSEL_EVCH0_gc;  // clk from Event channel 0

  //TCE0.INTCTRLA = 0x3; // hi level interrupt enable - Bit manipulating
  
  //TCE0.INTCTRLA = TC_OVFINTLVL_HI_gc; // hi level interrupt enable - group command
  //TCE0.INTCTRLA = TC_OVFINTLVL_MED_gc; //med  level interrupt enable
  //TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc; //low  level interrupt enable
  //TCE0.INTCTRLA = TC_OVFINTLVL_OFF_gc; //IRQ interrupt disable

  
  //TCD0.PER = 31250; // PERIOD register: used for Clear counter and count from <0> - So we have 1Hz IRQ
  //TCE0.PER = 31250/2; // PERIOD register: used for Clear counter and count from <0> - So we have 2Hz IRQ
  //TCE0.PER = 31250/10; // PERIOD register: used for Clear counter and count from <0> - So we have 10Hz IRQ
  //TCE0.PER = 31250/100; // PERIOD register: used for Clear counter and count from <0> - So we have 100Hz IRQ
  //TCE0.PER = 31250/1000; // PERIOD register: used for Clear counter and count from <0> - So we have 1000Hz IRQ
}


//*********************USARTE0: BEGIN
/*
 * PS. Partially used code from BostonAndroid XMEGA code: <xmega-waveform-serial.c>
*/

// RX Buffer (Filled via RX IRQ)
#define rx_buf_MAX 32
char USARTE0_rx_buf[rx_buf_MAX];
uint8_t USARTE0_rx_buf_idx = 0;

void USARTE0_init(void)
{
	//USARTE0_TX pin is PE3
	PORTE.DIRSET = (1<<3); // To OUT
	PORTE.OUTSET = (1<<3); // Set TX pin to OUT with logick LVL <1>

	//USARTE0_RX pin is PE2
	PORTE.DIRCLR = (1<<2); // To IN (float in as default)
	PORTE.PIN2CTRL |= PORT_OPC_PULLUP_gc; // RX Input  PULL-UP (to avoid receive garbage on floating in)

	//Choose Baudrate  (From Boston Android XMEGA code)
	USARTE0.BAUDCTRLA = 207; // 9600b  (BSCALE=207,BSEL=0)
	//  USARTE0.BAUDCTRLA = 103; // 19200b  (BSCALE=103,BSEL=0)
	//  USARTE0.BAUDCTRLA = 34;  // 57600b  (BSCALE=34,BSEL=0)
	//  USARTE0.BAUDCTRLA = 33; USARTE0.BAUDCTRLB = (-1<<4); // 115.2kb (BSCALE=33,BSEL=-1)
	//  USARTE0.BAUDCTRLA = 31; USARTE0.BAUDCTRLB = (-2<<4); // 230.4kb (BSCALE=31,BSEL=-2)
	//  USARTE0.BAUDCTRLA = 27; USARTE0.BAUDCTRLB = (-3<<4); // 460.8kb (BSCALE=27,BSEL=-3)
	//  USARTE0.BAUDCTRLA = 19; USARTE0.BAUDCTRLB = (-4<<4); // 921.6kb (BSCALE=19,BSEL=-4)
	//  USARTE0.BAUDCTRLA = 1; USARTE0.BAUDCTRLB = (1<<4); // 500kb (BSCALE=19,BSEL=-4)
	//  USARTE0.BAUDCTRLA = 1;   // 1Mb (BSCALE=1,BSEL=0)
	
	//USARTE0.CTRLA = USART_RXCINTLVL_HI_gc; // enable RX interrupts, high level
	USARTE0.CTRLA = USART_RXCINTLVL_LO_gc; // enable RX interrupts, low level
	USARTE0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
}

// Blocking Read char from USART RX
unsigned char USARTE0_ReadChar(void)
{
	while(!(USARTE0.STATUS&USART_RXCIF_bm));  // wait for RX complete

  	return USARTE0.DATA;
}

// Check if data available in USART RX
unsigned char USARTE0_RX_Available(void)
{
	//return (USARTE0.STATUS&USART_RXCIF_bm);
	return USARTE0_rx_buf_idx;
}


void USARTE0_WriteChar(unsigned char data)
{
    USARTE0.DATA = data; // transmit ascii 3 over and over
	if(!(USARTE0.STATUS&USART_DREIF_bm))
		while(!(USARTE0.STATUS & USART_TXCIF_bm)); // wait for TX complete
  	USARTE0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
}

// For STDOUT via printf
int USARTE0_putch(char ch,FILE *stream)
{
    USARTE0.DATA = ch; // transmit ascii 3 over and over
	if(!(USARTE0.STATUS&USART_DREIF_bm))
		while(!(USARTE0.STATUS & USART_TXCIF_bm)); // wait for TX complete
  	USARTE0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag

    return 0;
}

// For STDIN TEMPLATE (not used here)
/*
int uart1_getch(FILE *stream)
{
   unsigned char ch;

   while (!(UCSR1A & (1<<RXC1)));
   ch=UDR1;  

   // Echo the Output Back to terminal
   uart1_putch(ch,stream);       

   return ch;
}
*/

// write out a simple '\0' terminated string
void USARTE0_WriteString(char *string)
{

    while(*string != 0)
	  USARTE0_WriteChar(*string++);
}

// write out a simple '\0' terminated string and print "\n\r" at end
void USARTE0_WriteLine(char *string)
{
   USARTE0_WriteString(string);
   USARTE0_WriteString("\n\r");

}

// write out a simple '\0' terminated string from Programm Space (for Save RAM)
void USARTE0_WriteString_P(PGM_P string)
{

	while(pgm_read_byte(string))
    {
	  USARTE0_WriteChar(pgm_read_byte(string++));
	}
}


// IRQ on USARTE0_RX
ISR(USARTE0_RXC_vect)
{
	//Send BACK echo_symbol + 1
	//USARTE0_WriteChar(USARTE0_ReadChar() + 1);	

	// Fill in RX buffer
	if (USARTE0_rx_buf_idx < (rx_buf_MAX - 1))
	{
		// Rx Buffer is OK
		USARTE0_rx_buf[USARTE0_rx_buf_idx++] = USARTE0_ReadChar();
	}
	else
	{
		// Rx Buffer is overflowed
		char dummy = USARTE0_ReadChar();
	}
}

//*********************USARTE0: END

void avr_init(void)
{
	//!! Set-up F_CPU: BEGIN
	//PS. Default FREQ wo settings 2Mhz from internal factory calibrated RC 

	//Config32MHzClock_Internal(); // INTERNAL RC 32Mhz factory calibrated with accuracy of 1% at 3V and 25°C
	Config32MHzClock_External_XTAL_8Mhz_x_PLL4(); // EXT XTAL 8Mhz to SYSCLOCK 32Mhz with PLLx4
	//Config32MHzClock_External_XTAL_16Mhz_x_PLL2(); // EXT XTAL 16Mhz to SYSCLOCK 32Mhz with PLLx2
	//!! Set-up F_CPU: END
	
	
	
	// make clkout on PORTC:7
	PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PC7_gc;
	PORTC.DIR = (1<<7); // clkout

	//TCC0 not used here
	/*
	// configure timer/counter0
	TCC0.CTRLA = 0x7;   // clk/1024
	TCC0.CNT = 0;       // reset count
	*/

	// configure PORTA:6 as output to LED  && All other pins of PORTA as input
	// Classic style
	/*PORTA.DIR |= (1<<LED);   // PORTA.6 is out
	PORTA.DIR &= ~(1<<SW1);   // PORTA.7 is in*/
	
	// XMEGA style
	PORTA.DIRSET = 1<<LED;   // PORTA.6 is out  - DIRSET устанавливает битовую маску на <1>
	PORTA.DIRCLR = 1<<SW1;   // PORTA.7 is in - DIRCLR  устанавливает битовую маску на <0>
	
	
	// additional cfg for SW1 input pin for pull-up:
	PORTA.PIN7CTRL |= PORT_OPC_PULLUP_gc; // Totempole w/ Pull-up on Input  PULL-UP
	//PORTA.PIN7CTRL |= PORT_OPC_PULLDOWN_gc; // Totempole w/ Pull-down on Input  PULL-DOWN
	//PORTA.PIN7CTRL |= PORT_OPC_TOTEM_gc;  // Totempole - FLOATING - DEFAULT
	
	//USARTE0 init
	USARTE0_init();
	
	//!!Redirect STDOUT (printf etc..)
	stdout = &USARTE0_stdout;
	
	// Template to redirect STDOUT&STDIN
	/*
	stdout = stdin = &uart1_str;
	*/

	
	
	//USARTE0_WriteString("Hello from XMEGA USARTE0!\r\n"); // Classic style to waste RAM Print Out
	//USARTE0_WriteString_P(PSTR("\r\nXMEGA USARTE0 RX IRQ example..\r\n")); // for Save RAM style Print Out
	
	//!! TCE0 setup for 1Hz IRQ
	// Not used here
	//TCE0_init();
	
	//Micros timer init
	TCD0_init_micros();
	
	
	//!!********************TCC0	as FREQ OUT (freq generator) : BEGIN
	//PC0 define (OCA) as Output
	PORTC.DIRSET = 0x01;	
	//Set Timerprescaler
	TCC0.CTRLA = TC_CLKSEL_DIV4_gc;
	// Select the output frequency and output enable Compare A
	TCC0.CTRLB = TC_WGMODE_FRQ_gc | TC0_CCAEN_bm;
	// Set comparison value of CCA
	
	//TCC0.CCA = 0xffff; // ~ 61Hz	T=250ns*65536
	//TCC0.CCA = 40000-1; // 100Hz 	T=250ns*40000
	//TCC0.CCA = 8000-1; // 500Hz 	T=250ns*4000
	//TCC0.CCA = 4000-1; // 1kHz 	T=250ns*4000
	//TCC0.CCA = 800-1; // 5kHz 	T=250ns*800
	//TCC0.CCA = 400-1; // 10kHz 	T=250ns*400
	//TCC0.CCA = 200-1; // 20kHz 	T=250ns*400
	//TCC0.CCA = 80-1; // 50kHz		T=250ns*80
	//TCC0.CCA = 40-1; // 100kHz	T=250ns*40
	//TCC0.CCA = 20-1; // 200kHz	T=250ns*20
	//TCC0.CCA = 8-1; // 500kHz 	T=250ns*8
	//TCC0.CCA = 0x3; // 1Mhz		T=250ns*4
	//TCC0.CCA = 0x1; // 2Mhz		T=250ns*2
	TCC0.CCA = 0x0; // 4Mhz i.e 	T=250ns*1	
	
	//!!PS Formula for FREQ OUT with SYSCLOCK = 32Mhz && TCC0 prescaler 4
	// Step INC 250ns
	/*
	f = system clock / (2 * prescaler * (CCA + 1))
	In our example, the frequency is then as follows:
	fmin = 32MHz / (2 * 4 * (65535 + 1) = 4Mhz / 65536 = ca. 61 Hz	
	fmax = 32MHz / (2 * 4 * (0 + 1) = 4Mhz	/ 1 = 4Mhz
	*/
	//!!********************TCC0	as FREQ OUT (freq generator) : END
	
	//!!********************TCC1	as FREQ IN (freq counter) : BEGIN
	//!!Set-UP PIN for FREQ IN
	// PC1 for event trigger on Falling EDGE
	// Configure edge
	PORTC.PIN1CTRL = PORT_ISC_FALLING_gc;
	//PDC Pullup mode
	PORTC.PIN1CTRL |= PORT_OPC_PULLUP_gc;
	//Define PC1 as input
	PORTC.DIRCLR = (1<<1);

	//!!Set-UP Timer TCC1 for FREQ IN
	//Set PC1 as Event Channel1
	EVSYS.CH1MUX = EVSYS_CHMUX_PORTC_PIN1_gc;
	//Set timer mode on UPDOWN and selekt CH1 as an event channel
	TCC1.CTRLD = TC_EVACT_UPDOWN_gc | TC_EVSEL_CH1_gc;
	// Sets timer top value and therefore also establish
	// how many pulses the interrupt routine
	// runs (Here 65535 times fire to IRQ exec)
	TCC1.PER = 0xFFFF;
	//Define timer interrupt as LOW priority
	//TCC1.INTCTRLA = TC_OVFINTLVL_LO_gc; Try NOT used here
	//Clock Source Select in the event Event Channel 1
	TCC1.CTRLA = TC_CLKSEL_EVCH1_gc;	
	//!!********************TCC1	as FREQ IN (freq counter) : END
	
	// ADC PA2 init 2V-REF 11bit signed mode
	ADC_init_11bit_signed();
	
	//!! Enable LO LVL IRQ && Round robin mechanism for LOW LEVEL IRQ
	//PMIC.CTRL = PMIC_HILVLEN_bm; // enable high level interrupts
	PMIC.CTRL =  PMIC_RREN_bm | PMIC_LOLVLEN_bm;
	sei(); // enable interrupts
}

//TCE0 IRQ for overflow 
ISR(TCE0_OVF_vect)
{
	//Here every 1sec/0.5sec/100ms/10ms/1ms
	PORTA.OUTTGL = 1<<LED; // TOGGLE LED
	uptime++;
}

// RAM Memory usage test
int freeRam (void) {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

//*******************ADC Init: BEGIN

uint8_t ReadSignatureByte(uint16_t Address)
{
    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    uint8_t Result;
    __asm__ ("lpm %0, Z\n" : "=r" (Result) : "z" (Address));
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;
    return Result;
}

// GOOD считаем +-2**11 = +-2048 MAX single-ended c опорой 1/1.6/2V
// погрешность - ОК ~ 1% и 0 ~ соответствует.
// !! Шаг замера 1mv для диапазона [0-2V] c погрешностью ~1%
//!! Предварительная калибровка <0> НЕ НУЖНА (в отличии от ADC_init_8bit && ADC_init_12bit)
void ADC_init_11bit_signed(void)
{
	    // Переназначаем ADC входа в режим входа && Отключаем цифровой режим
	    PORTA.DIRCLR = 1<<2; //(PORTA2 as ADC iniput)
	    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc; // Disable Digital Input Buffer for PA2

        ADCA.CALL = ReadSignatureByte(0x20) ; //variant1 - ADC Calibration Byte 0
        ADCA.CALH = ReadSignatureByte(0x21) ; //variant1 - ADC Calibration Byte 1
 
		//ADCA.CALL = util_read_calib_byte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );//variant2 - ADC Calibration Byte 0
		//ADCA.CALH = util_read_calib_byte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );//variant2 - ADC Calibration Byte 1
        
        _delay_ms(1); // Wait at least 25 clocks


        ADCA.CTRLA = ADC_ENABLE_bm ; // Enable the ADC

        //!!********************Выбор ADC mode: BEGIN

        //ADCA.CTRLB = ADC_RESOLUTION_8BIT_gc; // Unsigned Mode 8 bit 
        //!! NOT ACCURATE MODE
        //!! Меряем 8bit unsigned right adjustened - при этом ~10 единиц снизу диапазона у <0> мертвые - НЕПРИЯТНАЯ ОСОБЕННОСТЬ ADC XMEGA)

        //ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc; // Unsigned Mode 12 bit 
        //!! NOT VARY GOOD MODE
        //!! Меряем (режим по умолчанию) 12bit unsigned right adjustened - при этом 200 единиц снизу диапазона у <0> мертвые - НЕПРИЯТНАЯ ОСОБЕННОСТЬ ADC XMEGA)

        ADCA.CTRLB = ADC_CONMODE_bm|ADC_RESOLUTION_12BIT_gc; // Signed Mode 11 bit 
        //!! GOOD MODE
        //!! Меряем 11bit signed right adjustened - при этом меряем +- 2^^11 бит от <0> -("Мертвая" зона см. <Unsigned Mode 12 bit> отсутствует!!)
        //!! Но отриц. напряжение не меряет - (подавал от AAA батарейки +-1.5V , В отриц. полярность показывает ~500mV)
        
        //!!********************Выбор ADC mode: END

        //!!********************Выбор AREF (опора для ADC): BEGIN
        //ADCA.REFCTRL = ADC_REFSEL_INT1V_gc; // Internal 1v ref
		//!! Т.к. считаем 2*11=> единица измерения 0.488mv - GOOD
		// 2.5% погрешности от 0.5mV - Шаг 0.5mv/Интервал [0-1V]
		
		ADCA.REFCTRL = ADC_REFSEL_VCC_gc; // Internal VCC / 1.6  = 2V (When Vcc=3.3V) 
		//!!(Если точнее 2.0625 ) Т.к. считаем 2**11=> единица измерения 1.007mv = Very GOOD 
		// 0.7% погрешности от 1mv! - Шаг 1 mv очень удобен/Интервал[0-2V]
		
		//ADCA.REFCTRL = ADC_REFSEL_VCCDIV2_gc;// Internal VCC / 2 = 1.65V (When Vcc=3.3V)
		//!! Т.к. считаем 2*11=> единица измерения=> единица измерения 0.806mv = NOT BAD 
		// 0.75% погрешности от 0.8mv - Но шаг 0.8mV не удобен/Интервал[0-1.65V]

 
		//ADCA.REFCTRL = ADC_REFSEL_AREFA_gc; //External reference on PORT A.0
		//!! При опоре от внешнего прецизионного стабилитрона 2.56В
		//!! Т.к. считаем 2*11=> единица измерения-шаг 1.25mV/Интервал[0-2.56V]
        //!!********************Выбор AREF (опора для ADC): END
        
        
        ADCA.EVCTRL = ADC_EVACT_NONE_gc ; // no events
        ADCA.PRESCALER = ADC_PRESCALER_DIV128_gc ; // ~ 30us for 1 samling - OK
		//ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc; // ~15us for 1 sampling - OK
		//ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc; // ~9us for 1 sampling - OK
		//ADCA.PRESCALER = ADC_PRESCALER_DIV16_gc; // ~5us for 1 sampling (Здесь и далее уже играет роль импенданс входной цепи начали "заваливать результат на ~ 2%" на сопротивлении 10К)
		//ADCA.PRESCALER = ADC_PRESCALER_DIV4_gc; // ~ 1us for 1 sampling - Здесь надо мерять через OP-AMP т.к. на сопротивлении 10К "завалено ~30% сигнала"
		
        
       

		// For READ ADC.CH2 - PA.2 - as default
		ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc ; // Gain = 1, Single Ended
		ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN2_gc; // (2<<3) Input PIN- PA2 for ADCA.CH0
		ADCA.CH0.INTCTRL = ADC_CH_INTLVL_OFF_gc ; // No interrupt
}
 
//READ ADC with auto setup CH
inline static uint16_t ReadADC(uint8_t Channel, uint8_t ADCMode) // Mode = 1 for single ended, 0 for internal
{
    ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADCMode ; // Gain = 1, Single Ended
    ADCA.CH0.MUXCTRL = (Channel<<3);
    ADCA.CH0.INTCTRL = ADC_CH_INTLVL_OFF_gc ; // No interrupt
    //ADCA.CH0.SCAN Another bogus register
    //for(uint8_t Waste = 0; Waste<2; Waste++)
    //{
        ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
        while (ADCA.INTFLAGS==0) ; // Wait for complete
        ADCA.INTFLAGS = ADCA.INTFLAGS ;// To Clear Interrupt Flag
    //}
    return ADCA.CH0RES ;
}

//READ ADC with auto setup channel & Average result from 16 times
inline static uint16_t ReadADC_16AVG(uint8_t Channel, uint8_t ADCMode) // Mode = 1 for single ended, 0 for internal
{
    long sum;
    uint8_t j;
    
    ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADCMode ; // Gain = 1, Single Ended
    ADCA.CH0.MUXCTRL = (Channel<<3);
    ADCA.CH0.INTCTRL = ADC_CH_INTLVL_OFF_gc ; // No interrupt
	/*
	 * 1 times reading
	ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
	while (ADCA.INTFLAGS==0) ; // Wait for complete
	ADCA.INTFLAGS = ADCA.INTFLAGS ;// To Clear Interrupt Flag
	return ADCA.CH0RES ;
	*/

	// Take multiple readings and average them out
	sum=0;
	for (j=0; j<16; j++) 
	{
		ADCA.CH0.CTRL |= ADC_CH_START_bm; // Start conversion
		while (ADCA.INTFLAGS==0) ; // Wait for complete
		ADCA.INTFLAGS = ADCA.INTFLAGS ;// To Clear Interrupt Flag
		sum += ADCA.CH0RES;
	}
	return (sum/16);
}

//*******************ADC Init: END

// Ardino <map> function
/*
map(value, fromLow, fromHigh, toLow, toHigh)
Description

Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, a value of fromHigh to toHigh, values in-between to values in-between, etc.

Does not constrain values to within the range, because out-of-range values are sometimes intended and useful. The constrain() function may be used either before or after this function, if limits to the ranges are desired.

Note that the "lower bounds" of either range may be larger or smaller than the "upper bounds" so the map() function may be used to reverse a range of numbers, for example

y = map(x, 1, 50, 50, 1);

The function also handles negative numbers well, so that this example

y = map(x, 1, 50, 50, -100);

is also valid and works well.

The map() function uses integer math so will not generate fractions, when the math might indicate that it should do so. Fractional remainders are truncated, and are not rounded or averaged.

Parameters

value: the number to map

fromLow: the lower bound of the value's current range

fromHigh: the upper bound of the value's current range

toLow: the lower bound of the value's target range

toHigh: the upper bound of the value's target range

Returns

The mapped value. 
*/
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void)
{
    // INIT MCU
	avr_init();
	uint32_t prev_time = 0;
	float uptime_float = 0;
	//char msg[64] = "Unknown time\r\n";
	

	// Print program metrics
	/*
	USARTE0_WriteString_P(str_prog_name);// Название программы
	USARTE0_WriteString_P(PSTR("Compiled at: "));
	USARTE0_WriteString_P(compile_time); // Время компиляции
	USARTE0_WriteString_P(PSTR(" ")); 
	USARTE0_WriteString_P(compile_date); // Дата компиляции 
	USARTE0_WriteString_P(PSTR("\r\n"));
	*/
	//Working via printf
	printf_P(str_prog_name);// Название программы
	printf_P(PSTR("Compiled at: "));
	printf_P(compile_time); // Время компиляции
	printf_P(PSTR(" ")); 
	printf_P(compile_date); // Дата компиляции 
	printf_P(PSTR("\r\n"));
	
	
	// FreeRam DEBUG
	//sprintf_P(msg, PSTR(">>>Free RAM is: %u bytes\r\n\r\n"), freeRam());
	//USARTE0_WriteString(msg);
	printf_P(PSTR(">>>Free RAM is: %u bytes\r\n\r\n"), freeRam());
	
	
//***************************Profiling micros: BEGIN
//Accurate Measure time interval from 1us till ~ 65.5ms
	uint16_t _micros;
	
	
	// Profiling time for 10us
	_micros = TCD0.CNT;
	_delay_us(10);
	_micros = TCD0.CNT - _micros;
	printf_P(PSTR(">>>10us is: %u us\r\n"), _micros);


	// Profiling time for 100us
	_micros = TCD0.CNT;
	_delay_us(100);
	_micros = TCD0.CNT - _micros;
	printf_P(PSTR(">>>100us is: %u us\r\n"), _micros);


	// Profiling time for 1ms
	_micros = TCD0.CNT;
	_delay_ms(1);
	_micros = TCD0.CNT - _micros;
	printf_P(PSTR(">>>1ms is: %u us\r\n"), _micros);


	// Profiling time for 65ms
	_micros = TCD0.CNT;
	_delay_ms(65);
	_micros = TCD0.CNT - _micros;
	printf_P(PSTR(">>>65ms is: %u us\r\n"), _micros);

//***************************Profiling micros: END
    
    // ADC measure VARs
    int16_t ADC_Reading;
    int16_t MAP_Reading;
    uint16_t FREQ_OUT;
	
// FREQ COUNT: BEGIN
// Measure COUNT from PC1 Input
	while(1)
	{
		// Setup FREQ Counter Timer TCC1 to start Count
		TCC1.CTRLA = TC_CLKSEL_OFF_gc;// Stop TCC1
		TCC1.CNT = 0; // CLEAR Counter (16bit)
		TCC1.INTFLAGS |= TC1_OVFIF_bm; // Clear OVF IRQ Flag for TCC1
		TCC1.CTRLA = TC_CLKSEL_EVCH1_gc;// Run TCC1 from Event CH1
		
		// PAUSE 1sec
		uint8_t i = 100;
		while(i--)
		{
			_micros = TCD0.CNT;
			while ((TCD0.CNT - _micros)<10000) //Wait 10_000us*100 = 1 sec
			{
				asm("nop");			
			}//while ((TCD0.CNT - _micros)<1000) //Wait 10_000us*100 = 1 sec
		}//while(i--)
		
		//!!Debug only
		//PORTA.OUTTGL = 1<<LED; // TOGGLE LED

		TCC1.CTRLA = TC_CLKSEL_OFF_gc;// Stop TCC1
		//Check TCC1 Overflow
		if (TCC1.INTFLAGS &  TC1_OVFIF_bm)
		{
			// Overflowed when 1sec pause => FREQ > 65535Hz
			//!! Debug only
			//printf_P(PSTR("ERROR FREQ IN on PC1: >65535Hz!!\r\n"), TCC1.CNT);
			
			// Поэтому замер производим ПОВТОРНО в окне 1ms, измеряемая частота 16Mhz - 66Khz

			// Setup FREQ Counter Timer TCC1 to start Count
			TCC1.CTRLA = TC_CLKSEL_OFF_gc;// Stop TCC1
			TCC1.CNT = 0; // CLEAR Counter (16bit)
			TCC1.INTFLAGS |= TC1_OVFIF_bm; // Clear OVF IRQ Flag for TCC1
			TCC1.CTRLA = TC_CLKSEL_EVCH1_gc;// Run TCC1 from Event CH1

			//PAUSE 1msec
			_micros = TCD0.CNT;
			while ((TCD0.CNT - _micros)<1000) //Wait 1000us = 1 msec
			{
				asm("nop");			
			}//while ((TCD0.CNT - _micros)<1000) //Wait 1000us
			
			TCC1.CTRLA = TC_CLKSEL_OFF_gc;// Stop TCC1
			
			//Check TCC1 Overflow
			if (TCC1.INTFLAGS &  TC1_OVFIF_bm)
			{
				printf_P(PSTR("ERROR FREQ IN on PC1: >65535kHz!!\r\n"), TCC1.CNT);
			}
			else
			{
				printf_P(PSTR("FREQ IN on PC1: %u kHz\r\n"), TCC1.CNT);
			}

		}//if (TCC1.INTFLAGS &  TC1_OVFIF_bm)
		else
		{
			// Измеряемая частота 65535Hz - 1Hz
			// No overflow when 1sec pause => FREQ =< 65535Hz
			printf_P(PSTR("FREQ IN on PC1: %u Hz\r\n"), TCC1.CNT);
		}//else
		//_delay_ms(1000); // 1 Measure per 1 sec
		
		//************ READ ADC PA2 value: BEGIN
		ADC_Reading = ReadADC_16AVG(2, 1); // CH2 & Ext Signal
		printf_P(PSTR("ADC PA.2(int) is: %d mV\r\n"), ADC_Reading); // Для опоры 3.3/1.6 - 1 единица измерения = 1 mv (3.3/1.6/2048)
		// MAP result
		// Check Interval inputs exceed MIN && MAX value
		if(ADC_Reading < 10) //MIN EXCEED
			MAP_Reading = 10;
		else if ((ADC_Reading > 1400)) //MAX EXCEED
			MAP_Reading = 1400;
		else // GOOD value
			MAP_Reading = ADC_Reading;
		// Т.к. резистор обычный - не многоооборотистый для плавности перестройки частоты диапазон 4Mhz-61Hz разбит на 4 поддиапазона для плавности перестройки
		//*** 4 диапазона перестройки от 4Mhz до 61Hz:
		//*** 4Mhz-200kHz/200kHz-10kHz/10kHz-500Hz/500Hz-61Hz
		FREQ_OUT = map(MAP_Reading, 10, 1400, 0, 20-1); // 4Mhz - 200kHz
		//FREQ_OUT = map(MAP_Reading, 10, 1400, 20-1, 400-1); // 200kHz - 10kHz
		//FREQ_OUT = map(MAP_Reading, 10, 1400, 400-1, 8000-1); // 10kHz - 500Hz
		//FREQ_OUT = map(MAP_Reading, 10, 1400, 8000-1, 65535); // 500Hz - 61Hz
		printf_P(PSTR(">>>FREQ_OUT (0-65535) is: %u\r\n\r\n"), FREQ_OUT);
		//************ READ ADC PA2 value: END
		
		//************ FREQ OUT Value set: BEGIN
		//Set Timerprescaler
		TCC0.CTRLA = TC_CLKSEL_OFF_gc; // STOP Timer for FREQ OUT
		TCC0.CCA = FREQ_OUT; // Value [0-65535] => [4Mhz-61Hz] FREQ
		TCC0.CTRLA = TC_CLKSEL_DIV4_gc; // PRESCALER for 4
		//************ FREQ OUT Value set: END
		
	}//while(1)

// FREQ COUNT: END
	
	while(1);
	
    return(0);
}


