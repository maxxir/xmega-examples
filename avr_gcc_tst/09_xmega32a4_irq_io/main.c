/*
+ PA.7 IRQ sense BOTH edges (Rise & Fall)
* PS. to IRQ for input to EVERY PORT i.e: PORTx_INT0_vect/PORTx_INT1_vect

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
const char str_prog_name[] PROGMEM   = "\r\n\r\nXMEGA32A4U IRQ IO 15/01/2015..\r\n"; // Program name

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
	
	
	// additional cfg for SW1 input pin for pull-up && IRQ-EVENT for falling-rising edge:
	//PORTA.PIN7CTRL |= PORT_OPC_PULLUP_gc; // Totempole w/ Pull-up on Input  PULL-UP
	//PORTA.PIN7CTRL |= PORT_OPC_PULLDOWN_gc; // Totempole w/ Pull-down on Input  PULL-DOWN
	//PORTA.PIN7CTRL |= PORT_OPC_TOTEM_gc;  // Totempole - FLOATING - DEFAULT
	PORTA.PIN7CTRL= PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc; // Totem-pole with pull-up, input sense both edges
	
	//PA7 via INT0
	PORTA.INT0MASK = 1<<7;
	//INT LVL LO for PORTA INT0
	PORTA.INTCTRL |= PORT_INT0LVL_LO_gc;	


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
	TCE0_init();
	
	//Micros timer init
	TCD0_init_micros();	
	
	//!! Enable LO LVL IRQ && Round robin mechanism for LOW LEVEL IRQ
	//PMIC.CTRL = PMIC_HILVLEN_bm; // enable high level interrupts
	PMIC.CTRL =  PMIC_RREN_bm | PMIC_LOLVLEN_bm;
	sei(); // enable interrupts
}

//TCE0 IRQ for overflow 
ISR(TCE0_OVF_vect)
{
	//Here every 1sec/0.5sec/100ms/10ms/1ms
	//PORTA.OUTTGL = 1<<LED; // TOGGLE LED
	uptime++;
}

ISR(PORTA_INT0_vect){
	// PORTA.7 IN rising-falling edges sensing
	// && transfer it state to LED out
	if (PORTA.IN & (1<<7))
	{
		// SW is unpressed - LED OFF
		PORTA.OUTCLR = 1<<6;
		printf_P(PSTR("Button is released\r\n"));
	}
	else
	{
		// SW is pressed - LED ON
		PORTA.OUTSET = 1<<6;
		printf_P(PSTR("Button is pressed\r\n"));
	}
}
// RAM Memory usage test
int freeRam (void) {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
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
	
	
	while(1);
	
    for(;;)
    {
		//Nothing to do
		//asm("nop");
		
		// Print OUT second tick & Received from RX
		if (prev_time != uptime)
		{
			// Here every second
			//!! Uptime Handle
			prev_time = uptime;
			//sprintf(msg, "Uptime is: %lu sec\r\n", prev_time);
			//USARTE0_WriteString(msg);
			printf_P(PSTR("Uptime is: %lu sec\r\n"), prev_time);
			uptime_float += 0.01;
			
			//!! Don't forget correct <makefile> PRINTF_LIB for:
			/*
			PRINTF_LIB = $(PRINTF_LIB_FLOAT)
			*/
			printf_P(PSTR("Uptime float is: %.2f \r\n"), uptime_float);
			
			//!! Received from RX Handle
			// Check received index
			if (USARTE0_rx_buf_idx > 0)
			{
				cli(); // Disable IRQ
				static char rx_buf[rx_buf_MAX+1];
				uint8_t idx = USARTE0_rx_buf_idx;
				memset(rx_buf, 0, sizeof(rx_buf)); // Fill RX_Buffer with Zeros
				memcpy(rx_buf, USARTE0_rx_buf, idx); // Transfer RX data to Temporary buffer
				USARTE0_rx_buf_idx = 0; // Set zero Received RX counter
				sei(); // Enable IRQ
				
				// Print OUT RX Received data
				//sprintf(msg, ">>RX: %s\r\n", rx_buf);
				//USARTE0_WriteString(msg);
				printf_P(PSTR(">>RX: %s\r\n"), rx_buf);
			}
		}
		/*
		// Not used because USARTE0_RX via IRQ
		else
		{
			// Check receive data from USARTE0 RX
			if (USARTE0_RX_Available())
			{
				//Print OUT recieved from USARTE0 RX
				rx_char[0] = USARTE0_ReadChar();
				sprintf(msg, ">> %s\r\n", rx_char);
				USARTE0_WriteString(msg);
			}
		}
		*/
    }
    return(0);
}


