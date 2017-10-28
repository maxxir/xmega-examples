/*
Minimal template for M_128 16Mhz
(c) Ibragimov M. Russia Togliatty 08/09/2014

LED PIN PD5 - flashing on "0"
UART1 is used: TX1/PD3, RX1/PD2
millis engine ON (Arduino-LIKE)
ADC5/PF5 read value
WatchDog ON 2sec
*/
// To avoid poisoned errors
#define __AVR_LIBC_DEPRECATED_ENABLE__

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <compat/deprecated.h>  //sbi, cbi etc..
#include "avr/wdt.h" // WatchDog
#include <stdio.h>  // printf etc..
//#include <stdlib.h> // itoa etc..

//Defines
#define LED 5
#define LED_PORT PORTD
#define LED_DDR DDRD
#define LED_ON  (LED_PORT &= ~(1<<(LED)))
#define LED_OFF (LED_PORT |=  (1<<(LED)))
#define LED_TGL (LED_PORT ^= (1<<(LED)))

//*********Global vars
#define TICK_PER_SEC 1000UL
volatile unsigned long _millis; // for millis tick !! Overflow every ~49.7 days

//*********Program metrics
const char compile_date[] PROGMEM    = __DATE__;     // Mmm dd yyyy - Дата компиляции
const char compile_time[] PROGMEM    = __TIME__;     // hh:mm:ss - Время компиляции
const char str_prog_name[] PROGMEM   = "\r\nAtMega128 v1.0 Base Template 08/09/2014\r\n"; // Program name


//FUNC headers
static void avr_init(void);
void timer0_init(void);
static inline unsigned long millis(void);


//******************* MILLIS ENGINE: BEGIN
SIGNAL (SIG_OUTPUT_COMPARE0)
{
	// Compare match Timer0
	// Here every 1ms
	_millis++; // INC millis tick
	// Тест мигаем при в ходе в прерывание
	// 500Hz FREQ OUT
	// LED_TGL;
}

static inline unsigned long millis(void)
{
	unsigned long i;
	cli();
	// Atomic tick reading
	i = _millis;
	sei();
	return i;
}
//******************* MILLIS ENGINE: END

//***************** UART1: BEGIN
#define BAUD_RATE 19200
void uart1_init(void)
{
  UBRR1H = (((F_CPU/BAUD_RATE)/16)-1)>>8;		// set baud rate
  UBRR1L = (((F_CPU/BAUD_RATE)/16)-1);
  UCSR1B = (1<<RXEN0)|(1<<TXEN0); 				// enable Rx & Tx
  UCSR1C=  (1<<UCSZ01)|(1<<UCSZ00);  	        // config USART; 8N1
}

/*
Unused here
void uart_flush(void)
{
  unsigned char dummy;

  while (UCSR1A & (1<<RXC1)) dummy = UDR1;
}
*/

int uart1_putch(char ch,FILE *stream)
{
   if (ch == '\n')
    uart1_putch('\r', stream);

   while (!(UCSR1A & (1<<UDRE1)));
   UDR1=ch;

   return 0;
}

int uart1_getch(FILE *stream)
{
   unsigned char ch;

   while (!(UCSR1A & (1<<RXC1)));
   ch=UDR1;  

   /* Echo the Output Back to terminal */
   uart1_putch(ch,stream);       

   return ch;
}

// Assign I/O stream to UART
FILE uart1_str = FDEV_SETUP_STREAM(uart1_putch, uart1_getch, _FDEV_SETUP_RW);

//***************** UART1: END

//***************** ADC: BEGIN
#ifndef ADC_DIV
//12.5MHz or over use this ADC reference clock
#define ADC_DIV (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) //:128 ADC Prescaler
#endif

#ifndef ADC_REF
// vcc voltage ref default
#define ADC_REF (1<<REFS0)
#endif

void adc_init(void)
{
	ADCSRA = 0;
	ADCSRA |= (ADC_DIV);    // ADC reference clock
	ADMUX |= (ADC_REF);     // Voltage reference 
	ADCSRA |= (1<<ADEN);    // Turn on ADC
	ADCSRA |= (1<<ADSC);    // Do an initial conversion because this one is the
	                        // slowest and to ensure that everything is up
							// and running
}

uint16_t adc_read(uint8_t channel)
{
	ADMUX &= 0b11100000;                    //Clear the older channel that was read
	ADMUX |= channel;                //Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                //Starts a new conversion
	while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done

	return ADCW;                    //Returns the ADC value of the chosen channel
}
//***************** ADC: END


int main(void)
{
    // INIT MCU
	avr_init();
	
	// Print program metrics
	printf_P(str_prog_name);// Название программы
	printf_P(PSTR("Compiled at: "));
	printf_P(compile_time); // Время компиляции
	printf_P(PSTR(" ")); 
	printf_P(compile_date); // Дата компиляции 
	printf_P(PSTR("\r\n"));
	
	sei(); //re-enable global interrupts

	//Short Blink LED twice on startup
	unsigned char i = 2;
	while(i--)
	{
		LED_ON;
		_delay_ms(100);
		LED_OFF;
		_delay_ms(400);
		wdt_reset();
	}

    unsigned long prev_millis = 0;
	unsigned long uptime = 0;
    for(;;)
    {
        // Tasks here.
		//LED_TGL;
		//_delay_us(10); // 50Khz FREQ OUT
		//_delay_ms(1);  // 500Hz FREQ OUT
		
		if((millis()-prev_millis)>TICK_PER_SEC)
		{
			wdt_reset(); // WDT reset every sec
			// 1sec tick
			prev_millis = millis();
			LED_TGL;
			printf_P(PSTR("Uptime %lu sec\r\n"), uptime++);
			printf_P(PSTR("ADC5: %d\r\n"), adc_read(5));
		}
    }
    
    return(0);
}


// Timer0
// 1ms IRQ
// Used for millis() timing
void timer0_init(void)
{
	TCCR0 = (1<<CS02)|(1<<WGM01); //TIMER0 SET-UP: CTC MODE & PS 1:64
	OCR0 = 249; // 1ms reach for clear (16mz:64=>250kHz:250-=>1kHz)
	TIMSK |= 1<<OCIE0;	 //IRQ on TIMER0 output compare 
}

static void avr_init(void)
{
    // Initialize device here.
	// WatchDog INIT
	wdt_enable(WDTO_2S);  // set up wdt reset interval 2 second
	wdt_reset(); // wdt reset ~ every <2000ms 
	
	
    LED_DDR  |= 1<<LED; // LED is output
	LED_OFF; // LED is OFF

    timer0_init();// Timer0 millis engine init
	
	//UART1 init
	// Define Output/Input Stream
	stdout = stdin = &uart1_str;
	// Initial UART Peripheral
	uart1_init();
	
	//ADC init
	adc_init();
	adc_read(5); //Dummy read
    return;
}
