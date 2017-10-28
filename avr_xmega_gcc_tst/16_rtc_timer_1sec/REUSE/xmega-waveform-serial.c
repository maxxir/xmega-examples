#include <stdio.h>
#include <avr\io.h>
#define F_CPU 32000000UL
#include <util\delay.h>
#include <math.h>

#define VERSION "0.0.1"

void GenerateArbWave12(int *data, int len, unsigned long int freq);
void SetWaveFreq(unsigned long int freq);
void SetWaveCount(char count);
void LoadSineWave(int len);
void LoadTriangleWave(int len);
void LoadRampWave(int len);
void LoadSquareWave(int len);
void Config32MHzClock(void);

void UsartWriteChar(unsigned char data);
unsigned char UsartReadChar(void);
void UsartWriteString(char *string);
void UsartWriteLine(char *string);
void Error(char *string);
void PrintUsage();
int IsAlpha(char val);
void ParseCommand(char *string);
int GetParams(char *string, unsigned int *Params);
unsigned int StringToInt(char *string);
void IntToString(int value, char *string);

void DoSineWave(unsigned int freq);
void DoTriangleWave(unsigned int freq);
void DoRampWave(unsigned int freq);
void DoSquareWave(unsigned int freq);
void DoArbWave(unsigned int freq, unsigned int samples, int *data);
void DoNCycles(char num);

volatile int data12[100];
volatile int gWaveNumSamp=50;
volatile int gWaveCount=0;


int main(void)
{
  int Reading, index;
  char data;
  char buffer[100];

  Config32MHzClock();

  CLK.PSCTRL = 0x00; // no division on peripheral clock

  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); // clkout
  TCC0.CTRLA = 4;

// setup DAC output on PORTB:1 as GND reference
  PORTB.DIR |= 0x02;
  PORTB.OUT &= 0xFD;


/*
// setup ADC input on PORTA:0-3 (0=hi, 1=hi, 2=samp, 3=gnd)
// and power PORTA:1 to create voltage divider
  PORTA.DIR = 0xB;
  PORTA.OUT = 0x3;

// setup adc for single ended one shot mode
  ADCA.CTRLA |= 0x1;       // enable adc
  ADCA.CTRLB = 0x4;        // set 8 bit conversion
  ADCA.CH0.CTRL = 0x1;     // single ended
  ADCA.CH0.MUXCTRL = 0x10; // PORTA:2
  ADCA.REFCTRL = 0x20;     // reference is PORTA:0
  ADCA.PRESCALER = 0x5;    // peripheral clk/128
*/

  LoadSineWave(gWaveNumSamp);

/*
  // startup in 10kHz
  GenerateArbWave12(data12,gWaveNumSamp*2,10000);

  while(1)
  {
  // read adc to determine waveform freq
    ADCA.CTRLA |= 0x4;               // start conversion ch0
    while(!ADCA.CH0.INTFLAGS);       // wait for conversion complete flag
    ADCA.CH0.INTFLAGS = 0;           // clear int flags  
	Reading = ADCA.CH0RESL;          // read 8 bit value from POT
    SetWaveFreq((Reading*Reading)+1); // set freq
    _delay_ms(100); 
  };
  */


  // configure PORTF, USARTF0 (PORTF:3=Tx, PORTF:2=Rx) as asynch serial port
  // This will connect to the USB-Serial chip on EVAL-USB boards
  // For other boards rewrite all occurences of USARTF0 below with USARTE0
  // then you can use PORTE:2,3 as asynch serial port (EVAL-01, EVAL-04 boards)
  PORTF.DIR |= (1<<3) | (1<<0); // set PORTF:3 transmit pin as output
  PORTF.OUT |= (1<<3);          // set PORTF:3 hi 
  USARTF0.BAUDCTRLA = 207; // 9600b  (BSCALE=207,BSEL=0)
//  USARTF0.BAUDCTRLA = 103; // 19200b  (BSCALE=103,BSEL=0)
//  USARTF0.BAUDCTRLA = 34;  // 57600b  (BSCALE=34,BSEL=0)
//  USARTF0.BAUDCTRLA = 33; USARTF0.BAUDCTRLB = (-1<<4); // 115.2kb (BSCALE=33,BSEL=-1)
//  USARTF0.BAUDCTRLA = 31; USARTF0.BAUDCTRLB = (-2<<4); // 230.4kb (BSCALE=31,BSEL=-2)
//  USARTF0.BAUDCTRLA = 27; USARTF0.BAUDCTRLB = (-3<<4); // 460.8kb (BSCALE=27,BSEL=-3)
//  USARTF0.BAUDCTRLA = 19; USARTF0.BAUDCTRLB = (-4<<4); // 921.6kb (BSCALE=19,BSEL=-4)
//  USARTF0.BAUDCTRLA = 1; USARTF0.BAUDCTRLB = (1<<4); // 500kb (BSCALE=19,BSEL=-4)
//  USARTF0.BAUDCTRLA = 1;   // 1Mb (BSCALE=1,BSEL=0)

  USARTF0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART


  UsartWriteString("\n\r\n\rXmega EVAL-USB Waveform Gen Terminal\n\rv ");
  UsartWriteString(VERSION);
  UsartWriteString("\n\r");
  PrintUsage();
  UsartWriteString(": ");
  while(1)
  {
    data=UsartReadChar(); // read char
	// check for carriage return and try to match/execute command
	if((data == '\r')||(index==sizeof(buffer)))
	{
	    PORTF.OUT ^= (1<<0);      // switch LED
		buffer[index]=0;          // null terminate
		index=0;                  // reset buffer index
		UsartWriteString("\n\r"); // echo newline
		ParseCommand(buffer);     // attempt to parse command
    	  UsartWriteString(": ");
	}
	else if(data==8)              // backspace character
	{
	    if(index>0)
	        index--;                  // backup one character
		UsartWriteChar(data);
	}
	else
	{
	    buffer[index++]=data;
		UsartWriteChar(data);
	};

//	UsartWriteChar(data); // write char
//	_delay_ms(100);
//	PORTF.OUT ^= (1<<0); // toggle LED

  };
};

void PrintUsage()
{
  UsartWriteLine("==============================================================================");
  UsartWriteLine("h                          : Help");
  UsartWriteLine("b <freq> <num>             : Blink red user led using _delay_ms() function");
  UsartWriteLine("d <ch> <val> <ref>         : Output 12bit val on DAC channel");
  UsartWriteLine("                             use ref (0=1V,1=AVCC,2=AREFA,3=AREFB)");
  UsartWriteLine("s <freq>                   : Generate Sine wave on DACB/CH0 at freq");
  UsartWriteLine("S <freq>                   : Generate Square wave on DACB/CH0 at freq");
  UsartWriteLine("r <freq>                   : Generate Ramp wave on DACB/CH0 at freq");
  UsartWriteLine("t <freq>                   : Generate Triangle wave on DACB/CH0 at freq");
  UsartWriteLine("w <freq> <num> [data]      : Generate arbitrary waveform on DACB/CH0 at freq");
  UsartWriteLine("                             num samples. data is list of pos 12bit integers");
  UsartWriteLine("n <count>                  : output n wave cycles(max 255), if n=0 inf cycles");
  UsartWriteLine("O                          : output off");
};


void UsartWriteChar(unsigned char data)
{
    USARTF0.DATA = data; // transmit ascii 3 over and over
	if(!(USARTF0.STATUS&USART_DREIF_bm))
		while(!(USARTF0.STATUS & USART_TXCIF_bm)); // wait for TX complete
  	USARTF0.STATUS |= USART_TXCIF_bm;  // clear TX interrupt flag
};

unsigned char UsartReadChar(void)
{
	while(!(USARTF0.STATUS&USART_RXCIF_bm));  // wait for RX complete

  	return USARTF0.DATA;
};

// write out a simple '\0' terminated string
void UsartWriteString(char *string)
{

    while(*string != 0)
	  UsartWriteChar(*string++);
};

// write out a simple '\0' terminated string and print "\n\r" at end
void UsartWriteLine(char *string)
{
   UsartWriteString(string);
   UsartWriteString("\n\r");

};

void Error(char *string)
{
   UsartWriteString("Err: ");
   UsartWriteLine(string);
};

#define MAX_COMMAND_PARAMS 55
void ParseCommand(char *string)
{
  char Command;
  char Command2 = 0;
  unsigned int Params[MAX_COMMAND_PARAMS];
  unsigned int NumParams=0;

/*  UsartWriteString("Command Received: ");
  UsartWriteString(string);
  UsartWriteString("\n\r");
*/
  // assume commands are single character followed by numerical parameters sep by spaces
  // e.g. "s 1 5", "b 7", "b 100 120 001 212 123"
  Command = string[0];
  if(IsAlpha(string[1])) // multi-char command (e.g. pa, oa, ia, etc)
	  Command2 = string[1];

  if(Command != 0)
  {
    NumParams=GetParams(string,Params); // read any optional parameters after command

/*
    UsartWriteString("CommandID: ");
    UsartWriteChar(Command);
    if(Command2 != 0)
      UsartWriteChar(Command2);
    UsartWriteString(" #Params: ");
    UsartWriteChar(48+NumParams);
    UsartWriteString("\n\r");
*/
  }
  else
  {
  UsartWriteString("No Command\n\r");
  };

  switch(Command)
  {
   	case 'b': // blink(freq,num)
	  DoBlink(Params[0],Params[1]);
	  break;

    case 'd': // DAC set
	  if(NumParams==2)
	    DoDAC(Params[0],Params[1],1); // default use AVCC as reference
	  else if(NumParams==3)
	    DoDAC(Params[0],Params[1],Params[2]);
	  break;

	case 's': // Sine wave
	  DoSineWave(Params[0]);
	  break;

	case 'S': // Square wave
	  DoSquareWave(Params[0]);
	  break;

	case 'r': // Ramp wave
	  DoRampWave(Params[0]);
	  break;

	case 't': // Sine wave
	  DoTriangleWave(Params[0]);
	  break;

    case 'w': // Arbitrary waveform
	  DoArbWave(Params[0], Params[1], &Params[2]);
	  break;

    case 'n': // n wave cycles
	  DoNCycles((char)(Params[0]&0xFF));
	  break;

    case 'O':  // OFF (catch all possible typos too)
	case 'o':
	case '0':
	  DoOutputOff();
	  break;

    case 'h': // Usage
	  PrintUsage();
	  break;

    

  };	  


return;
};

void DoBlink(unsigned int freq, unsigned int count)
{
int i;
int period = 500/freq;

  PORTF.DIR |= (1<<0);

  for(i=0;i<count;i++)
  {
    PORTF.OUT |= (1<<0);
    _delay_ms(period);
    PORTF.OUT &= ~(1<<0);
    _delay_ms(period);
  };

};

void DoDAC(unsigned int ch, unsigned int value, unsigned int ref)
{
  if(ref<4)
    DACB.CTRLC = (ref << 3);           // select reference 
  else return; // early return for bad parameter

  DACB.CTRLB = DAC_CHSEL_DUAL_gc;      // select dual output DAC
  if(ch==0)
  {
	  DACB.CTRLA = (1<<2) | (1<<0);    // enable DACB, CH0
	  DACB.CH0DATA = value;
  }
  if(ch==1)
  {
      DACB.CTRLA |= (1<<3) | (1<<0);   // enable DACB, CH1
      DACB.CH1DATA = value;
  };

};

void DoSineWave(unsigned int freq)
{
  gWaveNumSamp=50;

  LoadSineWave(gWaveNumSamp);
  GenerateArbWave12(data12,gWaveNumSamp*2,10000);
  SetWaveFreq(freq);
};

void DoTriangleWave(unsigned int freq)
{
  gWaveNumSamp=50;

  LoadTriangleWave(gWaveNumSamp);
  GenerateArbWave12(data12,gWaveNumSamp*2,10000);
  SetWaveFreq(freq);
};

void DoSquareWave(unsigned int freq)
{
  gWaveNumSamp=50;

  LoadSquareWave(gWaveNumSamp);
  GenerateArbWave12(data12,gWaveNumSamp*2,10000);
  SetWaveFreq(freq);
};

void DoRampWave(unsigned int freq)
{
  gWaveNumSamp=50;

  LoadRampWave(gWaveNumSamp);
  GenerateArbWave12(data12,gWaveNumSamp*2,10000);
  SetWaveFreq(freq);
};

void DoArbWave(unsigned int freq, unsigned int numSamples, int *data)
{
int i;

  for(i=0;i<numSamples;i++)
  {
    data12[i]=data[i];
  };
  gWaveNumSamp=numSamples;

  GenerateArbWave12(data12,gWaveNumSamp*2,10000);

  SetWaveFreq(freq);

};

void DoNCycles(char num)
{
  SetWaveCount(num);
};

void DoOutputOff()
{
  DMA.CTRL = 0x00; // stop dma
  DoDAC(0,0,1);    // drive DAC to 0
};







int GetParams(char *string, unsigned int *Params)
{
char buffer[10]; // max parameter length is 10 characters
int NumParams=0;
int index_in=0;
int index_buf=0;
int NumFound=0;

NumParams=0; // clear every time called

while(string[index_in] != 0)
{
	if((string[index_in] >= 48)&&(string[index_in] <= 57))
	{
	   buffer[index_buf++]=string[index_in++];
	   NumFound=1;
	}
    else if(NumFound && string[index_in] == 32)   // space
	{  
	   buffer[index_buf]=0; // terminate with 0
	   Params[NumParams]=StringToInt(buffer);
	   NumParams++; // increment num params parsed
       for(index_buf=0;index_buf<10;index_buf++) buffer[index_buf]=0;   // null buffer
	   index_buf=0; // reset buffer index to beginning
	   index_in++;
	}
	else // illegal character, ignore
	{
	   index_in++;
    };

};

// if bytes left, parse them as last parameter (non-space terminated)
if(index_buf > 0)
{
  	   Params[NumParams]=StringToInt(buffer);
	   NumParams++; // increment num params parsed
};


return NumParams;
};


int IsAlpha(char val)
{
  if(((val > 64)&&(val < 91)) || ((val > 60)&&(val < 123))) return 1;
  else return 0;
};


unsigned int StringToInt(char *string)
{
int index=0;
int value=0;

  // assume string is of the form "NNNNN", unsigned integer, no leading blank space
  while((string[index] >= 48)&&(string[index] <= 57))
  {
     value *= 10;
	 value += string[index]-48;
	 index++;
  };

  return value; // test code, just return 0 every time -RPM
};


void IntToString(int value, char *string)
{
int index=0;

if(value < 0)
{
  string[index++]='-';
  value = value * -1; // remove sign
};

if(value > 9999)
  string[index++]= (value / 10000) + 48;
else
  string[index++]= 48;
value %= 10000;

if(value > 999)
  string[index++]= (value / 1000) + 48;
else
  string[index++]= 48;
value %= 1000;

if(value > 99)
  string[index++]= (value / 100) + 48;
else
  string[index++]= 48;
value %= 100;

if(value > 9)
  string[index++]= (value / 10) + 48;
else
  string[index++]= 48;

value %= 10;

string[index++]=value + 48;

string[index++]=0; // null terminate

};




void LoadSineWave(int len)
{
int i;

  for(i=0;i<len;i++)
  {
    data12[i]=((sin((2.0/(float)len)*(float)i*M_PI)*0.5 + 0.5)*4095);
  };

};

void LoadTriangleWave(int len)
{
int i;

  for(i=0;i<len/2;i++)
  {
    data12[i]=(int) (4095.0 * i / (len * 0.5));
  };


  for(i=len/2;i<len;i++)
  {
    data12[i]=(int) (4095.0 * ((len * 0.5) -  i / (len * 0.5)));
  };

};

void LoadRampWave(int len)
{
int i;

  for(i=0;i<len;i++)
  {
    data12[i]=(int) ((4095.0 * i) / len);
  };

};

void LoadSquareWave(int len)
{
int i;

  for(i=0;i<len/2;i++)
  {
    data12[i]=4095;
  };


  for(i=len/2;i<len;i++)
  {
    data12[i]=0;
  };

};

void SetWaveFreq(unsigned long int freq)
{
  if(freq > 10000)
  {
    TCD0.CTRLA = 0x1;
    TCD0.PER = (int) ((F_CPU/freq/gWaveNumSamp));
  }
  else
  {
    TCD0.CTRLA = 0x2;
    TCD0.PER = (int) ((F_CPU/freq/gWaveNumSamp)/2.0);

  };
};

void SetWaveCount(char count)
{
  gWaveCount = count; // if count = 0 then repeat count is infinite
};

void GenerateArbWave12(int *data, int len, unsigned long int freq)
{
EVSYS.CH1MUX = 0xD0;    // CH1 = TCD0 overflow
TCD0.CTRLA = 0x02;      // Prescaler: clk/2
TCD0.PER   = F_CPU/(int)((len/2.0)/freq/2.0);
TCD0.CNT   = 0;     // reset count
DACB.CTRLA = 0x05;      // Enable DACB and CH0
DACB.CTRLB = 0x01;  // DAC.CH0 auto triggered by an event
DACB.CTRLC = 0x08;  // Use AVCC (3.3V), right adjusted
DACB.EVCTRL = 0x01; // Event CH1 triggers the DAC Conversion
DACB.TIMCTRL = 0x50;// Minimum 32 CLK between conversions

// reset DMA first
DMA.CTRL = 0x00;    // first disable DMA
DMA.CTRL |= 0x40;   // do soft reset

DMA.CTRL = 0x80;    // Enable, single buffer, round robin
DMA.CH0.ADDRCTRL = 0x59;// Reload, Increment source
DMA.CH0.TRIGSRC= 0x25;  // DACB CH0 is trigger source
DMA.CH0.TRFCNT = len;   // Buffer is len bytes
DMA.CH0.REPCNT = gWaveCount;
DMA.CH0.SRCADDR0  =(((uint32_t)data)>>0*8) & 0xFF;
DMA.CH0.SRCADDR1  =(((uint32_t)data)>>1*8) & 0xFF;
DMA.CH0.SRCADDR2  =(((uint32_t)data)>>2*8) & 0xFF;
DMA.CH0.DESTADDR0 =(((uint32_t)(&DACB.CH0DATA))>>0*8)&0xFF;
DMA.CH0.DESTADDR1 =(((uint32_t)(&DACB.CH0DATA))>>1*8)&0xFF;
DMA.CH0.DESTADDR2 =(((uint32_t)(&DACB.CH0DATA))>>2*8)&0xFF;
DMA.CH0.CTRLA = 0xA5;   // Enable, repeat, 2 byte, single 

};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};

