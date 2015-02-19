/*****************************************************************************
 *
 * seri2c -- serial to i2c adapter
 * 
 * 2009, Tod E. Kurt, ThingM Corp.  http://thingm.com/
 *
 * For MODE_FRAME:
 *
 * For MODE_SERI2C:
 * Command format is:
 * pos description
 *  0   <startbyte>           == 0x00
 *  1   <i2c_addr|command>    == 0x00-0x7f, 0xff == scanbus, 0xfe = framemode
 *  2   <num_bytes_to_send>
 *  3   <num_bytes_to_receive>
 *  4   <send_byte0>
 *  5..n [<send_byte1>...]
 *
 * For BlinkMs:
 * Thus minimum command length is 5 bytes long, for reading back a color, e.g.:
 *   {0x01,0x09,0x01,0x01, 'g'}
 * Most commands will be 8 bytes long, say to fade to an RGB color, e.g.:
 *   {0x01,0x09,0x04,0x00, 'f',0xff,0xcc,0x33}
 * The longest command is to write a script line, at 12 bytes long, e.g.:
 *   {0x01,0x09,0x08,0x00, 'W',0x00,0x01,50,'f',0xff,0xcc,0x33}
 *
 * For MODE_BLINKM_TEST:
 *
 *
 * Written for the ATtiny2313 and 'seri2c' board
 *
 * Based on AppNote AVR310 - Using the USI module as a TWI Master
 * Uses Peter Fleury's i2cmaster software I2C library
 *
 ****************************************************************************/

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <ctype.h>  
#include "i2cmaster.h"

#define BAUD 115200 
//#define BAUD 19200 
#define UBBRVAL (((F_CPU/16)/BAUD)-1)

// If MODE_FRAME defined, it's in 'frame mode', and reads a 'framebuffer',
// starting with the 4-byte sequence 0xFE,0x01,0x02,0x03, 
// then containing a set of PIXELCOUNT RGB triplets
// Each device searches within the framebuffer for its RGB triplet, based
// on its MYID
//#define MODE_FRAME 1
 if MODE_SERI2C defined, then functions as in BlinkMCommunicator
//#define MODE_SERI2C 1
// if MODE_BLINKM_TEST define, then is an interactive blinkm tester
//#define MODE_BLINKM_TEST 1


#define PIXELCOUNT 8
#define MYID 2

// outgoing i2c buffer size
#define MESSAGEBUF_SIZE       16

// LED and Button Ports 
#define PORT_LED    PORTD
#define DDR_LED     DDRD
#define P_LED_1     PD6         // used for init & data signal (for Debug)
#define P_LED_2     PD6         // used for debug
#define P_LED_3     PD6         // used for errors

// serial stuff
#define RX_BUFFER_SIZE 16
uint8_t rx_buffer[RX_BUFFER_SIZE];
int8_t rx_buffer_head = 0;
int8_t rx_buffer_tail = 0;

uint8_t blinkm_addr = 0x00;

uint8_t rc; // return code for i2c checking stuff

void ledBlink( uint8_t count, uint8_t decisec, uint8_t led);
void ledOn(uint8_t led); 
void ledOff(uint8_t led); 
uint8_t parseHex(char c);
char toHex(uint8_t n);


// -----------------------------------------------------------------------

//
static void serialFlush(void)
{
	rx_buffer_head = rx_buffer_tail;
}
//
static int serialAvailable(void)
{
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) % RX_BUFFER_SIZE;
}
//
static int serialRead(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_buffer_head == rx_buffer_tail) {
    return -1;
  } else {
    uint8_t c = rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}

// send a character out the serial port
static void serialPutChar(uint8_t c)
{
  while( ! (UCSRA & (1<<UDRE) ) ) ;
  UDR = c;
}
//
static void serialPutNewline(void) 
{
  serialPutChar('\r');
  serialPutChar('\n');
}
//
static void serialPutHex(uint8_t b)
{
  serialPutChar(toHex(b>>4)); serialPutChar(toHex(b&0x0f));
}
// send a string out the serial port
static void serialPutStr(uint8_t* str, uint8_t nl)
{
  while( *str )
    serialPutChar( *str++ );
  if( nl )
    serialPutNewline();
}
// send a string from ROM out the serial port
#define serialPutStrP(s,n) serialPutStrROM(PSTR(s),n)
static void serialPutStrROM(const char *str, uint8_t nl)
{
  uint8_t c;
  while((c = pgm_read_byte(str++)))
    serialPutChar(c);
  if( nl ) 
    serialPutNewline();
}

// receive chars from serial port
SIGNAL(SIG_USART0_RECV)
{
  unsigned char c = UDR; // read serial char
  uint8_t i = (rx_buffer_head + 1) % RX_BUFFER_SIZE;
  if (i != rx_buffer_tail) {      // stolen from wriing_serial.c
    rx_buffer[rx_buffer_head] = c;
    rx_buffer_head = i;
  }
}


// --------------------------------------------------------------------------
// i2c & blinkm commands
//

static uint8_t i2c_scanbus(uint8_t from, uint8_t to)
{
  i2c_init();
  serialPutStrP("i2cscan:",1);
  for( uint8_t a=from; a<to; a++ ) {
    if( i2c_start( (a<<1)|I2C_WRITE ) == 0 ) {
      serialPutStrP("found:0x",0);  serialPutHex(a); serialPutNewline();
    }
    i2c_stop();
  }
  return 0;
}

//
static uint8_t BlinkM_sendCmd(uint8_t addr, uint8_t* cmdbytes, uint8_t cmdlen)
{
  if( i2c_start((addr<<1)|I2C_WRITE) ) return 1;// set device addr & write mode
  for( uint8_t i=0; i<cmdlen; i++) 
    i2c_write(cmdbytes[i]);
  i2c_stop();                                // set stop conditon = release bus
  return 0;
}
//
static int BlinkM_receiveBytes(uint8_t addr, uint8_t* resp, uint8_t len)
{
  uint8_t ret;                            // set device address and write mode
  if( i2c_start((addr<<1)|I2C_WRITE) ) return -1;
  i2c_write('x');                         // write address = 5
  i2c_rep_start((addr<<1)|I2C_READ);      // set device address and read mode
  for( uint8_t i=0; i<len; i++ ) {      // FIXME: this is wrong
    ret = i2c_read(i==(len-1));           // read one byte
  }
  i2c_stop();

  return -1;
}
//
static uint8_t BlinkM_stop(uint8_t addr ) {
  if( i2c_start((addr<<1)|I2C_WRITE)  )  // returns 0 on success, 1 on failure
    return 1;
  i2c_write('o');
  i2c_stop();
  return 0;
}
//
static uint8_t BlinkM_setFadeSpeed(uint8_t addr, uint8_t f)
{
  if( i2c_start((addr<<1)|I2C_WRITE) ) return 1;
  i2c_write('f');
  i2c_write(f);
  i2c_stop();
  return 0;
}
//
static uint8_t BlinkM_cmd3Val(uint8_t addr, uint8_t cmd,
                              uint8_t a, uint8_t b, uint8_t c)
{
  if( i2c_start((addr<<1)|I2C_WRITE) ) return 1;
  i2c_write(cmd);
  i2c_write(a);
  i2c_write(b);
  i2c_write(c);
  i2c_stop();
  return 0;
}
#define BlinkM_fadeToRGB(addr,r,g,b) BlinkM_cmd3Val(addr,'c',r,g,b)
#define BlinkM_setRGB(addr,r,g,b)    BlinkM_cmd3Val(addr,'n',r,g,b)
#define BlinkM_fadeToHSB(addr,h,s,b) BlinkM_cmd3Val(addr,'h',h,s,b)

//---------------------------------------------------------------------------

#ifdef MODE_FRAME
static void handle_commands(void)
{
  int cmdi = serialAvailable();
  if( cmdi < 1 ) return;
  
  int cmd = serialRead();
  if( cmd == 0xFE ) {                // start of frame marker
    while( serialAvailable() < 3 );  // FIXME: possible infinite loop here
    if( serialRead() == 0x01 && 
        serialRead() == 0x02 && 
        serialRead() == 0x03 ) {
      // we got the header
      ledOn(P_LED_1);
      for( uint8_t i=0; i< PIXELCOUNT; i++ ) {
        while( serialAvailable() < 3 );
        uint8_t r = serialRead();
        uint8_t g = serialRead();
        uint8_t b = serialRead();
        if( i == MYID )  {   // we've found our pixel
          BlinkM_setRGB( blinkm_addr, r,g,b );
        }
      }
      ledOff(P_LED_1);
    } // end of frame command
  } // end of potential frame command (but not)
}
#endif  // MODE_FRAME

#ifdef MODE_SERI2C                 // FIXME: NOT TESTED YET
static void handle_commands(void)
{
  uint8_t addr, num_send,num_recv;
  int cmdi = serialAvailable();
  if( cmdi < 1 ) return;
  
  int cmd = serialRead();
  if( cmd == 0x00 ) {
    while( serialAvailable() < 3 );         // FIXME: possible infinite loop
    addr = serialRead();
    num_send = serialRead();
    num_recv = serialRead();
    
    while( serialAvailable() < num_send );  // FIXME: possible infinite loop
    if( i2c_start((addr<<1)|I2C_WRITE) ) {
      ; // error
    }
    for( uint8_t i=0; i<num_send; i++) {
      i2c_write( serialRead() );            // send from serial to i2c
    }
    
    if( num_recv != 0 ) {
      i2c_start_wait((addr<<1)|I2C_READ);
      for( uint8_t i=0; i<num_recv; i++) {
        uint8_t c = i2c_read(i!=(num_recv-1)); // send from i2c to serial
        serialPutChar( c );
      }
    }
    
    i2c_stop();
  }
}
#endif  // MODE_SERI2C
   
#ifdef MODE_BLINKM_TEST
static void handle_commands(void)
{
  int cmdi = serialAvailable();
  if( cmdi < 1 ) return;

  int cmd = serialRead();
  serialPutChar( cmd ); serialPutChar(':');
  
  if( cmd == 'b' ) {
    serialPutStrP("blink",1);
    ledBlink(10,10,P_LED_1);     // Blink as init signal
  }
  else if( cmd == 'd' ) {
    serialPutStrP("dump:",0);
    serialPutHex(cmdi);
  }
  else if( cmd == 'o' ) { 
    serialPutStrP("stop",1);
    rc = BlinkM_stop( blinkm_addr );
    if( rc == 1 )  serialPutChar('!');
  }
  else if( cmd == '0' ) {
    serialPutStrP("off",1);
    uint8_t cmd[] = { 'n', 0x00, 0x00, 0x00 };
    if( BlinkM_sendCmd( blinkm_addr, cmd, 4 ) == 1 ) serialPutChar('!');
  }
  else if( cmd == '1' ) { 
    serialPutStrP("on",1);
    uint8_t cmd[] = { 'n', 0xff, 0xff, 0xff };
    if( BlinkM_sendCmd( blinkm_addr, cmd, 4 ) == 1 ) serialPutChar('!');
  }
  else if( cmd == 'c' ) {
    serialPutStrP("color",1);
    while( serialAvailable() < 6 );  // FIXME: possible infinite loop here
    serialPutStrP("all",1);
    uint8_t cmdbuf[6]; 
    for( uint8_t i=0; i<6; i++) 
      cmdbuf[i] = serialRead();
    uint8_t r = parseHex(cmdbuf[1])<<4 | parseHex(cmdbuf[2]);
    uint8_t g = parseHex(cmdbuf[3])<<4 | parseHex(cmdbuf[4]);
    uint8_t b = parseHex(cmdbuf[5])<<4 | parseHex(cmdbuf[6]);
    BlinkM_fadeToRGB( blinkm_addr, r,g,b );
  }
  else if( cmd == 's' ) { 
    i2c_scanbus(1,100);
  }
  else if( cmd == '?' ) {
    serialPutStrP("\nseri2c: b,d,o,0,1,c",1);
  }
  else { 
    return;
  }
  cmdi=0; // say we ate the buffer up, yum
}
#endif  // MODE_BLINKM_TEST


//---------------------------------------------------------------------------
// Behavior: 
//      Blink LED at start.
//      ...
//---------------------------------------------------------------------------
int main( void )
{
  // initialize serial
  UBRRH = UBBRVAL >>8;       // set baud rate, hi reg
  UBRRL = UBBRVAL & 0xff;    // set baud rate, lo reg
  UCSRB = (1<<RXCIE) | (1 << RXEN) | (1 << TXEN); // enable RX & TX, & interrupt
  UCSRC = (1<<UCSZ1)|(1<<UCSZ0);    // Set frame format: async, 8N1

  // Initialize LED blinker 
  DDR_LED = _BV(P_LED_1) | _BV(P_LED_2) | _BV(P_LED_3);      // enable output
    
  ledBlink(5,10,P_LED_1);     // Blink as init signal, we're alive!

  i2c_init();              // initialize I2C library

  sei();

#ifdef MODE_FRAME
  BlinkM_stop( blinkm_addr );
  BlinkM_setFadeSpeed(blinkm_addr, 30);

  uint8_t cmdbytes[] = { 'c', 0x00, 0x00, 0x00 }; // fade to black
  if( BlinkM_sendCmd( blinkm_addr, cmdbytes, 4 ) == 1 ) serialPutChar('!');
  // FIXME: why must this line be in there? 
#endif

  serialPutStrP("\r\nseri2c ready!",1); 

  while(1) {
    handle_commands(); // yup
  }
   
}

//
void ledOn(uint8_t led) 
{
  PORT_LED |= _BV(led);
}
//
void ledOff(uint8_t led) 
{
  PORT_LED &=~ _BV(led);
}
// ------------------------------------------------------------------------
//  ledBlink - function to blink LED for count passed in
//      Assumes that leds are all on the same port. 
// ------------------------------------------------------------------------
void ledBlink( uint8_t count, uint8_t decisec, uint8_t led)
{
  uint8_t i;
  while (count > 0) {
    PORT_LED |= _BV(led);
    for (i=0;i<decisec;i++)  _delay_ms(10);
    PORT_LED &= ~_BV(led);
    for (i=0;i<decisec;i++)  _delay_ms(10);
    count--;
  }
}


//---------------------------------------------------------------------
// util funcs

// take a ascii hex character, return the nibble value for it
uint8_t parseHex(char c) 
{
  c = toupper(c);
  if (c >= '0' && c <= '9') return (c - '0');
  if (c >= 'A' && c <= 'F') return (c - 'A')+10;
  return 0;
}
// given a number, return the ascii hex character for it
char toHex(uint8_t n)
{
  n += 0x30;             // add offset to ASCII '0'
  if (n > '9') n += 7;  // if alpha add offset to ASCII 'A'
  return n;
}
