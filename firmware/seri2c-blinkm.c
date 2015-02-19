/*****************************************************************************
 *
 * File              seri2c
 * Date              10 Jan 2009
 * Updated by        tod

 * Command format is:
 * pos description
 *  0   <startbyte>
 *  1   <i2c_addr>
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

#define BAUD 9600 
#define UBBRVAL (((F_CPU/16)/BAUD)-1)

// outgoing i2c buffer size
#define MESSAGEBUF_SIZE       16

// LED and Button Ports 
#define PORT_LED    PORTD
#define DDR_LED     DDRD
#define P_LED_1     PD6         // used for init & data signal (for Debug)
#define P_LED_2     PD6         // used for debug
#define P_LED_3     PD6         // used for errors

// serial stuff
#define RX_BUFFER_SIZE 32
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;
uint8_t rx_buffer_tail = 0;

uint8_t blinkm_addr = 0x00;

void blinkLed( uint8_t count, uint8_t led);
uint8_t parseHex(char c);
char toHex(uint8_t n);


//
// -----------------------------------------------------------------------
//

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


// --------------------------------------------------------------------------
// i2c & blinkm commands
//

//
static void BlinkM_sendCmd(uint8_t addr, uint8_t* cmd, uint8_t cmdlen)
{
  i2c_start_wait((addr<<1)|I2C_WRITE);    // set device address and write mode
  for( uint8_t i=0; i<cmdlen; i++) 
    i2c_write(cmd[i]);
  i2c_stop();                             // set stop conditon = release bus
}
static int BlinkM_receiveBytes(uint8_t addr, uint8_t* resp, uint8_t len)
{
  uint8_t ret;
  i2c_start_wait((addr<<1)|I2C_WRITE);    // set device address and write mode
  i2c_write('x');                         // write address = 5
  i2c_rep_start((addr<<1)|I2C_READ);      // set device address and read mode
  for( uint8_t i=0; i<len; i++ ) {      // FIXME: this is wrong
    ret = i2c_read(i==(len-1));           // read one byte
  }
  i2c_stop();

  return -1;
}
//
static void BlinkM_stop(uint8_t addr ) {
  i2c_start_wait((addr<<1)|I2C_WRITE);
  i2c_write('o');
  i2c_stop();
}
//
static void BlinkM_setFadeSpeed(uint8_t addr, uint8_t f)
{
  i2c_start_wait((addr<<1)|I2C_WRITE);
  i2c_write('f');
  i2c_write(f);
  i2c_stop();
}
//
static void BlinkM_fadeToRGB(uint8_t addr, uint8_t r,uint8_t g,uint8_t b)
{
  i2c_start_wait((addr<<1)|I2C_WRITE);
  i2c_write('c');
  i2c_write(r);
  i2c_write(g);
  i2c_write(b);
  i2c_stop();
}

//
static void handle_commands(void)
{
  int cmdi = serialRead();
  if( cmdi == -1 ) return;

  char cmd = serialRead();
  if( cmd == 'b' ) {
    serialPutStrP("blink",1);
    blinkLed(2,P_LED_1);     // Blink as init signal
  }
  else if( cmd == 'd' ) {
    serialPutStrP("dump:",0);
    serialPutHex(cmdi);
  }
  else if( cmd == 'o' ) { 
    serialPutStrP("stop",1);
    BlinkM_stop( blinkm_addr );
  }
  else if( cmd == '0' ) {
    serialPutStrP("off",1);
    uint8_t cmd[] = { 'n', 0x00, 0x00, 0x00 };
    BlinkM_sendCmd( blinkm_addr, cmd, 4 );
  }
  else if( cmd == '1' ) { 
    serialPutStrP("on",1);
    uint8_t cmd[] = { 'n', 0xff, 0xff, 0xff };
    BlinkM_sendCmd( blinkm_addr, cmd, 4 );
  }
  else if( cmd == 'c' ) {
    serialPutStrP("color",1);
    while( serialAvailable() < 6 );  // FIXME: possible infinite loop here
    uint8_t cmdbuf[6]; 
    for( uint8_t i=0; i<6; i++) 
      cmdbuf[i] = serialRead();
    uint8_t r = parseHex(cmdbuf[1])<<4 | parseHex(cmdbuf[2]);
    uint8_t g = parseHex(cmdbuf[3])<<4 | parseHex(cmdbuf[4]);
    uint8_t b = parseHex(cmdbuf[5])<<4 | parseHex(cmdbuf[6]);
    BlinkM_fadeToRGB( blinkm_addr, r,g,b );
  }
  else { 
    return;
  }
  cmdi=0; // say we ate the buffer up, yum
}




//---------------------------------------------------------------------------
// Behavior: 
//      Blink LED at start.
//      Wait for button.
//      On button press, read Port Expander, store code 
//          back to Port Expander (4 ins, 4 outs).
//      Blink code on Error LED if there's an error.
//      If DEBUG is defined, blink Debug LED to indicate program progress and
//          use Signal LED to echo switch reading.
//---------------------------------------------------------------------------

int main( void )
{
  // initialize serial
  UBRRH = UBBRVAL >>8;
  UBRRL = UBBRVAL & 0xff; // baud rate 9600
  // Enable SERIAL receiver and transmitter, and RX interrupt
  UCSRB = (1<<RXCIE) | (1 << RXEN) | (1 << TXEN);
  // Set frame format: async, 8-bit, no parity, 1 stop
  UCSRC = (1<<UCSZ1)|(1<<UCSZ0);

  // Initialize LED blinker 
  DDR_LED = _BV(P_LED_1) | _BV(P_LED_2) | _BV(P_LED_3);      // enable output
    
  blinkLed(1,P_LED_1);     // Blink as init signal
    
  i2c_init();              // initialize I2C library

  sei();
    
  serialPutStrP("\r\nserialtoi2c ready",1);

  BlinkM_stop( blinkm_addr );
  BlinkM_setFadeSpeed(blinkm_addr, 30);
  
  while(1) {
    handle_commands();
    /*
    for(int i=0; i<4; i++) _delay_ms(100);
    send_color( 'n', 0xff,0xff,0xff);
    for(int i=0; i<2; i++) _delay_ms(100);
    send_color( 'c', 0,0,0);
    */
  }
   
}

// ------------------------------------------------------------------------
//  blinkLed - function to blink LED for count passed in
//      Assumes that leds are all on the same port. 1MHz Clock rate.
// ------------------------------------------------------------------------
void blinkLed( uint8_t count, uint8_t led)
{
  uint8_t i;
  while (count > 0) {
    PORT_LED |= _BV(led);
    for (i=0;i<5;i++)  _delay_ms(100);
    PORT_LED &= ~_BV(led);
    for (i=0;i<5;i++)  _delay_ms(100);
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
