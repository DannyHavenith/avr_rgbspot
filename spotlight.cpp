//#include <avr/io.h>
//#include <stdlib.h>
//#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include "round_robin_buffer.h"
#include "led_pwm.h"
#include "led_transitions.h"


// The address of this device.
// will be filled with an eeprom-based address.
uint8_t     device_address ;
static const int number_of_leds = 2;

// eeprom variables
/// the stored address of this device
EEMEM uint8_t stored_device_address;

/// initial values for each led.
EEMEM uint8_t initial_led_values[number_of_leds][3] = {{ 5, 5, 5}, { 0, 0, 0}}; // r,g, b values for each led.

/// this buffer is used to transfer incoming bytes from the serial interrupt to the main 
/// program.
volatile round_robin_buffer<8> data_buffer;
/// led state
volatile led leds[number_of_leds];
/// do not perform any transitions while this is set
volatile bool hold_transitions;

/// counter that is used to make transition steps every pwm cycle
static uint8_t pwm_cycle_counter;

/// transition state for each led
led_transition transitions[number_of_leds];


/// set up a timer that fires at approximately 50 * 256 times per second.
static void timer_init()
{

    // count to 1652     
	OCR1A = 1652; 

    // Set up timer:
    // * prescaler = 1 (CS1x = 001)
    // * CTC mode with OCR1A as upper level (WGM1x = 0100)
	TCCR1B |= _BV(CS10) | _BV(WGM12); 

    // enable timer interrupt
    TIMSK |= _BV(OCIE1A);
 }

static void usart_init()
{
    const unsigned long baudrate = 2400;
    const unsigned long ubrr = ((F_CPU + 8 * baudrate)/(16UL * baudrate)) - 1;

    UBRRL = ubrr & 0xff;
    UBRRH = ubrr >> 8;

    // leave the rest default: 8n1, 16 samples per bit
  
    // enable serial input interrupt and serial input. 
    // need the main program to do sei()
    UCSRB |= _BV( RXCIE) | _BV(RXEN);
}


static void ioinit()
{ 
    // hardcoded ioinit, since it's hard to capture the 
    // pin assignments in DEFINES.

    // make sure all lights are off.
    // (and all input pins are pull-up).
    PORTD = 0xff;
    PORTB = 0x10;

    // PB0-PB5 are PWM 0-5 outputs (rgb0 and rgb1)
    // PD4-PD6 are PWM 6-8 (rgb2)
    DDRD = 0x70; // bits 4,5,6 to output
    DDRB = 0x3f; // bits 0-5 to output
    DDRA = 0;
    PORTA = 0xff;

}

/// Set the leds to a value as programmed in eeprom
static void data_init()
{
    // first, obtain the device address
    device_address = eeprom_read_byte( &stored_device_address);

    // then, set the initial led light values.
    for ( uint8_t led = 0; led < number_of_leds; ++led)
    {
        register uint8_t red   = eeprom_read_byte( &initial_led_values[led][0]);
        register uint8_t green = eeprom_read_byte( &initial_led_values[led][1]);
        register uint8_t blue  = eeprom_read_byte( &initial_led_values[led][2]);

        leds[led].red.value     = red;
        leds[led].green.value   = green;
        leds[led].blue.value    = blue;
    }
}

void __attribute__((noinline)) do_fade( uint8_t led, uint8_t time, uint8_t red, uint8_t green, uint8_t blue)
{
	if (time)
	{
		transitions[led].setup( leds[led], time, red, green, blue);
	}
    else
    {
        leds[led].red.value = red;
        leds[led].green.value = green;
        leds[led].blue.value = blue;
    }
}

static void fade( uint8_t led_index, uint8_t time)
{


//    uint8_t time = data_buffer.read_w();
    uint8_t new_red = data_buffer.read_w();
    uint8_t new_green = data_buffer.read_w();
    uint8_t new_blue = data_buffer.read_w();

	if (!led_index || led_index == 3)
	{
		do_fade( 0, time, new_red, new_green, new_blue);
	}
	if (led_index & 0x01)//(led_index == 1 || led_index == 3)
	{
		do_fade( 1, time, new_red, new_green, new_blue);
	}
}

static void wait_for_fader( uint8_t fader)
{
	while (transitions[fader].steps) /* wait */;
}

void  my_eeprom_write_byte( uint8_t *ptr, uint8_t val)
{
    if (eeprom_read_byte( ptr) != val)
    {
        eeprom_write_byte( ptr, val);
    }
}

/// set the address of this device, under the condition that the 'address button' is being pushed 
/// the address button is active-low.
static void set_address()
{
    uint8_t new_address = data_buffer.read_w(); 
    if (!(PORTD & _BV(3)))
    {
        device_address = new_address;
        my_eeprom_write_byte( &stored_device_address, new_address);
    } 
}


/// read R, G, and B values and store them in EEPROM memory as startup values
/// for the given led.
static void set_initial_values( uint8_t led_index)
{
    uint8_t *led_values = initial_led_values[led_index];

    // this takes three writes, while it could be done in one write, but it just takes too much code space
    // to use eeprom_write_block.
    my_eeprom_write_byte( led_values++, data_buffer.read_w());
    my_eeprom_write_byte( led_values++, data_buffer.read_w());
    my_eeprom_write_byte( led_values++, data_buffer.read_w());
}



int
main(void)
{
    ioinit();
    data_init();
    timer_init();
    usart_init();
    sei();


  
	for (;;)
	{
        
        uint8_t  command = data_buffer.read_w();
        switch (command & 0xf0)
        {
 
            case 0xA0:  // fade with a given timeout to new values
                fade( command & 0x03 , data_buffer.read_w());
                break;
                
            case 0xB0:  // program startup values for a spot
                set_initial_values( command & 0x01);
                break;
                
            case 0xC0:  // hold all transitions.
                hold_transitions  = true;
                break;

            case 0xD0:  // resume transitions
                pwm_cycle_counter = 255; // make sure there is a transition shortly after enabling them.
                hold_transitions  = false;
                break;

           case 0xE0:   // set new color
                fade( command & 0x03, 0);
                break;

            case 0xF0:  // program a new device address
                set_address();
                break;
            case 0x90: // wait for a fader to complete
            	wait_for_fader( command & 0x01);
            	break;

            default:    // ignore anything else
            break;
                
        }       
	}
	return 0;
}

/// Receive a byte from the serial port and follow a packet protocol
/// This function implements a state machine that expects the following order of bytes:
/// 1) noise (random bytes)
/// 2) preamble (zeroes, 1-n)
/// 3) end-of-preamble (the value 0x55)
/// 4) an address byte (0-255)
/// 5) a byte with a size encoded in the lower 4 bits. upper 4 bits reserved
/// 6) "size" times a data byte
/// 7) two checksum bytes
/// 1) noise again, etc...
/// If the checksum bytes work out OK, the data bytes will be committed to 
/// a round-robin buffer.
ISR( USART_RX_vect)
{
    static uint8_t     count = 0;
    static uint16_t    crc = 0xFFFF;
    //


    static enum {
        Noise,
        Preamble,
        Address,
        Size,
        Data,
        Checksum1,
        Checksum2
        } state = Noise;

    register uint8_t in = UDR;

    switch (state)
    {
        case Noise:
        	// if we receive a zero, it could be the start of a preamble
            if (!in)
            {
                state = Preamble;
            }
            break;
        case Preamble:
        {
        	// remain in preamble state as long as we receive zeros, 0x55 means end-of-preamble,
        	// anything else is noise.
            const uint8_t end_preamble = 0x55;
            if (in)
            {
                if (in == end_preamble)
                {
                    state = Address;
                }
                else
                {
                    state = Noise;
                }
            }
        }
            break;
        case Address:
        	// we expect an address byte here
            // address zero means 'all', both as a packet
            // address and as a device address. 
            // in any other case, the packet addres should match
            // our device address
            if (in == device_address || !device_address || !in)
            {
                state = Size;
            }
            else 
            {
            	// goto noise state and wait for the next preamble to appear
                state = Noise;
            }
            break;
        case Size:
        	// we expect a size byte. lower nibble is payload size minus one.
        	// upper nibble is reserved for future use and should be ignored/set to zero.
            count = (in & 0x0F) + 1;
            state = Data;
            data_buffer.reset_tentative();
            crc = 0x0000; // reset crc
            break;
        case Data:
        	// we're expecting data bytes here.
            crc = _crc_xmodem_update( crc, in);
            // if we cannot write to the data_buffer, discard
            // the whole packet
            if (!data_buffer.write_tentative( in))
            {
            	// data didn't fit in the buffer anymore, ignore whole packet.
                //data_buffer.reset_tentative(); // < this is done in the 'size' state
                state = Noise;
            }
            else
            {
                // if this is the last byte, 
                // move to the checksum state
                if (!--count)
                {


                    state = Checksum1;
                }
            }
           break;
        case Checksum1:
            crc = _crc_xmodem_update( crc, in);
            state = Checksum2;
            break;
        case Checksum2:
            if (!_crc_xmodem_update( crc, in))
            {
                // checksum verified, commit packet data to data_buffer
                data_buffer.commit();
            }
            //data_buffer.reset_tentative() is done in the 'size' state

            // we're finished with our packet
            // fall back to noise state.
            state = Noise; 
            break;
    }

}

static void transition_step()
{
    volatile led *l = leds;
    for (led_transition *t = transitions; t != transitions + number_of_leds; ++t, ++l)
    {
        if (t->steps)
        {
            t->step( *l);
        }
    }
}

ISR( TIMER1_COMPA_vect)
{
    // calculate new pwm output states and move to ports.
    // this would be hard to parametrize through DEFINEs,
    // so, I'm just hardcoding the ports and the bits.

    // using volatile pointer to work around compiler optimizer bug
    volatile led * volatile two_leds = &leds[0];
    PORTB = (do_6pwm( two_leds) & 0x3f) ^ 0x3f; // PB0-PB5
    //PORTB = 0x20;

    //two_leds += 2; // next two leds
    //PORTD = ((do_3pwm( two_leds) << 4) | 0x8f) ^ 0x70; // PD4-PD6 (PD0 is serial in, PD3 is address button).
    //PORTD = 0xff;

    if (!hold_transitions && !++pwm_cycle_counter)
    {
        transition_step();
    }
}

