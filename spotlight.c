//#include <avr/io.h>
//#include <stdlib.h>
//#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <stdint.h>
#include "round_robin_buffer.h"


/// set up a timer interrupt that fires 16000 times per second,
/// This means that at 8 Mhz, the timer fires every 500 ticks.
/// 16000 ticks per second means that an 8 bit pwm will do 
/// 62.5 cycles per second.
void timer_init()
{

    // count to 500     
	OCR1A = 499; 

    // Set up timer:
    // * prescaler = 1 (CS1x = 001)
    // * CTC mode with OCR1A as upper level (WGM1x = 0100)
	TCCR1B |= _BV(CS10) | _BV(WGM12); 

    // enable timer interrupt
    TIMSK |= _BV(OCIE1A);
 }

void usart_init()
{
    const unsigned long baudrate = 2400;
    const unsigned long ubrr = (F_CPU/(16UL * baudrate)) - 1;

    UBRRL = ubrr;
    UBRRH = ubrr >> 8;

    // leave the rest default: 8n1, 16 samples per bit
  
    // enable serial input interrupt and serial input. 
    // need the main program to do sei()
    UCSRB |= _BV( RXCIE) | _BV(RXEN);
}


void ioinit()
{ 
    // hardcoded ioinit, since it's hard to capture the 
    // pin assignments in DEFINES.

    // PB0-PB5 are PWM 0-5 outputs (rgb0 and rgb1)
    // PD1-PD6 are PWM 6-11 (rgb2 and rgb3)
    DDRD = 0x7e; // all but bit 0 and 7 to output
    DDRB = 0x3f; // bits 0-5 to output
    DDRA = 0;

    // make sure all lights are off.
    PORTD = 0;
    PORTB = 0;
}

struct pwm_state
{
    uint8_t value; ///< actual pwm value
    uint8_t counter; ///< running counter
};

struct led
{
    struct pwm_state red;
    struct pwm_state green;
    struct pwm_state blue;
};


volatile round_robin_buffer<10> data_buffer = {0,0,0,0, {0}};
volatile led leds[4];

/// read a triplet from the data buffer and set
/// the rgb-values for the led with index led_index 
/// accordingly.
void set_triplet( uint8_t led_index)
{
    volatile led *current = &leds[led_index];
    current->red.value = data_buffer.read_w();
    current->green.value = data_buffer.read_w();
    current->blue.value = data_buffer.read_w();
}


int
main(void)
{
    for (int i = 0; i < 4; ++i)
    {
	    leds[i].red.value = 0;
	    leds[i].green.value = 0;
	    leds[i].blue.value = 0;
    }

 /*   leds[1].red.value = 16;
    leds[1].green.value = 32;
    leds[1].blue.value = 255;
/*    data_buffer.write( 0x91);
    data_buffer.write( 0);
    data_buffer.write( 0);
    data_buffer.write( 0);
    data_buffer.write( 0x91);
    data_buffer.write( 0);
    data_buffer.write( 0);
    data_buffer.write( 0);/**/
    data_buffer.write( 0x91);
    data_buffer.write( 16);
    data_buffer.write( 64);
    data_buffer.write( 255);
    /**/
    ioinit();
    timer_init();
    usart_init();
    sei();

    //PORTB = 0x3f;
	for (;;)
	{
        
        uint8_t  command = data_buffer.read_w();
        switch (command & 0xf0)
        {
            case 0x90:
                set_triplet( command & 0x03);
                break;
            default:
            break;
                // ignore
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
    static uint8_t     device_address = 0;
    static uint16_t    crc = 0xFFFF;

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
            if (!in)
            {
                state = Preamble;
            }
            break;
        case Preamble:
        {
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
                state = Noise;
            }
            break;
        case Size:
            count = (in & 0x0F) + 1;
            state = Data;
            data_buffer.reset_tentative();
            crc = 0x0000; // reset crc
            break;
        case Data:
            crc = _crc_xmodem_update( crc, in);
            // if we cannot write to the data_buffer, discard
            // the whole packet
            if (!data_buffer.write_tentative( in))
            {
                data_buffer.reset_tentative();
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
            else
            {
                data_buffer.reset_tentative();
            }

            // we're finished with our packet
            // fall back to noise state.
            state = Noise; 

            break;
    }

    //data_buffer.write( UDR);
}

/// Do 6 pwm channels and return 6 pwm bits to be sent to
/// 6 output ports.
/// This is all assembly, since using the carry bit and
/// rotating is soooo much faster.
static uint8_t do_6pwm(  volatile led *leds)
{
    // unrolled loop of 6:
    // for each pwm channel,
    // add the pwm value to a running counter and rotate the carry bit
    // into the combined result.
    uint8_t result;
    asm volatile(
        "ld     __tmp_reg__, %a1+\n"
        "ld     r24, %a1          \n"
        "add    __tmp_reg__, r24  \n" // add the first byte to the second
        "st     %a1+, __tmp_reg__\n"
        "rol    %0               \n" // rotate the carry into the pwm bits
        "\n"
        "ld     __tmp_reg__, %a1+\n"
        "ld     r24, %a1          \n"
        "add    __tmp_reg__, r24  \n" // add the first byte to the second
        "st     %a1+, __tmp_reg__\n"
        "rol    %0               \n" // rotate the carry into the pwm bits
        "\n"
        "ld     __tmp_reg__, %a1+\n"
        "ld     r24, %a1          \n"
        "add    __tmp_reg__, r24  \n" // add the first byte to the second
        "st     %a1+, __tmp_reg__\n"
        "rol    %0               \n" // rotate the carry into the pwm bits
        "\n"
        "ld     __tmp_reg__, %a1+\n"
        "ld     r24, %a1          \n"
        "add    __tmp_reg__, r24  \n" // add the first byte to the second
        "st     %a1+, __tmp_reg__\n"
        "rol    %0               \n" // rotate the carry into the pwm bits
        "\n"
        "ld     __tmp_reg__, %a1+\n"
        "ld     r24, %a1          \n"
        "add    __tmp_reg__, r24  \n" // add the first byte to the second
        "st     %a1+, __tmp_reg__\n"
        "rol    %0               \n" // rotate the carry into the pwm bits
        "\n"
        "ld     __tmp_reg__, %a1+\n"
        "ld     r24, %a1          \n"
        "add    __tmp_reg__, r24  \n" // add the first byte to the second
        "st     %a1+, __tmp_reg__\n"
        "rol    %0               \n" // rotate the carry into the pwm bits
        "\n"
        : "=a" (result)
        : "e" (leds)
        : "r24");
    return result;
}


ISR( TIMER1_COMPA_vect)
{
    // calculate new pwm output states and move to ports.
    // this would be hard to parametrize through DEFINEs,
    // so, I'm just hardcoding this.
    PORTB = do_6pwm( &leds[0]) & 0x3f; // PB0-PB5
    PORTD = (do_6pwm( &leds[2]) << 1) | 0x81; // PD1-PD6 (PD0 is serial in).

}

