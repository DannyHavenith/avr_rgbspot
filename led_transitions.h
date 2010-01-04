#ifndef LED_TRANSITIONS_H_
#define LED_TRANSITIONS_H_

#include <stdint.h>
#include "led_pwm.h"

struct bresenham_state
{
    uint8_t   accumulator;
    uint8_t   step;
    uint8_t   direction;
};

struct led_transition
{
    bresenham_state red;
    bresenham_state green;
    bresenham_state blue;

    
    uint8_t treshold;
    uint8_t steps;

    bool step( volatile led &my_led)
    {
        if (steps)
        {
            step( red, my_led.red);
            step( green, my_led.green);
            step( blue, my_led.blue);
            --steps;
        }
        return (steps != 0);
    }

    void setup( volatile led &current, 
                uint8_t time, uint8_t new_red, uint8_t new_green, uint8_t new_blue)
    {
        treshold = time;
        steps = time;
        setup_color( red, current.red, new_red);
        setup_color( green, current.green, new_green);
        setup_color( blue, current.blue, new_blue);
    }

  private:
    void setup_color(
        bresenham_state     &state,
        volatile pwm_state  &pwm,
        uint8_t             target_value
        )
    {
        if (target_value >= pwm.value)
        {
            state.step = target_value - pwm.value;
            // for some reason, if I move this line to after the
            // if-statement, the code becomes 300 bytes bigger.
            state.accumulator = treshold / 2;
            state.direction = 1;
        }
        else
        {
            state.step = pwm.value - target_value;
            state.accumulator = treshold / 2; 
            state.direction = 0xff; // -1, actually.
        }
    } 

    void step( bresenham_state &color, volatile pwm_state &led_color)
    {
        uint16_t accumulator = color.accumulator;
        accumulator += color.step;
        if ( accumulator >= treshold)
        {
            led_color.value += (color.direction * (accumulator / treshold));
            
        }
        color.accumulator = accumulator % treshold;
    }

};

#endif //LED_TRANSITIONS_H_
