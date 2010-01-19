#ifndef LED_PWM_H_
#define LED_PWM_H_

#include <stdint.h>

struct pwm_state
{
    uint8_t value; ///< actual pwm value
    uint8_t counter; ///< running counter
};

struct led
{
    struct pwm_state blue;
    struct pwm_state green;
    struct pwm_state red;
};

/// Do 6 pwm channels and return 6 pwm bits to be sent to
/// 6 output ports.
/// This is all assembly, since using the carry bit and
/// rotating is soooo much faster.
inline uint8_t do_6pwm(  volatile led *leds)
{
    // This function is mainly inline because it is being used in an 
    // ISR. If the function is inline, the optimizer will know which registers
    // need to be preserved and doesn't have to preserve all registers on stack...

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

/// Do 3 pwm channels and return 6 pwm bits to be sent to
/// 3 output ports.
inline uint8_t do_3pwm(  volatile led *leds)
{
    // This function is mainly inline because it is being used in an 
    // ISR. If the function is inline, the optimizer will know which registers
    // need to be preserved and doesn't have to preserve all registers on stack...

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
        : "=a" (result)
        : "e" (leds)
        : "r24");
    return result;
}

#endif //LED_PWM_H_
