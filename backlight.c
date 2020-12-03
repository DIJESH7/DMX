// Backlight Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   M0PWM3 (PB5) drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   M0PWM5 (PE5) drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   M0PWM4 (PE4) drives an NPN transistor that powers the blue LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "backlight.h"

// PortF masks
#define RED_BL_LED_MASK 2
#define BLUE_BL_LED_MASK 4
#define GREEN_BL_LED_MASK 8

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize backlight
void initBacklight()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three backlight LEDs
    GPIO_PORTF_DIR_R |= RED_BL_LED_MASK | GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;                       // make bit5 an output
    GPIO_PORTF_DR2R_R |= RED_BL_LED_MASK | GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;                      // set drive strength to 2mA
    GPIO_PORTF_DEN_R |= RED_BL_LED_MASK | GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;                       // enable digital
    GPIO_PORTF_AFSEL_R |= RED_BL_LED_MASK | GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;                     // select auxilary function
    GPIO_PORTF_PCTL_R &= GPIO_PCTL_PF1_M | GPIO_PCTL_PF3_M | GPIO_PCTL_PF2_M;                      // enable PWM
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;



    // Configure PWM module 0 to drive RGB backlight
    // RED   on M0PWM3 (PF1), M0PWM1b
    // BLUE  on M0PWM4 (PF2), M0PWM2a
    // GREEN on M0PWM5 (PF3), M0PWM2b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM1_3_CTL_R = 0;                                // turn-off PWM0 generator 2
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
                                                    // output 4 on PWM0, gen 2a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM1_2_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;
   PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
                                                     // invert outputs so duty cycle increases with increasing compare values
    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}

void setBacklightRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = 0;
    PWM1_3_CMPA_R = 2;
    PWM1_3_CMPB_R = 0;
}


