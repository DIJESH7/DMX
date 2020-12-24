//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "uart_input.h"
#include "uart1.h"
#include "eeprom.h"
#include "wait.h"
#include "backlight.h"
#define DE (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))

#define DE_MASK 128

uint32_t ReadIndex = 0;
uint32_t WriteIndex = 0;
uint32_t BUFFER_LENGTH = 100;
char chartxBuffer[100];
uint32_t phase = 0;
uint32_t rx_phase = 0;
uint32_t max_add = 514;
uint32_t DATA[512];
uint32_t start = 0;
uint16_t Mode = 0x01;
uint16_t Address = 0x0;
bool state = false;
uint32_t data = 0;
uint32_t LED_TIMEOUT_OFF = 10;
uint32_t LED_TIMEOUT_ON = 0;
uint32_t LED_BLUE = 0;
uint32_t LED_GREEN = 0;
uint32_t LED_RED = 0;
uint32_t LED_B = 0;
uint32_t LED_G = 0;
uint32_t LED_R = 0;
bool poll_request = false;
uint32_t current_time = 32400; //9 o'clock in the morning

uint32_t month = 0;
uint32_t date = 0;
uint32_t hr = 0;
uint32_t min = 0;
uint32_t sec = 0;
uint32_t alarm_time = 0;
uint32_t alarm_table[512][3];
uint32_t i = 0;
uint32_t j = 0;
uint32_t t = 0;
uint32_t current = 0;
uint32_t k = 0;
uint32_t soonest = 0;
uint32_t add = 0;
uint32_t ACK[512];
uint32_t acked = 0;
//uint32_t current_hms = sec + 60 * (min + 60 * (hr));
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCTIMER_R |= (SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2);

    _delay_cycles(3);

    GPIO_PORTC_DIR_R |= DE_MASK;
    GPIO_PORTC_DEN_R |= DE_MASK;

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 7040;     // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    //TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on time

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER2_TAILR_R = 4000000;  // set load value to 40e6 for 1 Hz interrupt rate
    //TIMER2_IMR_R = TIMER_IMR_TATOIM;
    // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER2A - 16);     // turn-on interrupt 37 (TIMER1A)
    //TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on time

}
void initHIB()
{
    SYSCTL_RCGCHIB_R = SYSCTL_RCGCHIB_R0;
    _delay_cycles(3);
    HIB_CTL_R |= HIB_CTL_CLK32EN;
    while (!(HIB_CTL_R & HIB_CTL_WRC))
        ;
    NVIC_EN1_R |= 1 << (INT_HIBERNATE - 16 - 32);

    while (!(HIB_CTL_R & HIB_CTL_WRC))
        ;
    HIB_IM_R |= HIB_IM_RTCALT0;
    while (!(HIB_CTL_R & HIB_CTL_WRC))
        ;
    HIB_CTL_R |= 0X00000041;
}

bool checkCommand(USER_DATA data)
{
    bool valid = false;
    if (isCommand(&data, "set", 2))
    {
        int32_t add = getFieldInteger(&data, 1);
        int32_t Data = getFieldInteger(&data, 2);
        PWM1_3_CMPB_R = 250;
        LED_TIMEOUT_OFF = 10;
        LED_GREEN = 1;
        TIMER2_TAILR_R = 4000000;
        TIMER2_IMR_R = TIMER_IMR_TATOIM;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;

        DATA[add - 1] = Data;
        //displayUart0("table updated\n\r");
        valid = true;
        return valid;
    }
    if (isCommand(&data, "set", 7))
    {
        int32_t add = getFieldInteger(&data, 1);
        int32_t value = getFieldInteger(&data, 2);
        int32_t hour = getFieldInteger(&data, 3);
        int32_t minute = getFieldInteger(&data, 4);
        int32_t second = getFieldInteger(&data, 5);
        int32_t mon = getFieldInteger(&data, 6);
        int32_t day = getFieldInteger(&data, 7);
        alarm_time = second
                + 60 * (minute + 60 * (hour + 24 * (mon * 30 + day)));

        if (current == 0)
        {
            alarm_table[current][0] = alarm_time;
            alarm_table[current][1] = add;
            alarm_table[current][2] = value;
            HIB_CTL_R |= 0x00000040;
            while (!(HIB_CTL_R & HIB_CTL_WRC))
                ;
            HIB_RTCM0_R = alarm_table[0][0];
            while (!(HIB_CTL_R & HIB_CTL_WRC))
                ;
            HIB_IM_R |= HIB_IM_RTCALT0;
            while (!(HIB_CTL_R & HIB_CTL_WRC))
                ;
            HIB_CTL_R |= 0x00000041;
//            soonest=current;

        }
        if (current >= 1)
        {
            alarm_table[current][0] = alarm_time;
            alarm_table[current][1] = add;
            alarm_table[current][2] = value;

            for (i = 0; i < current; i++)
                for (j = current; j > i; j--)
                    if (alarm_table[j - 1][0] > alarm_table[j][0])
                    {
                        for (k = 0; k < 3; k++)
                        {
                            // Swap all columns
                            t = alarm_table[j - 1][k];
                            alarm_table[j - 1][k] = alarm_table[j][k];
                            alarm_table[j][k] = t;

                        }
                    }
            HIB_CTL_R |= 0x00000040;
            while (!(HIB_CTL_R & HIB_CTL_WRC))
                ;
            HIB_RTCM0_R = alarm_table[0][0];
            while (!(HIB_CTL_R & HIB_CTL_WRC))
                ;
            HIB_IM_R |= HIB_IM_RTCALT0;
            while (!(HIB_CTL_R & HIB_CTL_WRC))
                ;
            HIB_CTL_R |= 0x00000041;

        }

        current++;
        //displayUart0("table updated\n\r");
        valid = true;
        return valid;
    }

    if (isCommand(&data, "on", 0))
    {

        valid = true;
        start = 1;
        PWM1_2_CMPB_R = 250;
        startDMXTX();
        return valid;
    }

    if (isCommand(&data, "controller", 0))
    {
        valid = true;
        writeEeprom(Mode, 0x01);
    }

    if (isCommand(&data, "device", 1))
    {
        uint32_t add1 = getFieldInteger(&data, 1);
//        PWM1_3_CMPB_R = 0;
//        LED_TIMEOUT_OFF = 10;
//                       LED_G = 1;
//                       TIMER2_TAILR_R = 4000000;
//                       TIMER2_IMR_R = TIMER_IMR_TATOIM;
//                       TIMER2_CTL_R |= TIMER_CTL_TAEN;
        writeEeprom(Mode, 0xFFFFFFFF);
        //writeEeprom(Address, add);
        writeEeprom(Address, add1);
        //Address = add;
        add = readEeprom(Address);
        char str[16];
        sprintf(str, "%u", add);
        displayUart0(str);
        valid = true;
        return valid;

    }

    if (isCommand(&data, "off", 0))
    {

        valid = true;
        start = 0;
        PWM1_2_CMPB_R = 0;
        return valid;

    }
    if (isCommand(&data, "get", 1))
    {
        int32_t add = getFieldInteger(&data, 1);
        uint32_t value = DATA[add];
        char str[16];
        sprintf(str, " %u", value);
        displayUart0(str);
        valid = true;
        displayUart0("retrieve from table\n\r");
        return valid;
    }

    if (isCommand(&data, "clear", 0))
    {
        int i = 0;
        for (i = 0; i < max_add; i++)
        {
            DATA[i] = 0;
        }
        valid = true;
        displayUart0("Data Cleared\n\r");
        return valid;
    }

    if (isCommand(&data, "max", 1))
    {
        int32_t max = getFieldInteger(&data, 1);
        max_add = max + 2;
        valid = true;
        displayUart0("set max variable\n\r");
        return valid;
    }

    if (isCommand(&data, "poll", 0))
    {
        poll_request = true;
        valid = true;
        startDMXTX();
        displayUart0("Poll Requested\n\r");
        return valid;
    }
    if (isCommand(&data, "time", 3))
    {
        hr = getFieldInteger(&data, 1);
        min = getFieldInteger(&data, 2);
        sec = getFieldInteger(&data, 3);

        current_time = sec + 60 * (min + 60 * (hr + 24 * (month * 30 + date)));
        while (!(HIB_CTL_R & HIB_CTL_WRC))
            ;
        HIB_RTCLD_R = current_time;
        valid = true;

        displayUart0("Time set\n\r");
        return valid;
    }
    if (isCommand(&data, "time", 0))
    {
        while (!(HIB_CTL_R & HIB_CTL_WRC))
            ;
        current_time = HIB_RTCC_R;
        month = current_time / (60 * 60 * 24 * 30);
        date = (current_time / (60 * 60 * 24)) - (month * 30);
        float Total_hr = 0.0;
        Total_hr = ((float) current_time / (60 * 60));
        uint32_t int_hr = Total_hr;
        hr = int_hr - (((month * 30) + date) * 24);
        float hr_dec = (float) int_hr - Total_hr;
        min = (-hr_dec) * 60;
        //min1 = (current_time / (60)) - (month * (60 * 60 * 24 * 30)) - (date * (60 * 60 * 24)) - (hr * 60);
        sec = (current_time) - (month * (60 * 60 * 24 * 30))
                - (date * (60 * 60 * 24)) - (hr * 60 * 60) - (min * 60);
        char hr1[16];
        sprintf(hr1, " %u", hr);
        displayUart0(hr1);
        char min1[16];
        sprintf(min1, " %u", min);
        displayUart0(min1);
        char sec1[16];
        sprintf(sec1, " %u", sec);
        displayUart0(sec1);
        valid = true;
        displayUart0("\n\r Time displayed \n\r");

        return valid;
    }

    if (isCommand(&data, "date", 2))
    {
        month = getFieldInteger(&data, 1);
        date = getFieldInteger(&data, 2);
        current_time = sec + 60 * (min + 60 * (hr + 24 * (month * 30 + date)));
        while (!(HIB_CTL_R & HIB_CTL_WRC))
            ;
        HIB_RTCLD_R = current_time;
        valid = true;

        displayUart0("Date set\n\r");
        return valid;
    }
    if (isCommand(&data, "date", 0))
    {
        while (!(HIB_CTL_R & HIB_CTL_WRC))
            ;
        current_time = HIB_RTCC_R;
        month = current_time / (60 * 60 * 24 * 30);
        date = (current_time / (60 * 60 * 24)) - (month * 30);
        char month1[16];
        sprintf(month1, " %u", month);
        displayUart0(month1);
        char date1[16];
        sprintf(date1, " %u", date);
        displayUart0(date1);

        valid = true;

        displayUart0("Date displayed\n\r");
        return valid;
    }

    if (!valid)
    {
        displayUart0("Invalid command\n\r");
        return valid;
    }
    return valid;
}

void alarmIsr()
{
    HIB_IC_R |= HIB_IC_RTCALT0;

    DATA[(alarm_table[0][1]) - 1] = alarm_table[0][2];
    if (alarm_table[1][0] == 0)
    {
        current = 0;
    }
    for (i = 0; i < current; i++)
    {
        for (j = 0; j < 3; j++)
        {
            alarm_table[i][j] = alarm_table[i + 1][j];
        }
    }

    while (!(HIB_CTL_R & HIB_CTL_WRC))
        ;
    HIB_RTCM0_R = alarm_table[0][0];
//    HIB_CTL_R |= 0x00000040;
//    while (!(HIB_CTL_R & HIB_CTL_WRC))
//        ;
//    HIB_RTCM0_R = alarm_table[soonest][0];
//    while (!(HIB_CTL_R & HIB_CTL_WRC))
//        ;
//    HIB_IM_R |= HIB_IM_RTCALT0;
//    while (!(HIB_CTL_R & HIB_CTL_WRC))
//        ;
//    HIB_CTL_R |= 0x00000041;

}
void UART0ISR()
{
    if (UART0_FR_R & UART_FR_TXFE)
    {

        UART0_DR_R = chartxBuffer[ReadIndex];
        ReadIndex = (ReadIndex + 1) % BUFFER_LENGTH;
        if (WriteIndex == ReadIndex)
        {
            UART0_IM_R &= ~(UART_IM_TXIM);
            ReadIndex = 0;
            WriteIndex = 0;
        }
    }

}

void displayUart0(char str[])
{

    bool full;

    full = ((WriteIndex + 1) % BUFFER_LENGTH) == ReadIndex;

    if (!full)
    {
        while (str[WriteIndex] != '\0')
        {
            chartxBuffer[WriteIndex] = str[WriteIndex];
            WriteIndex = (WriteIndex + 1) % BUFFER_LENGTH;
        }
        if (UART0_FR_R & UART_FR_TXFE)
        {
            UART0_DR_R = chartxBuffer[ReadIndex];
            ReadIndex = (ReadIndex + 1) % BUFFER_LENGTH;
        }

        UART0_IM_R = UART_IM_TXIM;
    }

}

void UART1ISR()
{
//step 8
    if (UART1_MIS_R & UART_MIS_RXMIS)
    {
        //if (UART1_RSR_R&UART_RSR_BE)
        //{
        //  displayUart0("u");
        //}
        data = UART1_DR_R;
        if (UART1_DR_R & UART_DR_BE)
        {

            rx_phase = 0;
            PWM1_3_CMPA_R = DATA[add + 1];
             PWM1_2_CMPB_R = DATA[add];
             /*LED_TIMEOUT_OFF = 10;
             LED_RED = 1;
             TIMER2_TAILR_R = 4000000;
             TIMER2_IMR_R = TIMER_IMR_TATOIM;
             TIMER2_CTL_R |= TIMER_CTL_TAEN;
             */
            //PWM1_2_CMPB_R = DATA[add];
            PWM1_3_CMPB_R = 250;

        }
        else
        {
            DATA[rx_phase] = (data);
            rx_phase++;

        }
//        if (UART1_DR_R == 0xF7)
//        {
//            GPIO_PORTB_DATA_R = 0; //Break
//        phase = 0;
//        TIMER1_TAILR_R = 7040;
//        if (DATA[rx_phase] == 1)
//        {
//                DE = 1;
//               GPIO_PORTB_DATA_R = 0; //Break
//               phase = 0;
//               TIMER1_TAILR_R = 7040;
//               DE=0;
//        }
//
//
//        }
    }

//step 6

    if (UART1_MIS_R & UART_MIS_TXMIS)
    {
        if (!(poll_request))
        {
            if ((phase - 2) < max_add)
            {
                while (UART1_FR_R & UART_FR_TXFF)
                    ;
                UART1_DR_R = DATA[phase - 2];
                phase++;
            }
            else if ((phase - 2) == max_add)
            {

                UART1_IM_R &= ~UART_IM_TXIM;
                GPIO_PORTB_AFSEL_R &= ~(UART1_TX_MASK | UART1_RX_MASK);
                GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M);
                while (UART1_FR_R & UART_FR_BUSY)
                    ;
                if (start == 1)
                {
                    startDMXTX();
                }
                else if (start == 0)
                {
                    UART1_IM_R &= ~UART_IM_TXIM;
                }

            }
        }

//        if (poll_request)
//        {
//            uint32_t z=0;
//
//            for (z=0;z<512;z++)
//            {
//                if (z=acked)
//                {
//                    ACK[z]=1;
//                }
//                else
//                {
//                    ACK[z]=0;
//                }
//            }
//        }

    }
}

void Timer1Isr()
{
    if (poll_request) //polling
    {
        if (phase == 0)
        {
            GPIO_PORTB_DATA_R = 0x00000002;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            TIMER1_TAILR_R = 480;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            phase = 1;
        }

        else if (phase == 1)
        {
            TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            TIMER1_TAILR_R = 7040;

            GPIO_PORTB_AFSEL_R |= UART1_TX_MASK;
            GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX;
            UART1_DR_R = 0xF7;
            phase = 2;
            UART1_IM_R |= UART_IM_TXIM;

        }
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    }

    if (!(poll_request))
    {
        if (phase == 0)
        {
            GPIO_PORTB_DATA_R = 0x00000002;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            TIMER1_TAILR_R = 480;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;

            phase = 1;
        }

        else if (phase == 1)
        {
            TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
            TIMER1_TAILR_R = 7040;

            GPIO_PORTB_AFSEL_R |= UART1_TX_MASK;
            GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX;
            UART1_DR_R = 0;
            phase = 2;
            UART1_IM_R |= UART_IM_TXIM;

        }
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    }

}

void startDMXTX()
{

    DE = 1;
    GPIO_PORTB_DATA_R = 0; //Break
    phase = 0;
    TIMER1_TAILR_R = 7040;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;

}
void Timer2Isr()
{

    if (LED_TIMEOUT_OFF > 0)
    {
        LED_TIMEOUT_OFF--;

    }

//    else if (LED_TIMEOUT_OFF == 0 && LED_B)
//        {
//            PWM1_3_CMPA_R = 200;
//            LED_BLUE = 0;
//            // TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
//            //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
//        }
//        else if (LED_TIMEOUT_OFF == 0 && LED_R)
//        {
//            PWM1_2_CMPB_R = 200;
//            LED_RED = 0;
//            // TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
//            //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
//        }
//        else if (LED_TIMEOUT_OFF == 0 && LED_G)
//        {
//            PWM1_3_CMPB_R = 200;
//            LED_GREEN = 0;
//            // TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
//            //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
//        }
    else if (LED_TIMEOUT_OFF == 0 && LED_BLUE)
    {
        PWM1_3_CMPA_R = 0;
        LED_BLUE = 0;
        // TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
        //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    }
    else if (LED_TIMEOUT_OFF == 0 && LED_RED)
    {
        PWM1_2_CMPB_R = 0;
        LED_RED = 0;
        // TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
        //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    }
    else if (LED_TIMEOUT_OFF == 0 && LED_GREEN)
    {
        PWM1_3_CMPB_R = 0;
        LED_GREEN = 0;
        // TIMER2_IMR_R &= ~TIMER_IMR_TATOIM;
        //TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    }

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

    USER_DATA data;
    initHw();
    initUart0();
    initUart1();
    initEeprom();
    initHIB();

    if (readEeprom(Mode) == 0xFFFFFFFF) //Device
    {
        if (readEeprom(Address) == 0xFFFFFFFF)
        {
            add = 1;

        }
        else if (readEeprom(Address) != 0xFFFFFFFF)
        {

            add = readEeprom(Address);
        }

        state = true;
        initBacklight();
        PWM1_2_CMPB_R = 0;
        GPIO_PORTB_AFSEL_R |= UART1_RX_MASK;
        GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB0_U1RX;
        UART1_IM_R |= UART_IM_RXIM;
    }

    if (readEeprom(Mode) != 0xFFFFFFFF) //Controller
    {
        initBacklight();
        state = false;
        start = 1;
        PWM1_2_CMPB_R = 250;
        //TIMER2_TAILR_R = 40000000;
        //TIMER2_CTL_R |= TIMER_CTL_TAEN;
        //TIMER2_IMR_R = TIMER_IMR_TATOIM;
        startDMXTX();
    }
    while (1)
    {
        // Get the string from the user
        getsUart0(&data);
        PWM1_3_CMPA_R = 250;
        LED_TIMEOUT_OFF = 10;
        LED_BLUE = 1;
        TIMER2_TAILR_R = 4000000;
        TIMER2_IMR_R = TIMER_IMR_TATOIM;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;

        // Echo back to the user of the TTY interface for testing
        putsUart0(data.buffer);

        // Parse fields
        parseFields(&data);

        // Echo back the parsed field information (type and fields)

        putcUart0('\n');
        putcUart0('\r');
        //PWM1_2_CMPB_R = 250;

        bool valid = false;
        valid = checkCommand(data);
    }

// while (true);
    return 0;
}
