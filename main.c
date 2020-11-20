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


#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "uart_input.h"
#include "uart1.h"
#define DE (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))


#define DE_MASK 128

uint32_t ReadIndex = 0;
uint32_t WriteIndex = 0;
uint32_t BUFFER_LENGTH =100;
char chartxBuffer[100];
uint32_t phase=0;
uint32_t max_add=512;
uint32_t DATA[512];
uint32_t start=0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5|SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    _delay_cycles(3);


    GPIO_PORTC_DIR_R |= DE_MASK;
    GPIO_PORTC_DEN_R |= DE_MASK;


    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 7040;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
                     // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
          //TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on time

}
bool checkCommand(USER_DATA data)
{
    bool valid=false;
        if (isCommand(&data, "set", 2))
        {
            int32_t add = getFieldInteger(&data, 1);
            int32_t Data= getFieldInteger(&data, 2);

            DATA[add]=Data;
            //displayUart0("table updated\n\r");
            valid = true;
            return valid;
        }

        if (isCommand(&data, "on", 0))
            {

                valid = true;
               start=1;
                //displayUart0("DATA TRANSMIT TURNED ON\n\r");
                startDMXTX();
                //TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

                return valid;
            }


        if (isCommand(&data, "off", 0))
            {

                valid = true;
                start=0;
                return valid;
            }
        if (isCommand(&data, "get", 1))
        {
            int32_t add = getFieldInteger(&data, 1);
            char* str = getFieldString(&data, 1);

            displayUart0(DATA[add]);
            valid = true;
            displayUart0("retrieve from table\n\r");
            return valid;
        }

        if (isCommand(&data, "clear", 0))
        {
            int i =0;
            for(i=0;i<max_add;i++)
            {
                DATA[i]=0;
            }
            valid = true;
            displayUart0("Data Cleared\n\r");
            return valid;
        }

        if (isCommand(&data, "max_add", 1))
        {
            char* str = getFieldString(&data, 1);
            valid = true;
            displayUart0("set max variable\n\r");
            return valid;
        }

        if (isCommand(&data, "tx_on", 0))
        {
            char* str = getFieldString(&data, 1);
            valid = true;
            displayUart0("Transmit on\n\r");
            return valid;
        }

        if (isCommand(&data, "tx_off", 0))
        {
            char* str = getFieldString(&data, 1);
            valid = true;
            displayUart0("Transmit off\n\r");
            return valid;
        }

        if (isCommand(&data, "requestpoll", 1))
        {
            char* str = getFieldString(&data, 1);
            valid = true;
            displayUart0("poll requested\n\r");
            return valid;
        }

        if (isCommand(&data, "set_time", 3))
        {
            int32_t hour = getFieldInteger(&data, 1);
            int32_t minute = getFieldInteger(&data, 2);
            int32_t second = getFieldInteger(&data, 3);
            valid = true;
            int32_t total= hour+minute+second;
            displayUart0("Time Set  \n\r");
            return valid;
        }

        if (isCommand(&data, "get_time", 3))
        {
            int32_t hour = getFieldInteger(&data, 1);
            int32_t minute = getFieldInteger(&data, 2);
            int32_t second = getFieldInteger(&data, 3);
            valid = true;
            displayUart0("Got Time\n\r");
            return valid;
        }

        if (isCommand(&data, "set_date", 2))
        {
            int32_t month = getFieldInteger(&data, 1);
            int32_t day = getFieldInteger(&data, 2);
            valid = true;
            displayUart0("Date Set\n\r");
            return valid;
        }

        if (isCommand(&data, "get_date", 2))
        {
            int32_t month = getFieldInteger(&data, 1);
            int32_t day = getFieldInteger(&data, 2);
            valid = true;
            displayUart0("Got date\n\r");
            return valid;
        }

        if (!valid)
        {
            displayUart0("Invalid command\n\r");
            return valid;
        }
        return valid;
}

void UART0ISR()
{
    if(UART0_FR_R &UART_FR_TXFE)
    {

        UART0_DR_R = chartxBuffer[ReadIndex];
        ReadIndex=(ReadIndex+1)% BUFFER_LENGTH;
        if(WriteIndex==ReadIndex)
           {
               UART0_IM_R &= ~(UART_IM_TXIM);
               ReadIndex=0;
               WriteIndex=0;
           }
    }

}

void displayUart0(char str[])
{

    bool full;

    full = ((WriteIndex+1) % BUFFER_LENGTH) == ReadIndex;

    if (!full)
    {
        while(str[WriteIndex]!='\0')
        {
            chartxBuffer[WriteIndex] = str[WriteIndex];
            WriteIndex=(WriteIndex+1) % BUFFER_LENGTH;
        }
        if(UART0_FR_R & UART_FR_TXFE)
               {
                   UART0_DR_R = chartxBuffer[ReadIndex];
                   ReadIndex=(ReadIndex+1)% BUFFER_LENGTH;

               }

        UART0_IM_R = UART_IM_TXIM;
     }

}


void Timer1Isr()
{
    //putsUart0("Hi");

       //
        if (phase==0)
            {
                GPIO_PORTB_DATA_R=0x00000002;
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
                TIMER1_TAILR_R=480 ;
                TIMER1_CTL_R |= TIMER_CTL_TAEN;

                phase = 1;
            }

        else if (phase==1)
            {


                TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
                TIMER1_TAILR_R= 7040;

                GPIO_PORTB_AFSEL_R |= (UART1_TX_MASK | UART1_RX_MASK);
               // GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M);
                GPIO_PORTB_PCTL_R |= (GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX);
                UART1_DR_R = 0;

                UART1_IM_R |= UART_IM_TXIM;
                phase = 2;

            }

        TIMER1_ICR_R = TIMER_ICR_TATOCINT;

}
void UART1ISR()
{
       if((phase-2)<max_add)
       {
           while (UART1_FR_R & UART_FR_TXFF);
           UART1_DR_R=DATA[phase-2];
           //displayUart0("ok ");
           phase++;
       }
       else if((phase-2)==max_add)
       {
           UART1_IM_R&= ~UART_IM_TXIM;
           GPIO_PORTB_AFSEL_R &= ~(UART1_TX_MASK | UART1_RX_MASK);
           GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M);
           while (UART1_FR_R & UART_FR_BUSY);
           if (start==1)
           {
               //DE=0;
               startDMXTX();
           }
           else if(start==0)
           {
               displayUart0("doneee");
               UART1_IM_R &= ~UART_IM_TXIM;

           }

       }



}

void startDMXTX()
{
    DE=1;
    GPIO_PORTB_DATA_R=0; //Break
    phase=0;
    TIMER1_TAILR_R=7040 ;

    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;


    //TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
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

    while(1)
    {
    // Get the string from the user
    getsUart0(&data);

    // Echo back to the user of the TTY interface for testing
   putsUart0(data.buffer);

    // Parse fields
    parseFields(&data);

    // Echo back the parsed field information (type and fields)

    putcUart0('\n');
    putcUart0('\r');

    bool valid=false;
    valid=checkCommand(data);
    }
    while (true);
    return 0;
}
