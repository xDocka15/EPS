/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430FR5x9x Demo - Toggle P1.0 using software
//
//  Description: Toggle P1.0 using software.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//           MSP430FR5994
//         ---------------
//     /|\|               |
//      | |               |
//      --|RST            |
//        |               |
//        |           P1.0|-->LED
//
//   William Goh
//   Texas Instruments Inc.
//   October 2015
//   Built with IAR Embedded Workbench V6.30 & Code Composer Studio V6.1
//******************************************************************************
#include <msp430.h>


#define A0        BIT4
#define A1        BIT5
#define A2        BIT6
#define CH1_EN    BIT0
#define ADC1      BIT0
#define ADC2      BIT1

volatile unsigned int adc_value_0 = 0;   // Global variable for CCS debugger
volatile unsigned int adc_value_1 = 0;
volatile unsigned int adc_millivolts_0 = 0;
volatile unsigned int adc_millivolts_1 = 0;
volatile unsigned int ch1_v = 0;
volatile unsigned int sol_v = 0;
volatile unsigned int floating = 0;
volatile unsigned int ameas_1 = 0;
volatile unsigned int batt_temp = 0;
volatile unsigned char adc_channel = 1;

// Initialize the 32.768 kHz crystal as ACLK
void init_clock()
{
    CSCTL0_H = CSKEY >> 8;        // Unlock CS registers

    // Enable LFXT pins (PJ.4 LFXIN, PJ.5 LFXOUT)
    PJSEL0 |= BIT4 | BIT5;

    // Max drive strength for startup
    CSCTL4 |= LFXTDRIVE_3;

    // Turn on LFXT
    CSCTL4 &= ~LFXTOFF;

    // Wait for crystal to stabilize
    do {
        CSCTL5 &= ~LFXTOFFG;   // Clear local fault flag
        SFRIFG1 &= ~OFIFG;     // Clear global fault flag
    } while (SFRIFG1 & OFIFG);

    // Reduce drive strength after stable
    CSCTL4 &= ~LFXTDRIVE_3;

    // Select ACLK = LFXT
    CSCTL2 = (CSCTL2 & ~SELA_7) | SELA__LFXTCLK;

    CSCTL0_H = 0;              // Lock CS registers
}

// Initialize ADC12_B to read P1.0 (A0)
void init_adc()
{
    // --- Configure P1.0 as analog input (A0) ---
    P1SEL1 |= ADC1 | ADC2;
    P1SEL0 |= ADC1 | ADC2;

    // --- Reset ADC registers ---
    ADC12CTL0 = 0;
    ADC12CTL1 = 0;
    ADC12CTL2 = 0;
    ADC12CTL3 = 0;

    // --- ADC configuration ---
    //ADC12CTL0 = ADC12SHT0_2 | ADC12ON;   // Sample time, ADC on
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON | ADC12MSC;
    //ADC12CTL1 = ADC12SHP;             // Use sampling timer
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;                
    ADC12CTL2 = ADC12RES_2;              // 12-bit resolution

    // --- Select input channel A0, reference = AVCC ---
    //ADC12MCTL0 = ADC12INCH_0 | ADC12VRSEL_0;
        ADC12MCTL0 = ADC12INCH_0;              // A0
        ADC12MCTL1 = ADC12INCH_1 | ADC12EOS;   // A1, end of sequence

    // --- Enable interrupt (optional) ---
    ADC12IER0 = ADC12IE0;

    // --- Enable ADC ---
    ADC12CTL0 |= ADC12ENC;
}

// Initialize Timer_A to toggle LED once per second
void init_timer()
{
    PJDIR |= CH1_EN;     // Configure PJ.0 as output (LED)
    PJOUT &= ~CH1_EN;    // LED off

    TA0CCR0 = 32768 - 1;        
    TA0CCTL0 = CCIE;            // Enable interrupt
    TA0CTL = TASSEL__ACLK | MC__UP | TACLR; // ACLK, up mode, clear timer
}

// Timer0_A0 interrupt service routine: toggle LED + sample ADC
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    // Toggle LED
    PJOUT ^= CH1_EN;

    // Start ADC conversion
    ADC12CTL0 |= ADC12SC;

    // Wait until conversion completes
    while (ADC12CTL1 & ADC12BUSY);

    // Store result for debugger
    adc_value_0 = ADC12MEM0;
    adc_value_1 = ADC12MEM1;

    ADC12IFGR0 = 0;

    // Convert to voltage (assuming 3.3 V reference) 
    adc_millivolts_0 = (adc_value_0 * 3300UL) / 4095;
    adc_millivolts_1 = (adc_value_1 * 3300UL) / 4095;

    switch(adc_channel)
    {
        case 1:
            batt_temp = adc_millivolts_0;
            ameas_1 = (adc_millivolts_1-500); // convert voltaage to mA. 0,5V at 0A then 100mV/1000mA

            P4OUT = (P4OUT & ~(A0 | A1 | A2)) | A0;   // A0 HIGH
            adc_channel = 2;
            break;

        case 2:
            sol_v = adc_millivolts_1 * 7; //*7 because of 10:1.6 voltage divider
            floating = adc_millivolts_0;

            P4OUT = (P4OUT & ~(A0 | A1 | A2)) | A1;   // A1 HIGH
            adc_channel = 3;
            break;
        
        case 3:
            ch1_v = adc_millivolts_0 * 2; //*2 because of 1:1 voltage divider


            P4OUT = (P4OUT & ~(A0 | A1 | A2)) ;   // All LOW
            adc_channel = 1; // next cahnnel to be measured
            break;   
    }
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;       // Unlock GPIO

    P3DIR |= BIT1;    // P3.1 output
    P3OUT |= BIT1;    // Set P3.1 HIGH (or LOW if you prefer)

    P4DIR |= A0 | A1 | A2;    // P4.4,5,6 output
    P4OUT |= A1;    // Set P4.5 HIGH (or LOW if you prefer)
    P4OUT &= ~(A0 | A2);    // Set P4.4,6 LOW

    init_clock();               // Start 32.768 kHz crystal
    init_adc();                 // Initialize ADC
    init_timer();               // Start Timer_A

    __enable_interrupt();       // Enable global interrupts

    while (1)
    {
        // Nothing needed here — everything happens in ISR
        __no_operation();       // Helps CCS place a breakpoint
    }
}