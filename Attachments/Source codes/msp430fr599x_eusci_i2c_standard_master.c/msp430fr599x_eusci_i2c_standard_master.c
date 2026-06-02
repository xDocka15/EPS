#include <msp430.h> 
#include <stdint.h>

// BQ76920 Configuration
#define BQ_ADDR      0x08       
#define SYS_STAT     0x00
#define CELL1_HI     0x0C
#define CELLBAL1     0x01

#define CB1  0b00000001  
#define CB2  0b00000010  
#define CB3  0b00000100  
//      CB4  0b00001000 is skipped because VC3-VC4 are shorted
#define CB4  0b00010000  

#define BAL_THRESHOLD 20 // 20mV difference

// I2C State Machine - Added TX_DATA for writing
typedef enum { IDLE, TX_ADDR, RX_DATA, SW_TO_RX, TX_DATA } I2C_State;
I2C_State MasterMode = IDLE;

uint8_t i;
uint8_t TransmitRegAddr = 0;
uint8_t ReceiveBuffer[20] = {0};
uint8_t TransmitBuffer[20] = {0}; // Buffer for writing data
uint8_t RXByteCtr = 0;
uint8_t TXByteCtr = 0;           // Counter for writing
uint8_t ReceiveIndex = 0;
uint8_t TransmitIndex = 0;       // Index for writing
uint16_t cell1 = 0;
uint16_t cell1_voltage = 0;
uint16_t cell2 = 0;
uint16_t cell2_voltage = 0;
uint16_t cell3 = 0;
uint16_t cell3_voltage = 0;
uint16_t cell4 = 0;
uint16_t cell4_voltage = 0;
uint16_t batt = 0;
uint16_t batt_voltage = 0;
uint16_t ADCGAIN = 0;
uint8_t ADCGAIN1 = 0;
uint8_t ADCGAIN2 = 0;
int8_t ADCOFFSET = 0;


// Function Prototypes
void initGPIO();
void initClock();
void initI2C();
void BQ_ReadReg(uint8_t reg, uint8_t len);
void I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    
    initClock();
    initGPIO();
    initI2C();

    
    BQ_ReadReg(0x50, 1); // index from 1
    ADCGAIN1 = ReceiveBuffer[0];
    BQ_ReadReg(0x59, 1); // index from 1
    ADCGAIN2 = ReceiveBuffer[0];
    ADCGAIN = 365 + (((ADCGAIN1 & 0x0C) << 1) | ((ADCGAIN2 & 0xE0) >> 5));
    BQ_ReadReg(0x51, 1); // index from 1
    ADCOFFSET = (int8_t)ReceiveBuffer[0];

    // Enable ADC and turn on internal power for measurements
    uint8_t sys_ctrl1_val = 0x10; 
    I2C_Master_WriteReg(BQ_ADDR, 0x04, &sys_ctrl1_val, 1);

    // 2. Clear ALL status bits (Write 0xFF to 0x00)
    uint8_t clear_stat = 0xFF;
    I2C_Master_WriteReg(BQ_ADDR, 0x00, &clear_stat, 1);

    // mandatory as per datasheet
    uint8_t cc_val = 0x19;
    I2C_Master_WriteReg(BQ_ADDR, 0x0B, &cc_val, 1);

    // Give it 250ms to complete the first measurement cycle
    __delay_cycles(4000000);

    __enable_interrupt();

    while(1) {

        // 1. Check if the IC is healthy (XR/Alert check)
        BQ_ReadReg(SYS_STAT, 1);
        if (ReceiveBuffer[0] != 0x00) {
            uint8_t clear_stat = ReceiveBuffer[0]; 
            I2C_Master_WriteReg(BQ_ADDR, SYS_STAT, &clear_stat, 1); // Clear the fault
        }

        // --- STEP 1: STOP BALANCING TO MEASURE ---
        uint8_t stop_bal = 0x00;
        I2C_Master_WriteReg(BQ_ADDR, CELLBAL1, &stop_bal, 1);

        // --- STEP 2: WAIT FOR VOLTAGES TO SETTLE ---
        // Wait ~250ms to ensure a fresh ADC conversion after balancing stopped.
        __delay_cycles(4000000); 

        BQ_ReadReg(0x2A, 2); // index from 1
        batt = (ReceiveBuffer[0] << 8) | ReceiveBuffer[1];
        batt_voltage = 4 * (uint16_t)(((uint32_t)batt * ADCGAIN) / 1000) + (4 * ADCOFFSET); // page 21 of datasheet

        BQ_ReadReg(CELL1_HI, 10); // index from 1
            
        cell1 = (ReceiveBuffer[0] << 8) | ReceiveBuffer[1]; // index from 0
        cell1_voltage = (uint16_t)(((uint32_t)cell1 * ADCGAIN) / 1000) + ADCOFFSET;
        
        cell2 = (ReceiveBuffer[2] << 8) | ReceiveBuffer[3];
        cell2_voltage = (uint16_t)(((uint32_t)cell2 * ADCGAIN) / 1000) + ADCOFFSET;
        
        cell3 = (ReceiveBuffer[4] << 8) | ReceiveBuffer[5];
        cell3_voltage = (uint16_t)(((uint32_t)cell3 * ADCGAIN) / 1000) + ADCOFFSET;
        
        cell4 = (ReceiveBuffer[8] << 8) | ReceiveBuffer[9];
        cell4_voltage = (uint16_t)(((uint32_t)cell4 * ADCGAIN) / 1000) + ADCOFFSET;

        BQ_ReadReg(0x00, 16); // index from 1


    volatile uint16_t max_v = cell1_voltage;
    volatile uint16_t min_v = cell1_voltage;

    if (cell2_voltage < min_v) min_v = cell2_voltage;
    if (cell2_voltage > max_v) max_v = cell2_voltage;

    if (cell3_voltage < min_v) min_v = cell3_voltage;
    if (cell3_voltage > max_v) max_v = cell3_voltage;

    if (cell4_voltage < min_v) min_v = cell4_voltage;
    if (cell4_voltage > max_v) max_v = cell4_voltage;

    // --- STEP 5: THE GATEKEEPER ---
    uint8_t best_bit = 0; // Default to "No Balancing"

    // Check if the total pack spread is actually wider than our limit
    if ((max_v - min_v) > BAL_THRESHOLD) {
        
        // ONLY if we are in here, do we look for which cell is the culprit.
        // We use a small epsilon (0.001) to handle float rounding errors.
        if (cell1_voltage >= (max_v))      best_bit = CB1;
        else if (cell2_voltage >= (max_v)) best_bit = CB2;
        else if (cell3_voltage >= (max_v)) best_bit = CB3;
        else if (cell4_voltage >= (max_v)) best_bit = CB4;
    }

    if (min_v < 2000 || max_v > 4500) {best_bit = 0;}

    // --- STEP 5: START BALANCING THE HIGHEST CELL ---
    
    if (best_bit != 0) {
        //I2C_Master_WriteReg(BQ_ADDR, CELLBAL1, &best_bit, 1);
        
        // --- STEP 6: BALANCING PERIOD ---
        for(i=0; i<5; i++) {
            __delay_cycles(16000000); // 1 second delay (adjust to your MCLK)
        }
    } else {
        // No cells need balancing, just wait a bit before checking again
        __delay_cycles(16000000); 
    }
    BQ_ReadReg(0x00, 16); // index from 1

    __delay_cycles(1600000); 
    }
}

// --- Added the Missing Write Function ---
void I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
    MasterMode = TX_ADDR;
    TransmitRegAddr = reg_addr;
    CopyArray(reg_data, TransmitBuffer, count);
    TXByteCtr = count;
    RXByteCtr = 0; // Ensure we are not in read mode
    TransmitIndex = 0;

    UCB0I2CSA = dev_addr;
    UCB0CTLW0 |= UCTR + UCTXSTT;             
    __bis_SR_register(LPM0_bits + GIE);      
}

void BQ_ReadReg(uint8_t reg, uint8_t len) {
    MasterMode = TX_ADDR;
    TransmitRegAddr = reg;
    RXByteCtr = len;
    TXByteCtr = 0; // Ensure we are not in write mode
    ReceiveIndex = 0;

    UCB0I2CSA = BQ_ADDR;
    UCB0CTLW0 |= UCTR + UCTXSTT;             
    __bis_SR_register(LPM0_bits + GIE);      
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count) {
    uint8_t i;
    for (i = 0; i < count; i++) dest[i] = source[i];
}

void initGPIO() {
    P1SEL1 |= (BIT6 | BIT7);
    P1SEL0 &= ~(BIT6 | BIT7);
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
    PM5CTL0 &= ~LOCKLPM5;
}

void initI2C() {
    UCB0CTLW0 = UCSWRST;
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; 
    UCB0BRW = 160;
    UCB0I2CSA = BQ_ADDR;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE = UCNACKIE + UCTXIE + UCRXIE;
}

void initClock() {
    FRCTL0 = FRCTLPW | NWAITS_1;
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_4 | DCORSEL; 
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCNACKIFG:
            UCB0CTLW0 |= UCTXSTP;
            __bic_SR_register_on_exit(LPM0_bits);
            break;
            
        case USCI_I2C_UCTXIFG0:
            switch (MasterMode) {
                case TX_ADDR:
                    UCB0TXBUF = TransmitRegAddr;
                    if (RXByteCtr) MasterMode = SW_TO_RX; // Transition to Read
                    else MasterMode = TX_DATA;           // Transition to Write Data
                    break;

                case TX_DATA:
                    if (TXByteCtr) {
                        UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                        TXByteCtr--;
                    } else {
                        UCB0CTLW0 |= UCTXSTP;
                        MasterMode = IDLE;
                        __bic_SR_register_on_exit(LPM0_bits);
                    }
                    break;

                case SW_TO_RX:
                    UCB0CTLW0 &= ~UCTR;
                    UCB0CTLW0 |= UCTXSTT;
                    if (RXByteCtr == 1) {
                        while(UCB0CTLW0 & UCTXSTT);
                        UCB0CTLW0 |= UCTXSTP;
                    }
                    MasterMode = RX_DATA;
                    break;
                default: break;
            }
            break;

        case USCI_I2C_UCRXIFG0:
            ReceiveBuffer[ReceiveIndex++] = UCB0RXBUF;
            RXByteCtr--;
            if (RXByteCtr == 1) {
                UCB0CTLW0 |= UCTXSTP;
            } else if (RXByteCtr == 0) {
                MasterMode = IDLE;
                __bic_SR_register_on_exit(LPM0_bits);
            }
            break;
    }
}

