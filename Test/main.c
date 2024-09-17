#include <msp430.h>

#define BUFFER_SIZE    128 // Size of buffer to hold captured Modbus frames
#define TIMEOUT_THRESHOLD  1820  // 1.82 MS FOR 3.5 character times at 19200 baud rate

volatile char unsigned modbusBuffer[BUFFER_SIZE];
volatile unsigned int bufferIndex = 0;
volatile unsigned int modbusFrameComplete = 0;

// Funciton to configure UART for Modbus communication (19200 baud, 8N1)
void configureClocks(void)
{
    CSCTL0_H = CSKEY >> 8;             // Unlock CS registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;      // Set DCO to 8MHz
    CSCTL2 = SELS__DCOCLK;             // Set SMCLK = DCO
    CSCTL3 = DIVS__1;                  // Set SMCLK divider to 1
    CSCTL0_H = 0;                      // Lock CS registers
}

    // Example function to configure UART

void configureUART()
{
    // Set UART configuration: 9600 baud rate, 8 data bits, no parity, 1 stop bit
    UCA1CTL1 |= UCSWRST;                    // Put eUSCI in reset
    UCA1CTL1 |= UCSSEL_2;                   // SMCLK as clock source

    UCA1BRW = 26;                          // Baud rate 9600 for 1MHz clock

    UCA1MCTLW = (0x54 << 8) | (0x00 << 4) | UCOS16;;                     // Modulation UCBRSx=0xD6, UCBRFx=0, oversampling

    UCA1CTL1 &= ~UCSWRST;                   // Initialize eUSCI
    UCA1IE |= UCRXIE;                       // Enable USCI_A0 RX interrupt
}

// Example function to initialize RS485 transceiver in receive mode
void configureRS485() {
    P3DIR |= BIT7;                          // Set RS485 DE (Driver Enable) pin as output
    P2OUT &= ~BIT7;                         // DE = 0 (receive mode)

    P3DIR |= BIT3;                          // Set RS485 RE (Receive Enable) pin as output
    P3OUT &= ~BIT3;                         // RE = 0 (receive mode)
}

void configureTimer(void){
    TA0CCTL0 = CCIE;                        // Enable interrupt for Timer_A capture/compare
    TA0CCR0 = TIMEOUT_THRESHOLD;            // Set compare value for timeout (1.82 ms)
    TA0CTL =TASSEL_2 + MC_0;                // Use SMCLK"(8MHz), stop the timer initially
}

void startTimer(void){
    TA0R = 0;                               // Reset timer counter
    TA0CTL |= MC_1;                         // Start timer in up mode
}

void stopTimer(void){

    TA0CTL &= ~MC_1;                        // Stop the timer
}

void sendBufferToPC(void){

    unsigned int i;
    for(i = 0; i<bufferIndex; i++){
        while(!(UCA1IFG & UCTXIFG));     // Wait until UART TX buffer is ready
        UCA1TXBUF = modbusBuffer[i];    // Send each byte from buffer to UART
    }
    bufferIndex = 0;                    // Reset buffer index after sending
}

// Example ISR for receiving Modbus data
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    if (UCA1IFG & UCRXIFG) {                      // Check if data has been received
        char receivedByte = UCA1RXBUF;            // Read received data

        // Reset and start timer for timeout detection
        stopTimer();
        startTimer();


        //Store the received byte in buffer if there's space
        if(bufferIndex<BUFFER_SIZE){
            modbusBuffer[bufferIndex++] = receivedByte;
        } else{
            // Handle buffer overflow : send the current buffer and reset
            sendBufferToPC();           // Send buffer contents to host
            modbusBuffer[0] = receivedByte;  // Store the current byte in new buffer
            bufferIndex = 1;              // Reset buffer index to 1 (since byte is stored)
        }

    }
}

#pragma vector=TIMER_A0_VECTOR
__interrupt void TIMER_A0_ISR(void){
    // Modbus frame timeout occurred (end of frame detected)
    stopTimer();
    modbusFrameComplete = 1;

    __bic_SR_register_on_exit(LPM0_bits);   // Exit low power mode after timeout
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    configureClocks();
    configureUART();                        // Initialize UART
    configureRS485();                       // Configure RS485 transceiver in receive mode

    __bis_SR_register(GIE);                 // Enable global interrupts
    while (1) {
        // Main loop: wait for modbus frame to be complete
        if(modbusFrameComplete){
            modbusFrameComplete= 0;     //Reset frame complete flag
            sendBufferToPC();           //Send buffer content to PC via UART
        }
    }
}
