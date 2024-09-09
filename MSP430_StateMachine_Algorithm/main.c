#include <msp430.h> 

#define LED_PIN     BIT0       // P3.0 LED Pin
#define RS485_R     BIT5      // P3.5 Receive Pin(R)
#define RS485_RE    BIT3      // P3.3 Receiver Enable(active-low)
#define RS485_DE    BIT7      // P3.7 Driver Enable(high for TX, low for RX)
//#define RS485_D     BIT4      // P3.4 Transmit pin(D)


//clock
void initClock(){
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_3;
    CSCTL2 = SELS_3 | SELM_3;
    CSCTL3 = DIVS_0 | DIVM_0;
    CSCTL0_H = 0;
}


//State Machine States

typedef enum {
    IDLE_STATE,
    RECEIVING_STATE,
    PROCESSING_STATE
}State;

volatile State currentState = IDLE_STATE;
volatile unsigned char rxBuffer[256];  // Buffer to store received data
volatile unsigned int rxIndex = 0;    // Index for received data
volatile unsigned int txReady = 1;

void initClock();
void initUART();
void initGPIO();
void enterLowPowerMode();
void processReceivedData();
void uartTransmitChar(char c);
void uartSendString(char *str);
void uartTransmitHexByte(unsigned char byte);




void initUART(){

    P3SEL1 &= ~BIT4;            // Clear P3SEL1 bit 4
    P3SEL0 |= BIT4;             // Set P3SEL0 bit 4

    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;

    UCA1BRW = 26;
    UCA1MCTLW |= (0x00 << 4) | (0x54 << 8) | UCOS16;  // UCBRSx = 0x54, UCBRFx = 0, enable UCOS16

    PM5CTL0 &= ~LOCKLPM5;
    UCA1CTLW0 &= ~UCSWRST;

    UCA1CTLW0 &= ~ UCSWRST;
    UCA1IE |= UCRXIE;            // Enable receive interrupt
    UCA1IE |= UCTXIE;           // Enable transmit interrupt
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
     // Initialize eUSCI_A1 module
    __enable_interrupt();
    uartSendString("Started....");
}

void initGPIO(){

    // Configure RS485 control pins
    P3DIR |=  RS485_RE | RS485_DE;     // Set RE and DE as output
    P3OUT &= ~RS485_DE;                // Disable transmitter
    P3OUT |= RS485_RE;                 // Enable receiver(active low)

    //Configure LED pin
    P3DIR |= LED_PIN;
    P3OUT &= ~LED_PIN;                // Turn off LED
    uartSendString("Starting....");

}



// Process received Modbus data(placeholder for further processing)
void processReceivedData(){
    // Process the received data (parsing Modbus Frame)
    // For now, just reset the index and turn off the LED
    rxIndex = 0;
    P3OUT &= ~LED_PIN; // Turn off LED
}

void enterLowPowerMode(){
    __bis_SR_register(LPM0_bits + GIE);     //Enter LPM0 with interrupt enabled
}

// Function to transmit a character over UART
void uartTransmitChar(char c){
    while(!(UCA1IFG & UCTXIFG)); // Wait until the buffer is ready
    UCA1TXBUF = c;

}

// Function to transmit a string over UART
void uartSendString(char *str)
{
    while(*str != '\0')
    {
      uartTransmitChar(*str++);
    }
}

// Function to transmit a byte in hex format over UART

void uartTransmitHexByte(unsigned char byte){
    const char hexDigits[] = "0123456789ABCDEF";
    uartTransmitChar(hexDigits[(byte >> 4)& 0xFF]);  // Transmit high nibble
    uartTransmitChar(hexDigits[byte & 0x0F]); // Transmit low nibble
}

void main(){
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

    //Initialize the functions
    initClock();
    initUART();
    initGPIO();

    __enable_interrupt();   // Enable Global interrupts
    uartSendString("Starting sniffer...\r\n");  // Notify that sniffer has started
    while(1){
        switch(currentState){
            case IDLE_STATE:
                // Stay idle, waiting for data
                uartSendString("Waiting for data...\r\n");
                enterLowPowerMode(); // Wait in LPM, wake up on interrupt
                break;
            case RECEIVING_STATE:
                // Data receiving is handled in the UART ISR
                uartSendString("Data received: ");
                currentState = PROCESSING_STATE; // Move to processing state
                break;
            case PROCESSING_STATE:
                // Process the data received in the buffer
                processReceivedData();
                currentState = IDLE_STATE;      // Return to idle state after processing
                break;

            default:
                currentState = IDLE_STATE;
                break;
        }
    }
}

// UART receive ISR

#pragma vector= USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    if (UCA1IFG & UCRXIFG) {
        unsigned char receivedByte = UCA1RXBUF;  // Read the received byte
        rxBuffer[rxIndex++] = receivedByte;      // Store in buffer
        if (rxIndex >= sizeof(rxBuffer)) {
            rxIndex = 0;  // Prevent buffer overflow
        }
        uartTransmitHexByte(receivedByte);       // Transmit the byte in hex format
        //uartTransmitString("\r\n");

        currentState = RECEIVING_STATE;  // Set the state to receiving

        P3OUT |= LED_PIN;                // Turn on LED when data is received
    }
    if (UCA1IFG & UCTXIFG) {
        // Transmit is complete, exit low power mode and mark TX as ready
        txReady = 1;
        __bic_SR_register_on_exit(LPM0_bits);  // Exit LPM0
    }
}
