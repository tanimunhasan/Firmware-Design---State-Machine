/* Date: 20.09.2024
  Author: Sayed Tanimun Hasan
  Project: Modbus Data Snipping
  Microcontroller: MSP430FR5987 Texas Insrument
  IDE: Code Composer Studio 
*/

#include <msp430.h>
#include<stdio.h>
#include<stdint.h>

#define BUFFER_SIZE    288 // Size of buffer to hold captured Modbus frames
#define TIMEOUT_THRESHOLD  14560  // 1.82 ms timeout at 8MHz clock for 19200

// Define state machine states
typedef enum {
    STATE_IDLE,
    STATE_RECEIVE_FRAME,
    STATE_PROCESS_FRAME,
    STATE_TRANSMIT_BUFFER
} SystemState;

volatile char bufferA[BUFFER_SIZE];  // Size of the buffer to hold captured Modbus frames
volatile char bufferB[BUFFER_SIZE];  // Second buffer for Modbus data
volatile char* activeBuffer = bufferA;  // Active buffer pointer (initially bufferA)
volatile char* sendBuffer = 0;       // Buffer to send to host

volatile unsigned int bufferIndex = 0;  // Index to track buffer position in the active buffer
volatile unsigned int bufferReadyToSend = 0; // Flag to indicate a buffer is ready to send
SystemState currentState = STATE_IDLE;  // Initialize state to IDLE

// Function to configure UART for Modbus communication (19200 baud, 8N1)
void configureClocks(void) {
    CSCTL0_H = CSKEY >> 8;             // Unlock CS registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;      // Set DCO to 8MHz
    //CSCTL2 = SELS__DCOCLK;             // Set SMCLK = DCO
    CSCTL3 = DIVS__1;                  // Set SMCLK divider to 1
    CSCTL0_H = 0;                      // Lock CS registers
}

// UART function to transmit single byte
void uartTransmitChar(char c) {
    while (!(UCA1IFG & UCTXIFG)); // Wait until the transmit buffer is empty
    UCA1TXBUF = c;                // Send character
}

// UART function to transmit a string
void uartTransmitString(const char* str) {
    while (*str) {                // Loop through the string until buffer is empty
        uartTransmitChar(*str);   // Send each character
        str++;                    // Move to the next character
    }
}


// UART function to transmit a single byte as a two-digit hex value
void uartTransmitHexByte(unsigned char byte) {
    const char hexChars[] = "0123456789ABCDEF";
    uartTransmitChar(hexChars[(byte >> 4) & 0x0F]);  // Send high nibble
    uartTransmitChar(hexChars[byte & 0x0F]);         // Send low nibble
}
// UART function to transmit the contents of a buffer as hex
void uartTransmitBufferHex(const char* buffer, unsigned int length) {
    unsigned int i;
    uartTransmitString("\r\nBuffer (Hex): ");
    for (i = 0; i < length; i++) {
        uartTransmitHexByte(buffer[i]);  // Send each byte in hex
        uartTransmitChar(' ');           // Add a space between bytes
    }
    uartTransmitString("\r\n");          // New line after buffer output
}

// UART function to transmit the contents of a buffer
void uartTransmitBuffer(const char* buffer, unsigned int length) {
    unsigned int i;
    uartTransmitString("\r\n (Hex): ");
    for (i = 0; i < length; i++) {
        uartTransmitHexByte(buffer[i]);  // Send each byte in the buffer
        uartTransmitChar(' ');
    }
    uartTransmitString("\r\n");       // New line after buffer output
}


// Function to configure UART
void configureUART() {
    // Set UART configuration: 19200 baud rate, 8 data bits, no parity, 1 stop bit
    UCA1CTL1 |= UCSWRST;                    // Put eUSCI in reset
    UCA1CTL1 |= UCSSEL_2;                   // SMCLK as clock source

    UCA1BRW = 26;                           // Baud rate 19200 for 8MHz clock
    UCA1MCTLW = (0x54 << 8) | (0x00 << 4) | UCOS16; // Modulation oversampling

    PM5CTL0 &= ~LOCKLPM5;       // Turn on I/O
    UCA1CTL1 &= ~UCSWRST;                   // Initialize eUSCI
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
}

void UARTRxTx(void) {
    // Setup Ports
    P3SEL1 &= ~BIT4;            // Clear P3SEL1 bit 4 (TXD)
    P3SEL0 |= BIT4;             // Set P3SEL0 bit 4 (TXD)

    P3SEL1 &= ~BIT5;            // Clear P3SEL1 bit 5 (RXD)
    P3SEL0 |= BIT5;             // Set P3SEL0 bit 5 (RXD)
}

// Configure P3.0 as output to control LED
void configureLED(void) {
    P3DIR |= BIT0;    // Set P3.0 as output (LED)
    P3OUT &= ~BIT0;   // Start with LED off
}

// Toggle the LED connected to P3.0
void toggleLED(void) {
    P3OUT ^= BIT0;    // Toggle P3.0 (LED)
}

// Example function to initialize RS485 transceiver in receive mode
void configureRS485() {
    P3DIR |= BIT7;                          // Set RS485 DE (Driver Enable) pin as output
    P3OUT &= ~BIT7;                         // DE = 0 (receive mode)

    P3DIR |= BIT3;                          // Set RS485 RE (Receive Enable) pin as output
    P3OUT &= ~BIT3;                         // RE = 0 (receive mode)
}

void configureTimer(void) {
    TA0CCTL0 = CCIE;                        // Enable interrupt for Timer_A capture/compare
    TA0CCR0 = TIMEOUT_THRESHOLD;            // Set compare value for timeout (1.82 ms)
    TA0CTL = TASSEL_2 + MC_0;               // Use SMCLK (8MHz), stop the timer initially
}

void startTimer(void) {
    TA0R = 0;                               // Reset timer counter
    TA0CTL |= MC_1;                         // Start timer in up mode
}

void stopTimer(void) {
    TA0CTL &= ~MC_1;                        // Stop the timer
}

// Function prototype for CRC16 calculation
unsigned int calculateCRC16(const char* data, unsigned int length);

// Function to validate received Modbus frame
int isValidFrame(const char* buffer, unsigned int length) {
    // Expected frame length (can be adjusted according to the actual expected length)
    const unsigned int expectedFrameLength = 100; // Adjust this as per the actual frame length

    // 1. Check if the received length matches the expected frame length
    if (length != expectedFrameLength) {
        return 0; // Frame is invalid due to incorrect length
    }

    // 2. Check the start byte (typically, Modbus addresses are in the range 0x01 to 0xF7)
    if (buffer[0] != 0x01) {
        return 0; // Frame is invalid if it doesn't start with the correct address
    }

    // 3. Calculate the CRC of the received data excluding the last 2 bytes (CRC)
    unsigned int receivedCRC = (buffer[length - 2] & 0xFF) | ((buffer[length - 1] & 0xFF) << 8);
    unsigned int calculatedCRC = calculateCRC16(buffer, length - 2);

    // 4. Compare calculated CRC with the received CRC
    if (receivedCRC != calculatedCRC) {
        return 0; // Frame is invalid due to CRC mismatch
    }

    // If all checks pass, the frame is valid
    return 1;
}

// CRC16 calculation for Modbus (Polynomial: 0xA001)
unsigned int calculateCRC16(const char* data, unsigned int length) {
    unsigned int crc = 0xFFFF;        // Initialize CRC to 0xFFFF
    unsigned int i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];               // XOR byte into the current CRC value
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;        // Apply polynomial
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ISR for receiving Modbus data
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    if (UCA1IFG & UCRXIFG) {                      // Check if data has been received
        char receivedByte = UCA1RXBUF;            // Read received data

        // Reset and start timer for timeout detection
        stopTimer();
        startTimer();

        // Store the received byte in buffer if there's space
        if (bufferIndex < BUFFER_SIZE) {
            activeBuffer[bufferIndex++] = receivedByte;
        } else{

        }
    }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) {
    // Modbus frame timeout occurred (end of frame detected)
    stopTimer();
    if(bufferIndex>0){
    bufferReadyToSend = 1; // Set the flag to signal the main loop

    // Swap buffers: make the current buffer ready to send and switch to the next buffer
    sendBuffer = activeBuffer;          // Assign activeBuffer to sendBuffer
    activeBuffer = (activeBuffer == bufferA) ? bufferB : bufferA; // Switch to the other buffer
    }
    //bufferIndex = 0;                    // Reset the buffer index
}

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    configureClocks();
    configureUART();                    // Initialize UART
    UARTRxTx();
    configureRS485();                   // Configure RS485 transceiver in receive mode
    configureLED();
    configureTimer();
    __bis_SR_register(GIE);             // Enable global interrupts

    while (1) {
        switch (currentState) {
            case STATE_IDLE:
                // Waiting for buffer to be ready
                if (bufferReadyToSend) {
                    bufferReadyToSend = 0;
                    currentState = STATE_PROCESS_FRAME;  // Move to processing state
                }
                break;

            case STATE_PROCESS_FRAME:
                if(isValidFrame((const char *)sendBuffer, bufferIndex)){
                    currentState = STATE_TRANSMIT_BUFFER;
                }else{

                    currentState = STATE_IDLE;
                }
                currentState = STATE_TRANSMIT_BUFFER;  // Move to transmit state
                break;

            case STATE_TRANSMIT_BUFFER:
                if(sendBuffer != NULL){
                uartTransmitBufferHex((const char *) sendBuffer, bufferIndex);  // Transmit buffer contents
                toggleLED();
                }
                bufferIndex = 0; // only reset buffer index after the frame is processed
                currentState = STATE_IDLE;            // Return to idle state
                break;

            default:
                currentState = STATE_IDLE;            // Fallback to idle in case of unexpected state
                break;
        }
    }
}
