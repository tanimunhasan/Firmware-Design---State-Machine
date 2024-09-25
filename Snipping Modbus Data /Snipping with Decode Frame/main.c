#include <msp430.h>
#include <stdio.h>
#include <stdint.h>

#define BUFFER_SIZE    256 // Size of buffer to hold captured Modbus frames
#define TIMEOUT_THRESHOLD  16000  // 1.82 ms timeout at 8MHz clock for 19200 [14560]

volatile char bufferA[BUFFER_SIZE];  // Size of the buffer to hold captured Modbus frames
volatile char bufferB[BUFFER_SIZE];  // Second buffer for Modbus data
volatile char* activeBuffer = bufferA;  // Active buffer pointer (initially bufferA)
volatile char* sendBuffer = 0;       // Buffer to send to host

volatile unsigned int bufferIndex = 0;  // Index to track buffer position in the active buffer
volatile unsigned int bufferReadyToSend = 0; // Flag to indicate a buffer is ready to send

// Define state machine states
typedef enum {
    STATE_IDLE,
    STATE_RECEIVE_FRAME,
    STATE_PROCESS_FRAME,
    STATE_TRANSMIT_BUFFER
} SystemState;
SystemState currentState = STATE_IDLE;  // Initialize state to IDLE

// Function to configure UART for Modbus communication (19200 baud, 8N1)
void configureClocks(void) {
    CSCTL0_H = CSKEY >> 8;             // Unlock CS registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;      // Set DCO to 8MHz
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
    uartTransmitString("\nBuffer (Hex): ");
    for (i = 0; i < length; i++) {
        uartTransmitHexByte(buffer[i]);  // Send each byte in hex
        uartTransmitChar(' ');           // Add a space between bytes
    }
    uartTransmitString("\n");          // New line after buffer output
}

// Function to configure UART
void configureUART() {
    UCA1CTL1 |= UCSWRST;                    // Put eUSCI in reset
    UCA1CTL1 |= UCSSEL_2;                   // SMCLK as clock source
    UCA1BRW = 26;                           // Baud rate 19200 for 8MHz clock
    UCA1MCTLW = (0x54 << 8) | (0x00 << 4) | UCOS16; // Modulation oversampling
    PM5CTL0 &= ~LOCKLPM5;                   // Turn on I/O
    UCA1CTL1 &= ~UCSWRST;                   // Initialize eUSCI
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt
}

void UARTRxTx(void) {
    P3SEL1 &= ~BIT4;            // Clear P3SEL1 bit 4 (TXD)
    P3SEL0 |= BIT4;             // Set P3SEL0 bit 4 (TXD)
    P3SEL1 &= ~BIT5;            // Clear P3SEL1 bit 5 (RXD)
    P3SEL0 |= BIT5;             // Set P3SEL0 bit 5 (RXD)
}

// Configure RS485 transceiver in receive mode
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
    const unsigned int expectedFrameLength = bufferIndex;  // Adjust as per the actual frame length
    if (length != expectedFrameLength) return 0; // Frame is invalid due to incorrect length
    if (buffer[0] != 0x01  && buffer[0] != 0x02) return 0;  // Invalid if it doesn't start with the correct address

    // Validate CRC
    unsigned int receivedCRC = (buffer[length - 2] & 0xFF) | ((buffer[length - 1] & 0xFF) << 8);
    unsigned int calculatedCRC = calculateCRC16(buffer, length - 2);
    return (receivedCRC == calculatedCRC);  // Return 1 if valid, 0 if invalid
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

// Function to decode and process a Modbus frame
void processModbusFrame(const char* buffer, unsigned int length) {
    unsigned char slaveAddress = buffer[0];  // Decode Slave Address
    unsigned char functionCode = buffer[1];  // Decode Function Code

    uartTransmitString("Decoded Frame: ");
    uartTransmitString("\nSlave Address: ");
    uartTransmitHexByte(slaveAddress);
    uartTransmitString("\nFunction Code: ");
    uartTransmitHexByte(functionCode);

    // Decode the frame based on the function code
    switch (functionCode) {
        case 0x03:  // Read Holding Registers response
        {
            int i;
            uartTransmitString("\nFunction: Read Holding Registers\n");

            unsigned char byteCount = buffer[2];  // Number of data bytes
            uartTransmitString("Byte Count: ");
            uartTransmitHexByte(byteCount);
            uartTransmitString("\n");

            // Decode the register values from the response
            for (i = 0; i < byteCount; i += 2) {
                unsigned int registerValue = (buffer[3 + i] << 8) | buffer[4 + i];
                uartTransmitString("Register Value: ");
                uartTransmitHexByte(registerValue >> 8);  // Send high byte
                uartTransmitHexByte(registerValue & 0xFF);  // Send low byte
                uartTransmitString("\n");
            }
        }
        break;

        case 0x06:  // Write Single Register response
        {
            uartTransmitString("\nFunction: Write Single Register\n");

            unsigned int registerAddress = (buffer[2] << 8) | buffer[3];
            unsigned int registerValue = (buffer[4] << 8) | buffer[5];

            // Decoding the register address and value
            uartTransmitString("Register Address: ");
            uartTransmitHexByte(registerAddress >> 8);
            uartTransmitHexByte(registerAddress & 0xFF);
            uartTransmitString("\n");

            uartTransmitString("Register Value: ");
            uartTransmitHexByte(registerValue >> 8);
            uartTransmitHexByte(registerValue & 0xFF);
            uartTransmitString("\n");
        }
        break;

        case 0x01:  // Read Coils response
        {
            int i,bit;
            uartTransmitString("\nFunction: Read Coils\n");

            unsigned char byteCount = buffer[2];  // Number of data bytes
            uartTransmitString("Byte Count: ");
            uartTransmitHexByte(byteCount);
            uartTransmitString("\n");

            // Decode each coil state (each bit represents one coil state)
            for (i = 0; i < byteCount; i++) {
                unsigned char coilByte = buffer[3 + i];  // Each byte contains 8 coil states
                for (bit = 0; bit < 8; bit++) {
                    unsigned char coilState = (coilByte >> bit) & 0x01;
                    uartTransmitString("Coil ");
                    uartTransmitHexByte(i * 8 + bit);  // Decode coil number
                    uartTransmitString(": ");
                    uartTransmitChar(coilState ? '1' : '0');  // Decode coil state (1 = ON, 0 = OFF)
                    uartTransmitString("\n");
                }
            }
        }
        break;

        default:
            uartTransmitString("\nUnknown function code: ");
            uartTransmitHexByte(functionCode);
            uartTransmitString("\n");
            break;
    }
}

// UART RX interrupt handler
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
    char rxChar = UCA1RXBUF;               // Read received byte
    switch (currentState) {
        case STATE_IDLE:
            activeBuffer[0] = rxChar;      // Store first byte
            bufferIndex = 1;               // Reset index
            currentState = STATE_RECEIVE_FRAME;  // Switch to receive state
            startTimer();                  // Start the timeout timer
            break;

        case STATE_RECEIVE_FRAME:
            activeBuffer[bufferIndex++] = rxChar;  // Store byte and increment index
            if (bufferIndex >= BUFFER_SIZE) {
                bufferIndex = BUFFER_SIZE - 1;     // Prevent buffer overflow
            }
            startTimer();  // Reset the timeout timer
            break;

        default:
            break;
    }
}

// Timer A0 interrupt handler (for Modbus frame timeout)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void) {
    stopTimer();  // Stop the timer
    if (currentState == STATE_RECEIVE_FRAME) {
        sendBuffer = activeBuffer;       // Set buffer for sending
        currentState = STATE_PROCESS_FRAME;  // Move to process state
    }
}

// Main function
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer
    configureClocks();         // Configure clocks
    configureUART();           // Configure UART
    configureTimer();          // Configure Timer for Modbus timeout
    configureRS485();          // Configure RS485 transceiver
    UARTRxTx();                // Set up UART Rx and Tx pins
    __enable_interrupt();      // Enable global interrupts

    while (1) {
        switch (currentState) {
            case STATE_IDLE:
                break;

            case STATE_PROCESS_FRAME:
                if (isValidFrame((const char *)sendBuffer, bufferIndex)) {
                    processModbusFrame((const char *)sendBuffer, bufferIndex);  // Process valid frame
                    currentState = STATE_TRANSMIT_BUFFER;  // Move to transmit state
                } else {
                    currentState = STATE_IDLE;  // Frame was invalid, go back to idle
                }
                break;

            case STATE_TRANSMIT_BUFFER:
                uartTransmitBufferHex((const char *)sendBuffer, bufferIndex);  // Transmit buffer as hex
                currentState = STATE_IDLE;  // Go back to idle state after transmission
                break;

            default:
                break;
        }
    }
}
