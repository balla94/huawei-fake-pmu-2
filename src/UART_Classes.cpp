#include "UART_Classes.h"

// ============================================================================
// DebugUART Implementation
// ============================================================================

void DebugUART::begin(uint32_t baud) {
    PORTB.DIRSET = PIN2_bm;
    PORTB.DIRCLR = PIN3_bm;

    uint32_t desired_baud = baud;
    uint32_t baud_value = (20000000UL * 64UL + (desired_baud * 16UL) / 2) / (desired_baud * 16UL);
    USART0.BAUD = baud_value;

    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc |
                  USART_PMODE_DISABLED_gc |
                  USART_SBMODE_1BIT_gc |
                  USART_CHSIZE_8BIT_gc;

    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    delay(1);
}

void DebugUART::write(uint8_t data) {
    while (!(USART0.STATUS & USART_DREIF_bm)) {}
    USART0.TXDATAL = data;
}

void DebugUART::print(const char* str) {
    while (*str) write(*str++);
}

void DebugUART::print(uint16_t num) {
    char buffer[6];
    itoa(num, buffer, 10);
    print(buffer);
}

void DebugUART::printHex(uint16_t num) {
    char buffer[6];
    sprintf(buffer, "0x%03X", num);
    print(buffer);
}

void DebugUART::println(const char* str) {
    print(str); write('\r'); write('\n');
}

void DebugUART::println() {
    write('\r'); write('\n');
}

uint8_t DebugUART::available() {
    return (USART0.STATUS & USART_RXCIF_bm);
}

uint8_t DebugUART::read() {
    return USART0.RXDATAL;
}

// ============================================================================
// NineBitUART Implementation - Interrupt-Driven
// ============================================================================

void NineBitUART::begin(uint32_t baud) {
    // Configure pins
    PORTA.DIRSET = PIN1_bm;  // TX as output
    PORTA.DIRCLR = PIN2_bm;  // RX as input
    PORTA.DIRSET = PIN4_bm;  // XDIR as output (hardware RS485 control)

    // Calculate and set baud rate
    uint32_t baud_value = (F_CPU * 64UL + (baud * 8UL)) / (baud * 16UL);
    USART1.BAUD = baud_value;

    // Configure 9-bit mode with hardware RS485 XDIR control
    USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc |
                  USART_PMODE_DISABLED_gc |
                  USART_SBMODE_1BIT_gc |
                  USART_CHSIZE_9BITL_gc;

    // Enable RS485 mode with hardware XDIR control (PA4)
    USART1.CTRLA = USART_RXCIE_bm | USART_RS485_ENABLE_gc;

    // Initialize buffer indices
    rxHead = rxTail = 0;
    txHead = txTail = 0;
    rxOverflow = false;
    txActive = false;

    // Clear any pending data
    while (USART1.STATUS & USART_RXCIF_bm) {
        (void)USART1.RXDATAL;
        (void)USART1.RXDATAH;
    }

    // Enable RX and TX
    USART1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

    delay(1);
}

// RS485 direction control
void NineBitUART::setTxMode(bool enable) {
    if (enable) {
        digitalWrite(RS485_DE, HIGH);
    } else {
        // Small delay to ensure last bit is transmitted
        delayMicroseconds(100);  // ~1 bit time at 9600 baud
        digitalWrite(RS485_DE, LOW);
    }
}

// Blocking write for RS485 - hardware XDIR handles direction control
bool NineBitUART::write9(uint16_t data) {
    // Wait for TX buffer to be empty
    while (!(USART1.STATUS & USART_DREIF_bm)) {}

    // Write data (hardware automatically controls XDIR)
    USART1.TXDATAL = data & 0xFF;
    USART1.TXDATAH = (data >> 8) & 0x01;

    // Wait for transmission complete
    while (!(USART1.STATUS & USART_TXCIF_bm)) {}
    USART1.STATUS |= USART_TXCIF_bm;  // Clear flag

    return true;
}

// Check if all data has been transmitted (always true with blocking TX)
bool NineBitUART::isTxComplete() {
    return true;  // TX is blocking, so always complete when write9 returns
}

// Get number of bytes available to read
uint8_t NineBitUART::available() {
    uint8_t head = rxHead;
    uint8_t tail = rxTail;

    if (head >= tail) {
        return head - tail;
    } else {
        return RX_BUFFER_SIZE - tail + head;
    }
}

// Read a 9-bit value from buffer
uint16_t NineBitUART::read9() {
    // Check if data available
    if (rxHead == rxTail) {
        return 0xFFFF;  // No data available
    }

    // Get data from buffer
    uint16_t data = rxBuffer[rxTail];
    rxTail = (rxTail + 1) & RX_BUFFER_MASK;

    return data;
}

// Read with timeout (semi-blocking, but with interrupts)
uint16_t NineBitUART::read9WithTimeout(uint16_t timeout_ms) {
    uint32_t start = millis();

    while (!available()) {
        if (millis() - start > timeout_ms) {
            return 0xFFFF;  // Timeout
        }
        // Allow interrupts to run
        asm("nop");
    }

    return read9();
}

// Peek at next value without removing from buffer
uint16_t NineBitUART::peek9() {
    if (rxHead == rxTail) {
        return 0xFFFF;  // No data available
    }
    return rxBuffer[rxTail];
}

// Clear all buffers
void NineBitUART::clearBuffer() {
    clearRxBuffer();
    clearTxBuffer();
}

// Clear RX buffer
void NineBitUART::clearRxBuffer() {
    // Disable interrupts briefly to ensure atomicity
    uint8_t oldSREG = SREG;
    cli();

    rxHead = rxTail = 0;
    rxOverflow = false;

    // Clear hardware buffer too
    while (USART1.STATUS & USART_RXCIF_bm) {
        (void)USART1.RXDATAL;
        (void)USART1.RXDATAH;
    }

    SREG = oldSREG;  // Restore interrupt state
}

// Clear TX buffer (not used with blocking TX)
void NineBitUART::clearTxBuffer() {
    // TX is blocking, nothing to clear
}

// Wait for bus idle (with interrupt-driven RX)
void NineBitUART::waitForBusIdle(uint8_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (available()) {
            read9();  // Discard data
            start = millis();  // Reset timer
        }
    }
}

// Print a string (8-bit characters, 9th bit = 0)
void NineBitUART::print(const char* str) {
    while (*str) {
        write9(*str++);
    }
}

// Print a string followed by newline
void NineBitUART::println(const char* str) {
    print(str);
    write9('\n');
}

// ============================================================================
// Interrupt Handlers - Called from ISR vectors
// ============================================================================

// RX Complete Interrupt Handler
void NineBitUART::handleRxInterrupt() {
    // Read data from hardware (must read even if buffer full to clear interrupt)
    uint8_t low_byte = USART1.RXDATAL;
    uint8_t high_byte = USART1.RXDATAH;

    // Check for errors
    if (high_byte & (USART_FERR_bm | USART_BUFOVF_bm)) {
        // Error occurred, discard data
        return;
    }

    uint16_t data = low_byte | ((high_byte & 0x01) << 8);

    // Calculate next head position
    uint8_t nextHead = (rxHead + 1) & RX_BUFFER_MASK;

    // Check for buffer overflow
    if (nextHead == rxTail) {
        rxOverflow = true;
        return;  // Buffer full, discard data
    }

    // Store data in buffer
    rxBuffer[rxHead] = data;
    rxHead = nextHead;
}

// TX interrupt handlers removed - TX is blocking for RS485 reliability

// ============================================================================
// Global Instance Definitions
// ============================================================================

DebugUART Debug;
NineBitUART UART9;

// ============================================================================
// Interrupt Service Routines (ISRs)
// ============================================================================

// USART1 RX Complete interrupt
ISR(USART1_RXC_vect) {
    UART9.handleRxInterrupt();
}

// TX interrupts not used - TX is blocking for reliable RS485 control
