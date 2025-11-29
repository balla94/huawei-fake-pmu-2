#ifndef UART_CLASSES_H
#define UART_CLASSES_H

#include <Arduino.h>

#define RS485_DE PIN_PA4
#define RESPONSE_TIMEOUT 5

/**
 * @brief Debug UART interface using USART0
 * Provides serial communication for debugging at 115200 baud
 */
class DebugUART {
public:
    void begin(uint32_t baud = 115200);
    void write(uint8_t data);
    void print(const char* str);
    void print(uint16_t num);
    void printHex(uint16_t num);
    void println(const char* str);
    void println();
    uint8_t available();
    uint8_t read();
};

/**
 * @brief 9-bit UART interface using USART1 with interrupt-driven I/O
 * Provides RS485 communication with 9-bit addressing mode
 * Uses circular buffers for non-blocking operation
 */
class NineBitUART {
private:
    // Circular buffer configuration
    static constexpr uint8_t RX_BUFFER_SIZE = 64;  // Must be power of 2
    static constexpr uint8_t TX_BUFFER_SIZE = 64;  // Must be power of 2
    static constexpr uint8_t RX_BUFFER_MASK = RX_BUFFER_SIZE - 1;
    static constexpr uint8_t TX_BUFFER_MASK = TX_BUFFER_SIZE - 1;

    // Circular buffers for 9-bit data (stored as uint16_t)
    volatile uint16_t rxBuffer[RX_BUFFER_SIZE];
    volatile uint16_t txBuffer[TX_BUFFER_SIZE];

    // Buffer indices (volatile for ISR access)
    volatile uint8_t rxHead;
    volatile uint8_t rxTail;
    volatile uint8_t txHead;
    volatile uint8_t txTail;

    // Status flags
    volatile bool rxOverflow;
    volatile bool txActive;  // True when transmitter is actively sending

    // RS485 direction control
    void setTxMode(bool enable);

public:
    void begin(uint32_t baud = 9600);

    // Non-blocking write - returns false if buffer full
    bool write9(uint16_t data);

    // Check if TX complete (all data sent)
    bool isTxComplete();

    // Non-blocking read
    uint8_t available();
    uint16_t read9();
    uint16_t read9WithTimeout(uint16_t timeout_ms = 5);
    uint16_t peek9();  // Read without removing from buffer

    // Buffer management
    void clearBuffer();
    void clearRxBuffer();
    void clearTxBuffer();
    bool isRxOverflow() { return rxOverflow; }
    void clearRxOverflow() { rxOverflow = false; }

    // Bus idle detection
    void waitForBusIdle(uint8_t ms);

    // String output methods
    void print(const char* str);
    void println(const char* str);

    // Interrupt handler (must be called from ISR)
    void handleRxInterrupt();
    // TX is blocking - no TX interrupts used
};

// Global instances
extern DebugUART Debug;
extern NineBitUART UART9;

#endif
