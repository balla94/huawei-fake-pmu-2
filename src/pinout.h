/**
 * @file pinout.h
 * @brief Hardware pin definitions for ATtiny3226 Huawei Micro PSU Controller
 *
 * Physical Pin Layout:
 *          ATtiny3226 (20-pin)
 *                 +----------+
 *    PA6 (ENC A) 1|*         | 20  PA7 (ENC B)
 *    PA5 (SW)    2|          | 19  GND
 *    PA4 (XDIR)  3|          | 18  PC0
 *    VDD         4|          | 17  PC1
 *    GND         5|          | 16  PC2
 *    PA0         6|          | 15  PC3
 *    PB2 (TX0)   7|          | 14  UPDI
 *    PB3 (RX0)   8|          | 13  PA3
 *    PB1 (SDA)   9|          | 12  PA1 (TX1)
 *    PB0 (SCL)  10|          | 11  PA2 (RX1)
 *                 +----------+
 */

#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// =============================================================================
// ROTARY ENCODER PINS (PORTA)
// =============================================================================
#define ENCODER_SW   PIN_PA5   // Pin 2  - Encoder push button
#define ENCODER_CLK  PIN_PA7   // Pin 20 - Encoder B (swapped for direction)
#define ENCODER_DT   PIN_PA6   // Pin 1  - Encoder A (swapped for direction)

// =============================================================================
// RS485 COMMUNICATION PINS (USART1 DEFAULT)
// =============================================================================
#define RS485_TX     PIN_PA1   // Pin 12 - UART1 TX
#define RS485_RX     PIN_PA2   // Pin 11 - UART1 RX
#define RS485_DE     PIN_PA4   // Pin 3  - Direction control (DE/RE)

// =============================================================================
// I2C PINS (PORTB - LCD)
// =============================================================================
#define I2C_SCL      PIN_PB0   // Pin 10 - I2C clock
#define I2C_SDA      PIN_PB1   // Pin 9  - I2C data
#define I2C_FREQ     400000    // 400kHz fast mode

// =============================================================================
// DEBUG UART PINS (USART0 DEFAULT)
// =============================================================================
#define DEBUG_TX     PIN_PB2   // Pin 7 - Debug TX
#define DEBUG_RX     PIN_PB3   // Pin 6 - Debug RX

// =============================================================================
// LCD CONFIGURATION
// =============================================================================
#define LCD_ADDRESS  0x27
#define LCD_COLS     20
#define LCD_ROWS     4

// =============================================================================
// PSU PROTOCOL CONSTANTS
// =============================================================================
#define PSU_COUNT           10
#define PSU_BASE_ADDRESS    0x120
#define PSU_ACK             0x07F
#define INIT_COMMAND        0x1C0
#define CMD_BYTE            0xC8
#define INIT_MAGIC_BYTE     0x01
#define PSU_NO_DATA         0x073

// Magic bytes for different packet types
#define MAGIC_VOLTAGE_CURRENT      0x024
#define MAGIC_AC_STATUS            0x023
#define MAGIC_SET_VOLTAGE_CURRENT  0x042
#define MAGIC_POWER_CONTROL        0x029

// Timing constants
#define RX_TIMEOUT_MS        5
#define POLL_INTERVAL_MS     1000
#define DISCOVERY_INTERVAL_MS 3000
#define BUTTON_DEBOUNCE_MS   50

// UART buffer sizes (must be power of 2)
#define RX_BUFFER_SIZE  64
#define TX_BUFFER_SIZE  64
#define RX_BUFFER_MASK  (RX_BUFFER_SIZE - 1)
#define TX_BUFFER_MASK  (TX_BUFFER_SIZE - 1)

#endif // PINOUT_H
