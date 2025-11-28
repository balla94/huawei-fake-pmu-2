/**
 * @file main.cpp
 * @brief Simple hardware test - sends "Hello" on debug UART and RS485
 *
 * Hardware: ATtiny3226 @ 20MHz
 */

#include <Arduino.h>
#include <Wire.h>
#include "pinout.h"
#include "LiquidCrystal_I2C.h"
#include <RotaryEncoder.h>

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
RotaryEncoder* encoder = nullptr;
long lastEncoderPosition = 0;
volatile bool buttonRawState = false;
bool lastButtonState = false;
uint32_t lastButtonChange = 0;

// =============================================================================
// DEBUG UART (USART0)
// =============================================================================

void debugBegin(uint32_t baud) {
    PORTB.DIRSET = PIN2_bm;
    PORTB.DIRCLR = PIN3_bm;
    uint32_t baud_value = (20000000UL * 64UL + (baud * 16UL) / 2) / (baud * 16UL);
    USART0.BAUD = baud_value;
    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
                   USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    delay(1);
}

void debugWrite(uint8_t data) {
    while (!(USART0.STATUS & USART_DREIF_bm)) {}
    USART0.TXDATAL = data;
}

void debugPrint(const char* str) {
    while (*str) debugWrite(*str++);
}

void debugPrintln(const char* str) {
    debugPrint(str);
    debugWrite('\r');
    debugWrite('\n');
}

// =============================================================================
// RS485 UART (USART1)
// =============================================================================

void rs485Begin(uint32_t baud) {
    // TX and XDIR as output, RX as input
    PORTA.DIRSET = PIN1_bm | PIN4_bm;
    PORTA.DIRCLR = PIN2_bm;

    uint32_t baud_value = (F_CPU * 64UL + (baud * 8UL)) / (baud * 16UL);
    USART1.BAUD = baud_value;

    // 9-bit mode for space parity (9th bit always 0), 1 stop bit
    USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
                   USART_SBMODE_1BIT_gc | USART_CHSIZE_9BITL_gc;

    // Enable RS485 mode with hardware XDIR control on PA4
    USART1.CTRLA = USART_RS485_ENABLE_gc;

    USART1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    delay(1);
}

void rs485Write(uint8_t data) {
    while (!(USART1.STATUS & USART_DREIF_bm)) {}
    // 9th bit = 0 (space parity)
    USART1.TXDATAH = 0;
    USART1.TXDATAL = data;
}

void rs485Println(const char* str) {
    while (*str) {
        rs485Write(*str++);
    }
    rs485Write('\r');
    rs485Write('\n');

    // Wait for transmission complete (XDIR handled by hardware)
    while (!(USART1.STATUS & USART_TXCIF_bm)) {}
    USART1.STATUS |= USART_TXCIF_bm;
}

// =============================================================================
// ENCODER ISR
// =============================================================================

void encoderISR() {
    buttonRawState = !digitalRead(ENCODER_SW);
    if (encoder) encoder->tick();
}

// =============================================================================
// MAIN SETUP AND LOOP
// =============================================================================

uint32_t lastSendTime = 0;

void setup() {
    delay(100);

    // Initialize debug UART
    debugBegin(115200);

    // Initialize RS485 (XDIR on PA4 controlled by hardware)
    rs485Begin(9600);

    // Initialize LCD
    lcd.init();
    Wire.setClock(I2C_FREQ);
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hello World");

    // Initialize rotary encoder
    encoder = new RotaryEncoder(ENCODER_DT, ENCODER_CLK, RotaryEncoder::LatchMode::FOUR3);
    lastEncoderPosition = 0;
    pinMode(ENCODER_SW, INPUT_PULLUP);

    // Configure pin interrupts for encoder
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;

    attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_SW), encoderISR, CHANGE);

    debugPrintln("Hardware initialized");
}

void loop() {
    uint32_t now = millis();

    // Send Hello every 1 second
    if (now - lastSendTime >= 1000) {
        lastSendTime = now;
        debugPrintln("Hello");
        rs485Println("Hello");
    }

    // Handle encoder rotation
    if (encoder) {
        encoder->tick();
        long newPosition = encoder->getPosition();

        if (newPosition != lastEncoderPosition) {
            if (newPosition > lastEncoderPosition) {
                debugPrintln("+");
            } else {
                debugPrintln("-");
            }
            lastEncoderPosition = newPosition;
        }
    }

    // Handle button press with debounce
    bool currentButton = buttonRawState;
    if (currentButton != lastButtonState) {
        if (now - lastButtonChange >= BUTTON_DEBOUNCE_MS) {
            lastButtonState = currentButton;
            lastButtonChange = now;

            if (currentButton) {
                debugPrintln("P");
            }
        }
    } else {
        lastButtonChange = now;
    }
}
