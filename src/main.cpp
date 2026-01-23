/**
 * @file main.cpp
 * @brief Simple hardware test - sends "Hello" on debug UART and RS485
 *
 * Hardware: ATtiny3226 @ 20MHz
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "pinout.h"
#include "LiquidCrystal_I2C.h"
#include <RotaryEncoder.h>
#include <stdarg.h>
#include "UART_Classes.h"

#include "PSU_Constants.h"
#include "config.h"

#define BUS_RX_BUFFER_LENGTH 64

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
RotaryEncoder *encoder = nullptr;
long lastEncoderPosition = 0;
long encoderPosition=0;
volatile bool buttonRawState = false;
bool lastButtonState = false;
uint32_t lastButtonChange = 0;
uint32_t buttonPressStart = 0;
bool longPressHandled = false;
const uint32_t LONG_PRESS_MS = 500;

// =============================================================================
// DEBUG UART (USART0)
// =============================================================================

void debugBegin(uint32_t baud)
{
    PORTB.DIRSET = PIN2_bm;
    PORTB.DIRCLR = PIN3_bm;
    uint32_t baud_value = (20000000UL * 64UL + (baud * 16UL) / 2) / (baud * 16UL);
    USART0.BAUD = baud_value;
    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
                   USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
    delay(1);
}

void debugWrite(uint8_t data)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
    {
    }
    USART0.TXDATAL = data;
}

void debugPrint(const char *str)
{
    while (*str)
        debugWrite(*str++);
}

void debugPrintln(const char *str)
{
    debugPrint(str);
    debugWrite('\r');
    debugWrite('\n');
}

// =============================================================================
// ENCODER ISR
// =============================================================================

void encoderISR()
{
    buttonRawState = !digitalRead(ENCODER_SW);
    if (encoder)
        encoder->tick();
}

// =============================================================================
// MAIN SETUP AND LOOP
// =============================================================================

/* PSU vars*/

uint8_t psu_id = 0;
unsigned long timer = 0;
uint8_t bus_rx[64];
uint8_t bus_rx_length;
uint8_t bus_tx[64];
uint8_t bus_tx_length;

uint8_t retry_count = 0;

enum BusStates
{
    BUS_IDLE,
    READ_CYCLE_SEND_REQUEST,      // We send 0x120
    READ_CYCLE_READ_DATA,         // Receive data from PSU
    READ_CYCLE_WAIT_FOR_SEND_ACK, // We have to wait a bit until we can send ACK
    READ_CYCLE_SEND_ACK,          // Send ACK
    READ_CYCLE_BUS_IDLE_AFTER_ACK,
    WRITE_CYCLE_SEND_REQUEST, // We send 0x1CO
    WRITE_CYCLE_WAIT_CLEAR_TO_SEND,
    WRITE_CYCLE_WAIT_AFTER_CLEAR_TO_SEND,
    WRITE_CYCLE_WRITE_REQUEST,
    WRITE_CYCLE_WAIT_FOR_PSU_ACK
};

BusStates busState = BUS_IDLE;

enum busActions
{
    ACTION_PSU_INIT,
    ACTION_GET_PSU_SERIAL,
    ACTION_GET_AC_PARAMETERS,
    ACTION_GET_OUTPUT_PARAMETERS,
    ACTION_GET_AC_STATUS,
    ACTION_SET_OUTPUT_ENABLED,
    ACTION_SET_OUTPUT_PARAMETERS
};

struct PSU
{
    bool online;
    uint8_t offline_ignored_poll_cycles;
    uint16_t setVoltage;
    uint16_t setCurrent;
    uint16_t actVoltage;
    uint16_t actCurrent;
    bool setOutputEnable;
    bool isOutputEnable;
    bool wasOnBus;          // Persisted in EEPROM - true if PSU was ever seen on bus
    String deviceInfo;
    uint16_t inputVoltage;
    uint16_t inputCurrent;
    uint16_t inputFreq;
    uint16_t inputPowerVA;
    uint16_t inputPowerW;
    uint16_t inputPowerFactor;
    uint16_t inputTemperature;
    uint16_t outputTemperature;
    bool acPresent;
    bool setVoltageError;
    busActions busAction;
};

PSU psu[10];

// =============================================================================
// EEPROM PERSISTENT STORAGE
// =============================================================================

// Per-PSU settings stored in EEPROM
struct PsuEepromData
{
    uint16_t setVoltage;      // 3000-6000 (30.00V - 60.00V)
    uint16_t setCurrent;      // 100-5000 (1.00A - 50.00A)
    bool outputEnabled;       // Output enable state
    bool wasOnBus;            // True if PSU was ever seen on bus
};

// Global settings stored in EEPROM
struct EepromConfig
{
    uint16_t magic;           // Magic number to validate EEPROM
    uint8_t version;          // Config version for future compatibility
    bool groupMode;           // True = all PSUs use group voltage/current
    uint16_t groupVoltage;    // Group voltage setting (3000-6000)
    uint16_t groupCurrent;    // Group current setting (100-50000)
    bool alwaysOnBacklight;   // True = backlight always on
    PsuEepromData psuData[10]; // Per-PSU settings
    uint8_t crc;              // CRC of all preceding bytes
};

EepromConfig eepromConfig;

// Backlight timeout tracking (for On Alert mode)
#define BACKLIGHT_TIMEOUT_MS 30000
uint32_t lastEncoderActivityTime = 0;
bool backlightCurrentlyOn = true;

// Calculate CRC8 for EEPROM data
uint8_t calculateEepromCrc(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07; // CRC-8 polynomial
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Write EEPROM config to flash
void eepromSave()
{
    // Calculate CRC over all data except the CRC byte itself
    eepromConfig.crc = calculateEepromCrc((uint8_t *)&eepromConfig, sizeof(EepromConfig) - 1);

    debugPrintln("[EEPROM] Saving configuration...");

    uint8_t *ptr = (uint8_t *)&eepromConfig;
    for (size_t i = 0; i < sizeof(EepromConfig); i++)
    {
        EEPROM.write(i, ptr[i]);
    }

    char debugMsg[60];
    sprintf(debugMsg, "[EEPROM] Saved %d bytes, CRC=0x%02X", (int)sizeof(EepromConfig), eepromConfig.crc);
    debugPrintln(debugMsg);
}

// Load EEPROM config from flash
bool eepromLoad()
{
    debugPrintln("[EEPROM] Loading configuration...");

    uint8_t *ptr = (uint8_t *)&eepromConfig;
    for (size_t i = 0; i < sizeof(EepromConfig); i++)
    {
        ptr[i] = EEPROM.read(i);
    }

    char debugMsg[80];
    sprintf(debugMsg, "[EEPROM] Read %d bytes, magic=0x%04X, ver=%d",
            (int)sizeof(EepromConfig), eepromConfig.magic, eepromConfig.version);
    debugPrintln(debugMsg);

    // Validate magic number
    if (eepromConfig.magic != EEPROM_MAGIC)
    {
        sprintf(debugMsg, "[EEPROM] Invalid magic: 0x%04X (expected 0x%04X)",
                eepromConfig.magic, EEPROM_MAGIC);
        debugPrintln(debugMsg);
        return false;
    }

    // Validate CRC
    uint8_t expectedCrc = calculateEepromCrc((uint8_t *)&eepromConfig, sizeof(EepromConfig) - 1);
    if (eepromConfig.crc != expectedCrc)
    {
        sprintf(debugMsg, "[EEPROM] CRC mismatch: 0x%02X (expected 0x%02X)",
                eepromConfig.crc, expectedCrc);
        debugPrintln(debugMsg);
        return false;
    }

    debugPrintln("[EEPROM] Configuration valid");
    return true;
}

// Initialize EEPROM with default values
void eepromInitDefaults()
{
    debugPrintln("[EEPROM] Initializing with defaults...");

    eepromConfig.magic = EEPROM_MAGIC;
    eepromConfig.version = EEPROM_VERSION;
    eepromConfig.groupMode = DEFAULT_GROUP_MODE;
    eepromConfig.groupVoltage = PSU_VOLTAGE_DEFAULT;
    eepromConfig.groupCurrent = GROUP_CURRENT_DEFAULT;
    eepromConfig.alwaysOnBacklight = DEFAULT_ALWAYS_ON_BACKLIGHT;

    for (int i = 0; i < 10; i++)
    {
        eepromConfig.psuData[i].setVoltage = PSU_VOLTAGE_DEFAULT;
        eepromConfig.psuData[i].setCurrent = PSU_CURRENT_DEFAULT;
        eepromConfig.psuData[i].outputEnabled = DEFAULT_OUTPUT_ENABLED;
        eepromConfig.psuData[i].wasOnBus = DEFAULT_WAS_ON_BUS;
    }

    eepromSave();
    debugPrintln("[EEPROM] Defaults saved");
}

// Apply EEPROM config to PSU runtime data
void eepromApplyConfig()
{
    char debugMsg[80];

    if (eepromConfig.groupMode)
    {
        debugPrintln("[EEPROM] Group mode active - applying group settings");
        sprintf(debugMsg, "[EEPROM] Group: V=%d.%02dV, C=%d.%02dA",
                eepromConfig.groupVoltage / 100, eepromConfig.groupVoltage % 100,
                eepromConfig.groupCurrent / 100, eepromConfig.groupCurrent % 100);
        debugPrintln(debugMsg);

        for (int i = 0; i < 10; i++)
        {
            psu[i].setVoltage = eepromConfig.groupVoltage;
            psu[i].setCurrent = eepromConfig.groupCurrent;
            psu[i].wasOnBus = eepromConfig.psuData[i].wasOnBus;
            // In group mode, output enable is handled globally, but we still load the flag
            psu[i].setOutputEnable = eepromConfig.psuData[i].outputEnabled;
        }
    }
    else
    {
        debugPrintln("[EEPROM] Individual mode - applying per-PSU settings");

        for (int i = 0; i < 10; i++)
        {
            psu[i].setVoltage = eepromConfig.psuData[i].setVoltage;
            psu[i].setCurrent = eepromConfig.psuData[i].setCurrent;
            psu[i].setOutputEnable = eepromConfig.psuData[i].outputEnabled;
            psu[i].wasOnBus = eepromConfig.psuData[i].wasOnBus;

            sprintf(debugMsg, "[EEPROM] PSU%d: V=%d.%02dV, C=%d.%02dA, Out=%d, WasOnBus=%d",
                    i,
                    psu[i].setVoltage / 100, psu[i].setVoltage % 100,
                    psu[i].setCurrent / 100, psu[i].setCurrent % 100,
                    psu[i].setOutputEnable, psu[i].wasOnBus);
            debugPrintln(debugMsg);
        }
    }
}

// Update wasOnBus flag for a specific PSU and save to EEPROM
void eepromSetWasOnBus(uint8_t psuIndex)
{
    if (psuIndex >= 10) return;
    if (eepromConfig.psuData[psuIndex].wasOnBus) return; // Already set

    char debugMsg[50];
    sprintf(debugMsg, "[EEPROM] PSU%d first seen on bus, saving", psuIndex);
    debugPrintln(debugMsg);

    eepromConfig.psuData[psuIndex].wasOnBus = true;
    psu[psuIndex].wasOnBus = true;
    eepromSave();
}

// Software reset using watchdog
void softwareReset()
{
    debugPrintln("[SYSTEM] Rebooting...");
    delay(100); // Allow debug message to be sent

    // Use watchdog timer for reset
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8CLK_gc); // Shortest timeout
    while (1) {} // Wait for watchdog reset
}

// Helper: Get digit from value at position (0=tens, 1=ones, 2=tenths, 3=hundredths)
uint8_t getDigitFromValue(uint16_t value, uint8_t pos)
{
    switch (pos)
    {
    case 0: return (value / 1000) % 10;      // Tens
    case 1: return (value / 100) % 10;       // Ones
    case 2: return (value / 10) % 10;        // Tenths
    case 3: return value % 10;               // Hundredths
    default: return 0;
    }
}

// Helper: Set digit in value at position, respecting min/max limits
uint16_t setDigitInValue(uint16_t value, uint8_t pos, int8_t delta, uint16_t minVal, uint16_t maxVal)
{
    // Get current digit and calculate new digit
    int16_t digit = getDigitFromValue(value, pos);
    digit += delta;

    // Wrap digit 0-9
    if (digit > 9) digit = 0;
    if (digit < 0) digit = 9;

    // Calculate multiplier for position
    uint16_t multiplier = 1;
    switch (pos)
    {
    case 0: multiplier = 1000; break;
    case 1: multiplier = 100; break;
    case 2: multiplier = 10; break;
    case 3: multiplier = 1; break;
    }

    // Calculate new value
    uint16_t oldDigitValue = getDigitFromValue(value, pos) * multiplier;
    uint16_t newDigitValue = digit * multiplier;
    int32_t newValue = (int32_t)value - oldDigitValue + newDigitValue;

    // Clamp to valid range
    if (newValue < minVal) newValue = minVal;
    if (newValue > maxVal) newValue = maxVal;

    return (uint16_t)newValue;
}

uint8_t unrecoverable_errors = 0;
uint8_t retry_attempts = 0;

void forgePacket(uint8_t magic, uint8_t payloadLength, ...)
{
    bus_tx_length = 0;

    bus_tx[bus_tx_length++] = 0x00;
    bus_tx[bus_tx_length++] = 2 + payloadLength;
    bus_tx[bus_tx_length++] = 0xC8;
    bus_tx[bus_tx_length++] = magic;

    va_list args;
    va_start(args, payloadLength);
    for (int i = 0; i < payloadLength; i++)
    {
        bus_tx[bus_tx_length++] = (uint8_t)va_arg(args, int);
    }
    va_end(args);

    uint8_t crc = 0;
    for (int i = 0; i < bus_tx_length; i++)
    {
        crc += bus_tx[i];
    }
    bus_tx[bus_tx_length++] = crc;
}

void reset_bus_rx_buffer()
{
    for (int i = 0; i < BUS_RX_BUFFER_LENGTH - 1; i++)
    {
        bus_rx[i] = 0x00;
    }
    bus_rx_length = 0;
}

void reset_psu_struct()
{
    for (int i = 0; i < 10; i++)
    {
        psu[i].online = false;
        psu[i].busAction = ACTION_PSU_INIT;
        psu[i].setVoltage = PSU_VOLTAGE_DEFAULT;
        psu[i].setCurrent = PSU_CURRENT_DEFAULT;
        psu[i].setOutputEnable = DEFAULT_OUTPUT_ENABLED;
        psu[i].wasOnBus = DEFAULT_WAS_ON_BUS;
        psu[i].offline_ignored_poll_cycles = 0;
    }
}

uint8_t calculateCRC(const uint8_t *buffer, uint8_t length)
{
    if (length < 6)
    {
        return 0; // Not enough data (at least 5 header bytes + 1 CRC byte)
    }

    uint8_t crc = 0;

    // Calculate CRC over all bytes except the last one (which is the CRC itself)
    for (int i = 1; i < length - 1; i++)
    {
        crc += buffer[i];
    }

    return crc;
}

void setup()
{
    delay(100);

    lcd.init();
    Wire.setClock(I2C_FREQ);
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing");

    // Initialize debug UART
    debugBegin(115200);
    UART9.begin(9600);

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
    timer = millis();
    retry_count = 0;
    reset_psu_struct();

    // Initialize EEPROM configuration
    if (!eepromLoad())
    {
        // EEPROM invalid or first boot - initialize with defaults
        eepromInitDefaults();
    }
    // Apply EEPROM config to PSU runtime data
    eepromApplyConfig();

    // Initialize backlight state from EEPROM
    lastEncoderActivityTime = millis();
    backlightCurrentlyOn = true; // Already turned on above

    debugPrintln("Boot complete");
}

void processAnswer(uint8_t psu_id, const uint8_t *buffer, uint8_t length)
{
    char debugMsg[80]; // Buffer for formatted strings

    sprintf(debugMsg, "Length: %d, buffer[3]: 0x%02X, buffer[4]: 0x%02X",
            length, buffer[3], buffer[4]);
    debugPrintln(debugMsg);

    if (buffer[3] != 0xC8)
    {
        debugPrintln("Magic error");
    }

    switch (buffer[4])
    {
    case MAGIC_PSU_INIT:
    {
        psu[psu_id].busAction = ACTION_PSU_INIT;
        debugPrintln("PSU Init");
        break;
    }

    case MAGIC_PSU_POLL_OUTPUT:
    {
        psu[psu_id].actVoltage = (buffer[8] << 8) | buffer[9];
        psu[psu_id].actCurrent = (buffer[10] << 8) | buffer[11];

        sprintf(debugMsg, "Output Voltage: %d.%02d V",
                psu[psu_id].actVoltage / 100,
                psu[psu_id].actVoltage % 100);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "Output Current: %d.%02d A",
                psu[psu_id].actCurrent / 100,
                psu[psu_id].actCurrent % 100);
        debugPrintln(debugMsg);

        uint16_t power = (psu[psu_id].actVoltage / 100) * (psu[psu_id].actCurrent / 100);
        sprintf(debugMsg, "Output Power: ~%d W", power);
        debugPrintln(debugMsg);
        break;
    }

    case MAGIC_PSU_POLL_SERIAL:
    {
        psu[psu_id].deviceInfo = "";
        for (int i = 5; i < length - 1; i++)
        {
            if (buffer[i] != 0x00 && buffer[i] != 0xFF)
            {
                if (buffer[i] == '\n' || buffer[i] == '\r')
                    psu[psu_id].deviceInfo.concat(' ');
                else
                    psu[psu_id].deviceInfo.concat((char)buffer[i]);
            }
        }
        sprintf(debugMsg, "Device Info: %s", psu[psu_id].deviceInfo.c_str());
        debugPrintln(debugMsg);
        break;
    }

    case MAGIC_PSU_SET_VOLTAGE:
    {
        if (buffer[7] == 0x00)
        {
            psu[psu_id].setVoltageError = true;
            debugPrintln("VSET OK");
        }
        else
        {
            psu[psu_id].setVoltageError = false;
            debugPrintln("VSET Error");
        }
        break;
    }

    case MAGIC_PSU_POLL_AC_STATUS:
    {
        if (buffer[10] == 0x00)
        {
            psu[psu_id].acPresent = true;
            debugPrintln("AC OK");
        }
        else
        {
            psu[psu_id].acPresent = false;
            debugPrintln("AC LOST");
        }

        break;
    }

    case MAGIC_PSU_SET_OUTPUT:
    {
        if (buffer[7] == 0x55)
        {
            psu[psu_id].isOutputEnable = true;
            debugPrintln("OUT ON");
        }
        else if (buffer[7] == 0xAA)
        {
            psu[psu_id].isOutputEnable = true;
            debugPrintln("OUT OFF");
        }
        else
        {
            debugPrintln("OUT Unknown");
        }

        break;
    }

    case MAGIC_PSU_POLL_AC_PARAMETERS:
    {

        psu[psu_id].inputVoltage = (buffer[8] << 8) | buffer[9];
        psu[psu_id].inputCurrent = (buffer[10] << 8) | buffer[11];
        psu[psu_id].inputFreq = (buffer[12] << 8) | buffer[13];
        psu[psu_id].inputPowerVA = (buffer[16] << 8) | buffer[17];
        psu[psu_id].inputPowerW = (buffer[20] << 8) | buffer[21];
        psu[psu_id].inputPowerFactor = (buffer[22] << 8) | buffer[23];
        psu[psu_id].inputTemperature = (buffer[24] << 8) | buffer[25];
        psu[psu_id].outputTemperature = (buffer[26] << 8) | buffer[27];

        psu[psu_id].inputVoltage = (buffer[8] << 8) | buffer[9];
        psu[psu_id].inputCurrent = (buffer[10] << 8) | buffer[11];
        psu[psu_id].inputFreq = (buffer[12] << 8) | buffer[13];
        psu[psu_id].inputPowerVA = (buffer[16] << 8) | buffer[17];
        psu[psu_id].inputPowerW = (buffer[20] << 8) | buffer[21];
        psu[psu_id].inputPowerFactor = (buffer[22] << 8) | buffer[23];
        psu[psu_id].inputTemperature = (buffer[24] << 8) | buffer[25];
        psu[psu_id].outputTemperature = (buffer[26] << 8) | buffer[27];

        sprintf(debugMsg, "    Input Voltage: %d.%01d V",
                psu[psu_id].inputVoltage / 10,
                psu[psu_id].inputVoltage % 10);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Input Current: %d.%02d A",
                psu[psu_id].inputCurrent / 100,
                psu[psu_id].inputCurrent % 100);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Input Frequency: %d.%01d Hz",
                psu[psu_id].inputFreq / 10,
                psu[psu_id].inputFreq % 10);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Input Power (VA): %d VA",
                psu[psu_id].inputPowerVA);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Input Power (W): %d W",
                psu[psu_id].inputPowerW);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Power Factor: %d.%03d",
                psu[psu_id].inputPowerFactor / 1000,
                psu[psu_id].inputPowerFactor % 1000);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Input Temperature: %d C",
                psu[psu_id].inputTemperature);
        debugPrintln(debugMsg);

        sprintf(debugMsg, "    Output Temperature: %d C",
                psu[psu_id].outputTemperature);
        debugPrintln(debugMsg);
        break;
    }

    default:
        sprintf(debugMsg, ">>> UNKNOWN Magic: 0x%02X", buffer[4]);
        debugPrintln(debugMsg);
        debugPrint("    Raw data: ");
        for (int i = 0; i < length; i++)
        {
            sprintf(debugMsg, "%02X ", buffer[i]);
            debugPrint(debugMsg);
        }
        debugPrintln("");
        break;
    }
}

void printOfflineReason(const char *string)
{
    debugPrintln(string);
    if (psu[psu_id].online == true)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(string);
        unrecoverable_errors++;
    }
}

void psu_loop()
{
    if (psu_id >= 10)
        psu_id = 0;
    switch (busState)
    {
    case BUS_IDLE:
        if (millis() - timer > BUS_IDLE_TIMER)
        {
            busState = READ_CYCLE_SEND_REQUEST;
            timer = millis();
            retry_count = 0;
        }
        break;

    case READ_CYCLE_SEND_REQUEST:
        // Skip offline PSUs for PSU_IGNORE_OFFLINE_CYCLES cycles
        if (psu[psu_id].online == false && psu[psu_id].offline_ignored_poll_cycles < PSU_IGNORE_OFFLINE_CYCLES)
        {
            psu[psu_id].offline_ignored_poll_cycles = psu[psu_id].offline_ignored_poll_cycles + 1;
            psu_id = psu_id + 1;
            if (psu_id >= 10)
                psu_id = 0;
            break;
        }

        // If offline counter reached limit, reset it to try polling again
        if (psu[psu_id].online == false && psu[psu_id].offline_ignored_poll_cycles >= PSU_IGNORE_OFFLINE_CYCLES)
        {
            psu[psu_id].offline_ignored_poll_cycles = 0;
        }

        // Flush any stale data from UART buffer before sending request
        while (UART9.available())
            UART9.read9();

        UART9.write9(PSU_READ_REQUEST + psu_id);
        timer = millis();
        busState = READ_CYCLE_READ_DATA;
        reset_bus_rx_buffer();
        break;

    case READ_CYCLE_READ_DATA:
        // If UART buffer has data, read it to buffer and reset the timer
        if (UART9.available() > 0)
        {
            timer = millis();
            bus_rx[bus_rx_length] = (uint8_t)(UART9.read9() & 0xFF);
            bus_rx_length = bus_rx_length + 1;
            psu[psu_id].offline_ignored_poll_cycles = 0;
        }
        // If no response, then mark PSU offline and timeout
        else if (millis() - timer > PSU_ANSWER_TIMEOUT)
        {
            timer = millis();
            if (retry_count >= RETRY_COUNT || psu[psu_id].online == false)
            {
                printOfflineReason("READ_ANSWER");
                psu[psu_id].online = false;
                psu_id = psu_id + 1;
                busState = BUS_IDLE;
                debugPrint("answerTimeout");
                break;
            }
            else 
            {
                retry_attempts++;
                retry_count++;
                busState = READ_CYCLE_SEND_REQUEST;
            }
        }
        // If we got a 'nothing to say' byte, just mark the PSU online and continue writing
        if (bus_rx_length == 1 && bus_rx[0] == PSU_NO_DATA_ACK)
        {
            busState = READ_CYCLE_BUS_IDLE_AFTER_ACK;
            timer = millis();
            bool wasOffline = !psu[psu_id].online;
            psu[psu_id].online = true;
            if (wasOffline) eepromSetWasOnBus(psu_id);
            debugPrint("noRx");
        }
        // If we have the same amount in the buffer that we expects from the header,
        // calculate CRC and process it
        if (bus_rx_length > 3 && bus_rx[2] == bus_rx_length - 4)
        {
            if (calculateCRC(bus_rx, bus_rx_length) == bus_rx[bus_rx_length - 1])
            {
                debugPrint("CRC OK");
                retry_count = 0;
                bool wasOffline = !psu[psu_id].online;
                psu[psu_id].online = true;
                if (wasOffline) eepromSetWasOnBus(psu_id);
                processAnswer(psu_id, bus_rx, bus_rx_length);
                busState = READ_CYCLE_WAIT_FOR_SEND_ACK;
                timer = millis();
            }
            else
            {
                debugPrint("CRC Error");
                char out[60];
                sprintf(out, "length: %d crc: %02X exp: %02X\n",
                        bus_rx_length, calculateCRC(bus_rx, bus_rx_length), bus_rx[bus_rx_length - 1]);
                debugPrint(out);
                timer = millis();
                printOfflineReason("CRC");
                psu[psu_id].online = false;
                psu_id = psu_id + 1;
                busState = BUS_IDLE;
                timer = millis();
            }
        }
        break;

    case READ_CYCLE_WAIT_FOR_SEND_ACK:
        if (millis() - timer > PMU_ANSWER_DELAY)
        {
            busState = READ_CYCLE_SEND_ACK;
            timer = millis();
        }
        break;

    case READ_CYCLE_SEND_ACK:
        UART9.write9(PSU_ACK);
        busState = READ_CYCLE_BUS_IDLE_AFTER_ACK;
        timer = millis();
        break;

    case READ_CYCLE_BUS_IDLE_AFTER_ACK:
        if (millis() - timer > PMU_READ_WRITE_WAIT)
        {
            busState = WRITE_CYCLE_SEND_REQUEST;
            timer = millis();
        }
        break;

    case WRITE_CYCLE_SEND_REQUEST:
        // Flush any stale data from UART buffer before sending request
        while (UART9.available())
            UART9.read9();

        UART9.write9(PSU_WRITE_REQUEST + psu_id);
        timer = millis();
        busState = WRITE_CYCLE_WAIT_CLEAR_TO_SEND;
        break;

    case WRITE_CYCLE_WAIT_CLEAR_TO_SEND:
        if (UART9.available() > 0)
        {
            uint8_t rx = (uint8_t)(UART9.read9() & 0xFF);
            if (rx == psu_id)
            {
                debugPrintln("Got CTS");
                retry_count = 0;
                timer = millis();
                busState = WRITE_CYCLE_WAIT_AFTER_CLEAR_TO_SEND;
            }
            else
            {
                debugPrintln("Invalid CTS");
                psu_id = psu_id + 1;
                busState = BUS_IDLE;
                timer = millis();
            }
        }
        else if (millis() - timer > PSU_ANSWER_TIMEOUT)
        {
            timer = millis();
            if (retry_count >= RETRY_COUNT)
            {
                printOfflineReason("WRITE_CTS");
                psu[psu_id].online = false;
                psu_id = psu_id + 1;
                busState = BUS_IDLE;
            }
            else
            {
                busState = WRITE_CYCLE_SEND_REQUEST;
                retry_attempts++;
                retry_count++;
            }
        }
        break;

    case WRITE_CYCLE_WAIT_AFTER_CLEAR_TO_SEND:
        if (millis() - timer > PMU_ANSWER_DELAY)
        {
            busState = WRITE_CYCLE_WRITE_REQUEST;
            timer = millis();
        }
        break;

    case WRITE_CYCLE_WRITE_REQUEST:
        switch (psu[psu_id].busAction)
        {
        case ACTION_PSU_INIT:
            forgePacket(MAGIC_PSU_INIT, 2, 0xFF, 0xFF);
            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_GET_PSU_SERIAL;
            break;

        case ACTION_GET_PSU_SERIAL:
            forgePacket(MAGIC_PSU_POLL_SERIAL, 2, 0xFF, 0xFF);
            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_GET_AC_PARAMETERS;
            break;

        case ACTION_GET_AC_PARAMETERS:
            forgePacket(MAGIC_PSU_POLL_AC_PARAMETERS, 2, 0xFF, 0xFF);
            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_GET_OUTPUT_PARAMETERS;
            break;

        case ACTION_GET_OUTPUT_PARAMETERS:
            forgePacket(MAGIC_PSU_POLL_OUTPUT, 2, 0xFF, 0xFF);
            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_GET_AC_STATUS;
            break;

        case ACTION_GET_AC_STATUS:
            forgePacket(MAGIC_PSU_POLL_AC_STATUS, 2, 0xFF, 0xFF);
            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_SET_OUTPUT_ENABLED;
            break;

        case ACTION_SET_OUTPUT_ENABLED:
            forgePacket(
                MAGIC_PSU_SET_OUTPUT,
                4, 0xFF, 0xFF,
                psu[psu_id].setOutputEnable ? 0x00 : 0x01, 0x00);

            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_SET_OUTPUT_PARAMETERS;
            break;

        case ACTION_SET_OUTPUT_PARAMETERS:

            forgePacket(
                MAGIC_PSU_SET_VOLTAGE,
                6, 0xFF, 0xFF,
                (psu[psu_id].setVoltage >> 8) & 0xFF, psu[psu_id].setVoltage & 0xFF,
                (psu[psu_id].setCurrent >> 8) & 0xFF, psu[psu_id].setCurrent & 0xFF);

            for (int i = 0; i < bus_tx_length; i++)
            {
                UART9.write9(bus_tx[i]); // Use write() for data bytes, not write9()
            }
            psu[psu_id].busAction = ACTION_GET_AC_PARAMETERS;
            break;
        }
        timer = millis();
        busState = WRITE_CYCLE_WAIT_FOR_PSU_ACK;
        break;

    case WRITE_CYCLE_WAIT_FOR_PSU_ACK:
        if (millis() - timer > PSU_ANSWER_TIMEOUT)
        {
            timer = millis();
            printOfflineReason("WRITE_ACK");
            psu[psu_id].online = false;
            psu_id = psu_id + 1;
            busState = BUS_IDLE;
        }
        else if (UART9.available() > 0)
        {
            uint8_t rx = (uint8_t)(UART9.read9() & 0xFF);
            if (rx == PSU_ACK)
            {
                debugPrintln("Got ACK");
                bool wasOffline = !psu[psu_id].online;
                psu[psu_id].online = true;
                if (wasOffline) eepromSetWasOnBus(psu_id);

                // Only move to next PSU when we've completed the full cycle
                // (when we're back at ACTION_GET_AC_PARAMETERS, which means
                // we just finished ACTION_SET_OUTPUT_PARAMETERS)
                if (psu[psu_id].busAction == ACTION_GET_AC_PARAMETERS)
                {
                    psu_id = psu_id + 1;
                }

                busState = BUS_IDLE;
                timer = millis();
            }
        }
        break;
    }
}

// Menu screens (top-level navigation)
enum MenuScreen
{
    MENU_HOMESCREEN,
    MENU_OPTIONS,
    MENU_LOADSHARING_SELECT,   // < ON OFF selection
    MENU_LOADSHARING_VGROUP,   // SET VGROUP with digit editing
    MENU_LOADSHARING_CGROUP,   // SET CGROUP with digit editing
    MENU_PARAMETERS_PSU_SELECT, // Select PSU 0-9 or <
    MENU_PARAMETERS_ONOFF,      // < ON OFF for selected PSU
    MENU_PARAMETERS_VSET,       // VSET for individual PSU
    MENU_PARAMETERS_CSET,       // CSET for individual PSU
    MENU_FORGET_PSU,            // Forget offline PSUs YES/NO
    MENU_BACKLIGHT_SELECT,      // Backlight mode selection
    MENU_MESSAGE_DISPLAY        // Temporary message display
};
MenuScreen currentScreen = MENU_HOMESCREEN;

// LoadSharing menu state
enum LoadSharingSelection
{
    LS_SEL_BACK = 0,   // <
    LS_SEL_ON = 1,     // ON
    LS_SEL_OFF = 2     // OFF
};
LoadSharingSelection lsSelection = LS_SEL_ON;

// Parameters menu state
uint8_t paramSelectedPsu = 0;    // 0-9 for PSU, 10 for < (back)
enum ParamOnOffSelection
{
    PARAM_SEL_BACK = 0,   // <
    PARAM_SEL_ON = 1,     // ON
    PARAM_SEL_OFF = 2     // OFF
};
ParamOnOffSelection paramOnOff = PARAM_SEL_ON;
uint8_t editingPsu = 0;          // Which PSU we're editing (0-9)

// Message display state
uint32_t messageStartTime = 0;
const uint32_t MESSAGE_DISPLAY_MS = 2000;
MenuScreen returnScreen = MENU_OPTIONS;  // Screen to return to after message

// Value editing state
uint8_t editCursorPos = 0;     // 0-4 for digits, 5 for > button
bool editMode = false;          // true = editing digit, false = cursor mode
uint16_t editVoltage = 0;       // Temporary voltage during editing
uint16_t editCurrent = 0;       // Temporary current during editing

// Blinking state for cursor
uint32_t blinkTimer = 0;
bool blinkState = false;
const uint32_t BLINK_INTERVAL_MS = 300;

// Homescreen internal states
enum HomescreenState
{
    HOMESCREEN_FORMAT_PSU_ROW,
    HOMESCREEN_FORMAT_ACTIVE_PSUS,
    HOMESCREEN_FORMAT_WRITE_CHAR,
    HOMESCREEN_FORMAT_BOTTOM_LINE
};
HomescreenState homescreenState = HOMESCREEN_FORMAT_PSU_ROW;

// Homescreen bottom line display modes
enum HomescreenBottomLineMode
{
    HOMESCREEN_VALUES_VOLTAGES,     // Vi and Vo
    HOMESCREEN_VALUES_INPUT,        // Vi and Ci (or AC LOST)
    HOMESCREEN_VALUES_OUTPUT,       // Vo and Co (or PSU STANDBY)
    HOMESCREEN_VALUES_TEMPERATURES  // Ti and To
};
HomescreenBottomLineMode bottomLineMode = HOMESCREEN_VALUES_VOLTAGES;

// Options menu items
enum OptionsMenuItem
{
    OPTIONS_LOAD_SHARING,
    OPTIONS_PARAMETERS,
    OPTIONS_FORGET_PSU,
    OPTIONS_BACKLIGHT,
    OPTIONS_EXIT
};
OptionsMenuItem selectedOption = OPTIONS_LOAD_SHARING;

// Options menu item labels (centered in 14 chars between < and >)
const char* const optionLabels[] = {
    " LoadSharing  ",
    "  Parameters  ",
    "  Forget PSU  ",
    "   Backlight  ",
    "     EXIT     "
};

// Forget PSU menu state
enum ForgetPsuSelection
{
    FORGET_SEL_NO = 0,
    FORGET_SEL_YES = 1
};
ForgetPsuSelection forgetSelection = FORGET_SEL_NO;

// Backlight mode selection
enum BacklightSelection
{
    BACKLIGHT_ALWAYS_ON = 0,
    BACKLIGHT_ON_ALERT = 1
};
BacklightSelection backlightSelection = BACKLIGHT_ALWAYS_ON;

// LCD state machine (common for all screens)
enum lcdLoopStates
{
    LCD_STATE_PROCESS_CURRENT_SCREEN,
    LCD_STATE_SNAKE_CHECK,
    LCD_STATE_SNAKE_WRITE_ROW,
    LCD_STATE_SETCURSOR_LINE_1,
    LCD_STATE_WRITE_LINE_1,
    LCD_STATE_SETCURSOR_LINE_2,
    LCD_STATE_WRITE_LINE_2
};

uint8_t selectedPsu = 0;

// Snake animation - border positions clockwise, 5 pixel snake
// Border: 5x8 char, perimeter = 5+7+4+6 = 22 pixels (corners shared)
// Path clockwise starting top-left:
// Pos 0-4:   top row    (row 0, cols 0->4)
// Pos 5-11:  right col  (col 4, rows 1->7)
// Pos 12-15: bottom row (row 7, cols 3->0) - col 4 already counted
// Pos 16-21: left col   (col 0, rows 6->1) - row 7 and row 0 already counted

// Bit positions: col0=0b10000, col1=0b01000, col2=0b00100, col3=0b00010, col4=0b00001
const uint8_t snakeFrames[22][8] PROGMEM = {
    // Frame 0: snake at positions 0,21,20,19,18 (top-left corner + left col)
    {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b00000, 0b00000, 0b00000},
    // Frame 1: positions 1,0,21,20,19
    {0b11000, 0b10000, 0b10000, 0b10000, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 2: positions 2,1,0,21,20
    {0b11100, 0b10000, 0b10000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 3: positions 3,2,1,0,21
    {0b11110, 0b10000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 4: positions 4,3,2,1,0 (full top row)
    {0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 5: positions 5,4,3,2,1 (turning top-right corner)
    {0b01111, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 6: positions 6,5,4,3,2
    {0b00111, 0b00001, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 7: positions 7,6,5,4,3
    {0b00011, 0b00001, 0b00001, 0b00001, 0b00000, 0b00000, 0b00000, 0b00000},
    // Frame 8: positions 8,7,6,5,4
    {0b00001, 0b00001, 0b00001, 0b00001, 0b00001, 0b00000, 0b00000, 0b00000},
    // Frame 9: positions 9,8,7,6,5
    {0b00000, 0b00001, 0b00001, 0b00001, 0b00001, 0b00001, 0b00000, 0b00000},
    // Frame 10: positions 10,9,8,7,6
    {0b00000, 0b00000, 0b00001, 0b00001, 0b00001, 0b00001, 0b00001, 0b00000},
    // Frame 11: positions 11,10,9,8,7 (bottom-right corner, row 7 col 4)
    {0b00000, 0b00000, 0b00000, 0b00001, 0b00001, 0b00001, 0b00001, 0b00001},
    // Frame 12: positions 12,11,10,9,8 (bottom row col3)
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b00001, 0b00001, 0b00011},
    // Frame 13: positions 13,12,11,10,9 (bottom row col2)
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b00001, 0b00111},
    // Frame 14: positions 14,13,12,11,10 (bottom row col1)
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00001, 0b01111},
    // Frame 15: positions 15,14,13,12,11 (bottom row col0 = bottom-left corner)
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111},
    // Frame 16: positions 16,15,14,13,12 (left col row6 + bottom row)
    // pos16=row6col0, pos15=row7col0, pos14=row7col1, pos13=row7col2, pos12=row7col3
    // row6: 0b10000, row7: 0b10000+0b01000+0b00100+0b00010 = 0b11110
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b10000, 0b11110},
    // Frame 17: positions 17,16,15,14,13
    // pos17=row5col0, pos16=row6col0, pos15=row7col0, pos14=row7col1, pos13=row7col2
    // row5: 0b10000, row6: 0b10000, row7: 0b10000+0b01000+0b00100 = 0b11100
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b10000, 0b10000, 0b11100},
    // Frame 18: positions 18,17,16,15,14
    // pos18=row4col0, pos17=row5col0, pos16=row6col0, pos15=row7col0, pos14=row7col1
    // row4-6: 0b10000 each, row7: 0b10000+0b01000 = 0b11000
    {0b00000, 0b00000, 0b00000, 0b00000, 0b10000, 0b10000, 0b10000, 0b11000},
    // Frame 19: positions 19,18,17,16,15
    // pos19=row3col0, pos18=row4col0, pos17=row5col0, pos16=row6col0, pos15=row7col0
    {0b00000, 0b00000, 0b00000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000},
    // Frame 20: positions 20,19,18,17,16
    // pos20=row2col0, pos19=row3col0, pos18=row4col0, pos17=row5col0, pos16=row6col0
    {0b00000, 0b00000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b00000},
    // Frame 21: positions 21,20,19,18,17
    // pos21=row1col0, pos20=row2col0, pos19=row3col0, pos18=row4col0, pos17=row5col0
    {0b00000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b00000, 0b00000}};

// Normal small digits (1 pixel padding, centered)
const uint8_t normalDigits[10][8] PROGMEM = {
    {0b00000, 0b01110, 0b01010, 0b01010, 0b01010, 0b01110, 0b00000, 0b00000}, // 0
    {0b00000, 0b00100, 0b01100, 0b00100, 0b00100, 0b01110, 0b00000, 0b00000}, // 1
    {0b00000, 0b01110, 0b00010, 0b01110, 0b01000, 0b01110, 0b00000, 0b00000}, // 2
    {0b00000, 0b01110, 0b00010, 0b01110, 0b00010, 0b01110, 0b00000, 0b00000}, // 3
    {0b00000, 0b01010, 0b01010, 0b01110, 0b00010, 0b00010, 0b00000, 0b00000}, // 4
    {0b00000, 0b01110, 0b01000, 0b01110, 0b00010, 0b01110, 0b00000, 0b00000}, // 5
    {0b00000, 0b01110, 0b01000, 0b01110, 0b01010, 0b01110, 0b00000, 0b00000}, // 6
    {0b00000, 0b01110, 0b00010, 0b00010, 0b00100, 0b00100, 0b00000, 0b00000}, // 7
    {0b00000, 0b01110, 0b01010, 0b01110, 0b01010, 0b01110, 0b00000, 0b00000}, // 8
    {0b00000, 0b01110, 0b01010, 0b01110, 0b00010, 0b01110, 0b00000, 0b00000}  // 9
};

// Inverted small digits (filled background with digit cut out)
const uint8_t invertedDigits[10][8] PROGMEM = {
    {0b11111, 0b10001, 0b10101, 0b10101, 0b10101, 0b10001, 0b11111, 0b11111}, // 0
    {0b11111, 0b11011, 0b10011, 0b11011, 0b11011, 0b10001, 0b11111, 0b11111}, // 1
    {0b11111, 0b10001, 0b11101, 0b10001, 0b10111, 0b10001, 0b11111, 0b11111}, // 2
    {0b11111, 0b10001, 0b11101, 0b10001, 0b11101, 0b10001, 0b11111, 0b11111}, // 3
    {0b11111, 0b10101, 0b10101, 0b10001, 0b11101, 0b11101, 0b11111, 0b11111}, // 4
    {0b11111, 0b10001, 0b10111, 0b10001, 0b11101, 0b10001, 0b11111, 0b11111}, // 5
    {0b11111, 0b10001, 0b10111, 0b10001, 0b10101, 0b10001, 0b11111, 0b11111}, // 6
    {0b11111, 0b10001, 0b11101, 0b11101, 0b11011, 0b11011, 0b11111, 0b11111}, // 7
    {0b11111, 0b10001, 0b10101, 0b10001, 0b10101, 0b10001, 0b11111, 0b11111}, // 8
    {0b11111, 0b10001, 0b10101, 0b10001, 0b11101, 0b10001, 0b11111, 0b11111}  // 9
};

// Custom characters for bottom line labels (main letter + subscript)
// Vi = Voltage In
const uint8_t charVi[8] PROGMEM = {
    0b10001,
    0b10001,
    0b01010,
    0b00100,
    0b00000,
    0b10111,
    0b10101,
    0b10101
};

// Vo = Voltage Out
const uint8_t charVo[8] PROGMEM = {
    0b10001,
    0b10001,
    0b01010,
    0b00100,
    0b00000,
    0b01110,
    0b01010,
    0b01110
};

// Ci = Current In
const uint8_t charCi[8] PROGMEM = {
    0b01111,
    0b10000,
    0b10000,
    0b01111,
    0b00000,
    0b10111,
    0b10101,
    0b10101
};

// Co = Current Out
const uint8_t charCo[8] PROGMEM = {
    0b01111,
    0b10000,
    0b10000,
    0b01111,
    0b00000,
    0b01110,
    0b01010,
    0b01110
};

// Ti = Temperature In
const uint8_t charTi[8] PROGMEM = {
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00000,
    0b10111,
    0b10101,
    0b10101
};

// To = Temperature Out
const uint8_t charTo[8] PROGMEM = {
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00000,
    0b01110,
    0b01010,
    0b01110
};

uint8_t snakeFrame = 0;
uint8_t snakeRowIndex = 0;
uint8_t snakeCharData[8];
uint32_t lastSnakeUpdate = 0;

lcdLoopStates lcdLoopState = LCD_STATE_PROCESS_CURRENT_SCREEN;

// Inverted digit custom char data (loaded dynamically)
uint8_t invertedCharData[8];
uint8_t invertedCharRowIndex = 0;

void lcd_loop()
{
    static char lcdLines[2][17] = {"                ", "                "}; // 16 chars + null
    static uint8_t lcdCharIndex = 0;
    static char activePsus = '0';

    if (busState != BUS_IDLE && busState != READ_CYCLE_BUS_IDLE_AFTER_ACK && busState != READ_CYCLE_WAIT_FOR_SEND_ACK)
        return;

    switch (lcdLoopState)
    {
    case LCD_STATE_PROCESS_CURRENT_SCREEN:
        switch (currentScreen)
        {
        case MENU_HOMESCREEN:
            switch (homescreenState)
            {
            case HOMESCREEN_FORMAT_PSU_ROW:
            {
                // Track encoder changes for menu navigation
                static long lastEncoderForMenu = 0;

                if (encoderPosition != lastEncoderForMenu)
                {
                    if (encoderPosition > lastEncoderForMenu)
                    {
                        selectedPsu = (selectedPsu + 1) % 10;
                    }
                    else
                    {
                        selectedPsu = (selectedPsu + 9) % 10; // Wrap around backwards
                    }
                    lastEncoderForMenu = encoderPosition;
                }

                // Build PSU row: positions 0-9 show PSU status
                activePsus = '0';
                for (int i = 0; i < 10; i++)
                {
                    if (i == selectedPsu)
                    {
                        lcdLines[0][i] = 1; // Custom char 1 (inverted digit)
                    }
                    else if (psu[i].online)
                    {
                        lcdLines[0][i] = '0' + i;
                        activePsus++;
                    }
                    else if (psu[i].wasOnBus)
                    {
                        lcdLines[0][i] = 'x'; // Offline but was previously on bus
                    }
                    else
                    {
                        lcdLines[0][i] = ' '; // Never seen on bus
                    }
                }
                // Count selected PSU if online (wasn't counted in loop above)
                if (psu[selectedPsu].online)
                {
                    activePsus++;
                }

                // Positions 10-13: spaces, 14: count, 15: snake
                lcdLines[0][10] = ' ';
                lcdLines[0][11] = ' ';
                lcdLines[0][12] = ' ';
                lcdLines[0][13] = ' ';
                lcdLines[0][14] = activePsus;
                lcdLines[0][15] = 0; // Custom char 0 (snake)

                homescreenState = HOMESCREEN_FORMAT_ACTIVE_PSUS;
                break;
            }

            case HOMESCREEN_FORMAT_ACTIVE_PSUS:
                // Load inverted digit pattern for selected PSU into custom char 1
                for (int i = 0; i < 8; i++)
                {
                    invertedCharData[i] = pgm_read_byte(&invertedDigits[selectedPsu][i]);
                }
                invertedCharRowIndex = 0;
                homescreenState = HOMESCREEN_FORMAT_WRITE_CHAR;
                break;

            case HOMESCREEN_FORMAT_WRITE_CHAR:
                // Write inverted char one row at a time (CGRAM address 0x48 = char 1)
                lcd.command(0x48 + invertedCharRowIndex);
                lcd.write(invertedCharData[invertedCharRowIndex]);
                invertedCharRowIndex++;
                if (invertedCharRowIndex >= 8)
                {
                    homescreenState = HOMESCREEN_FORMAT_BOTTOM_LINE;
                }
                break;

            case HOMESCREEN_FORMAT_BOTTOM_LINE:
            {
                // Write custom chars 2 and 3 for bottom line icons
                const uint8_t *char2Src;
                const uint8_t *char3Src;
                switch (bottomLineMode)
                {
                case HOMESCREEN_VALUES_VOLTAGES:
                    char2Src = charVi;
                    char3Src = charVo;
                    break;
                case HOMESCREEN_VALUES_INPUT:
                    char2Src = charVi;
                    char3Src = charCi;
                    break;
                case HOMESCREEN_VALUES_OUTPUT:
                    char2Src = charVo;
                    char3Src = charCo;
                    break;
                case HOMESCREEN_VALUES_TEMPERATURES:
                default:
                    char2Src = charTi;
                    char3Src = charTo;
                    break;
                }

                // Write char 2 to CGRAM
                lcd.command(0x50);
                for (int i = 0; i < 8; i++)
                    lcd.write(pgm_read_byte(&char2Src[i]));

                // Write char 3 to CGRAM
                lcd.command(0x58);
                for (int i = 0; i < 8; i++)
                    lcd.write(pgm_read_byte(&char3Src[i]));

                // Switch back to DDRAM
                lcd.command(0x80);

                // Format bottom line text based on mode
                uint16_t val1, val2;
                bool showCustomChars = false;
                if (!psu[selectedPsu].online)
                {
                    snprintf(lcdLines[1], 17, "OFFLINE         ");
                }
                else
                {
                    switch (bottomLineMode)
                    {
                    case HOMESCREEN_VALUES_VOLTAGES:
                        val1 = psu[selectedPsu].inputVoltage;
                        val2 = psu[selectedPsu].actVoltage;
                        snprintf(lcdLines[1], 17, " %3d.%02dV  %2d.%02dV",
                                 val1 / 100, val1 % 100, val2 / 100, val2 % 100);
                        showCustomChars = true;
                        break;

                    case HOMESCREEN_VALUES_INPUT:
                        if (!psu[selectedPsu].acPresent)
                        {
                            snprintf(lcdLines[1], 17, "AC LOST         ");
                        }
                        else
                        {
                            val1 = psu[selectedPsu].inputVoltage;
                            val2 = psu[selectedPsu].inputCurrent;
                            snprintf(lcdLines[1], 17, " %3d.%02dV %2d.%02dA",
                                     val1 / 100, val1 % 100, val2 / 100, val2 % 100);
                            showCustomChars = true;
                        }
                        break;

                    case HOMESCREEN_VALUES_OUTPUT:
                        if (!psu[selectedPsu].isOutputEnable)
                        {
                            snprintf(lcdLines[1], 17, "PSU STANDBY     ");
                        }
                        else
                        {
                            val1 = psu[selectedPsu].actVoltage;
                            val2 = psu[selectedPsu].actCurrent;
                            snprintf(lcdLines[1], 17, " %2d.%02dV  %2d.%02dA",
                                     val1 / 100, val1 % 100, val2 / 100, val2 % 100);
                            showCustomChars = true;
                        }
                        break;

                    case HOMESCREEN_VALUES_TEMPERATURES:
                        val1 = psu[selectedPsu].inputTemperature;
                        val2 = psu[selectedPsu].outputTemperature;
                        snprintf(lcdLines[1], 17, " %2d.%02d\337C %2d.%02d\337C",
                                 val1 / 100, val1 % 100, val2 / 100, val2 % 100);
                        showCustomChars = true;
                        break;
                    }
                }
                // Insert custom char references only when showing values
                if (showCustomChars)
                {
                    lcdLines[1][0] = 2;   // Custom char 2
                    lcdLines[1][8] = 3;   // Custom char 3
                    if (lcdLines[1][15] == '\0')
                        lcdLines[1][15] = ' '; // Only replace null to avoid snake char
                }

                lcdCharIndex = 0;
                homescreenState = HOMESCREEN_FORMAT_PSU_ROW;
                lcdLoopState = LCD_STATE_SNAKE_CHECK;
                break;
            }
            }
            break;

        case MENU_OPTIONS:
            // Top line: "    OPTIONS    " with snake at position 15
            snprintf(lcdLines[0], 16, "    OPTIONS    ");
            lcdLines[0][15] = 0; // Snake char
            // Bottom line: "< option name >"
            lcdLines[1][0] = '<';
            memcpy(&lcdLines[1][1], optionLabels[selectedOption], 14);
            lcdLines[1][15] = '>';
            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;

        case MENU_LOADSHARING_SELECT:
        {
            // Top line: "  LoadSharing  " with snake at position 15
            snprintf(lcdLines[0], 16, "  LoadSharing  ");
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "< ON OFF" with brackets around selected item
            // Format: "[<] ON  OFF " or " <  [ON] OFF " or " <  ON  [OFF]"
            if (lsSelection == LS_SEL_BACK)
                snprintf(lcdLines[1], 17, "[<]  ON   OFF   ");
            else if (lsSelection == LS_SEL_ON)
                snprintf(lcdLines[1], 17, " <  [ON]  OFF   ");
            else
                snprintf(lcdLines[1], 17, " <   ON  [OFF]  ");

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_LOADSHARING_VGROUP:
        {
            // Update blink state
            if (millis() - blinkTimer >= BLINK_INTERVAL_MS)
            {
                blinkTimer = millis();
                blinkState = !blinkState;
            }

            // Top line: " SET VGROUP   " with snake at position 15
            snprintf(lcdLines[0], 16, "  SET VGROUP   ");
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "VSET: XX.XXV [>]"
            // Format voltage as XX.XX
            uint8_t d0 = getDigitFromValue(editVoltage, 0);
            uint8_t d1 = getDigitFromValue(editVoltage, 1);
            uint8_t d2 = getDigitFromValue(editVoltage, 2);
            uint8_t d3 = getDigitFromValue(editVoltage, 3);

            // Build the line with cursor indication
            snprintf(lcdLines[1], 17, "VSET: %d%d.%d%dV   ",
                     d0, d1, d2, d3);

            // Add > button at end
            if (editCursorPos == 4)
                snprintf(&lcdLines[1][13], 4, "[>]");
            else
                snprintf(&lcdLines[1][13], 4, " > ");

            // Handle blinking for cursor/edit mode
            // Digit positions in string: 6, 7, 9, 10 (positions 0,1,2,3)
            const uint8_t digitPos[] = {6, 7, 9, 10};
            if (editCursorPos < 4)
            {
                uint8_t pos = digitPos[editCursorPos];
                if (editMode)
                {
                    // Edit mode: blink the entire character
                    if (blinkState)
                        lcdLines[1][pos] = ' ';
                }
                else
                {
                    // Cursor mode: show underline (using underscore, or use blinking to indicate)
                    if (blinkState)
                        lcdLines[1][pos] = '_';
                }
            }

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_LOADSHARING_CGROUP:
        {
            // Update blink state
            if (millis() - blinkTimer >= BLINK_INTERVAL_MS)
            {
                blinkTimer = millis();
                blinkState = !blinkState;
            }

            // Top line: " SET CGROUP   " with snake at position 15
            snprintf(lcdLines[0], 16, "  SET CGROUP   ");
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "CSET: XXX.XXA [>]"
            // For group current, we have 5 digits (100-50000 = 1.00A to 500.00A)
            // Format as XXX.XX
            uint16_t currentVal = editCurrent;
            snprintf(lcdLines[1], 17, "CSET:%3d.%02dA   ",
                     currentVal / 100, currentVal % 100);

            // Add > button at end
            if (editCursorPos == 5)
                snprintf(&lcdLines[1][13], 4, "[>]");
            else
                snprintf(&lcdLines[1][13], 4, " > ");

            // Handle blinking for cursor/edit mode
            // Digit positions in string: 5, 6, 7, 9, 10 (for XXX.XX format)
            const uint8_t digitPos[] = {5, 6, 7, 9, 10};
            if (editCursorPos < 5)
            {
                uint8_t pos = digitPos[editCursorPos];
                uint8_t digit = 0;
                switch (editCursorPos)
                {
                case 0: digit = (currentVal / 10000) % 10; break;
                case 1: digit = (currentVal / 1000) % 10; break;
                case 2: digit = (currentVal / 100) % 10; break;
                case 3: digit = (currentVal / 10) % 10; break;
                case 4: digit = currentVal % 10; break;
                }
                (void)digit; // Suppress unused warning

                if (editMode)
                {
                    // Edit mode: blink the entire character
                    if (blinkState)
                        lcdLines[1][pos] = ' ';
                }
                else
                {
                    // Cursor mode: show underline
                    if (blinkState)
                        lcdLines[1][pos] = '_';
                }
            }

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_PARAMETERS_PSU_SELECT:
        {
            // Top line: "  Parameters   " with snake at position 15
            snprintf(lcdLines[0], 16, "  Parameters   ");
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "Select PSU: X" where X is 0-9 or <
            if (paramSelectedPsu < 10)
            {
                snprintf(lcdLines[1], 17, " Select PSU:[%d] ", paramSelectedPsu);
            }
            else
            {
                snprintf(lcdLines[1], 17, " Select PSU:[<] ");
            }

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_PARAMETERS_ONOFF:
        {
            // Top line: "  SET PSU X    " with snake at position 15
            snprintf(lcdLines[0], 16, "   SET PSU %d   ", editingPsu);
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "[<] ON OFF" or "< [ON] OFF" or "< ON [OFF]"
            if (paramOnOff == PARAM_SEL_BACK)
                snprintf(lcdLines[1], 17, "[<]  ON   OFF   ");
            else if (paramOnOff == PARAM_SEL_ON)
                snprintf(lcdLines[1], 17, " <  [ON]  OFF   ");
            else
                snprintf(lcdLines[1], 17, " <   ON  [OFF]  ");

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_PARAMETERS_VSET:
        {
            // Update blink state
            if (millis() - blinkTimer >= BLINK_INTERVAL_MS)
            {
                blinkTimer = millis();
                blinkState = !blinkState;
            }

            // Top line: " VSET PSU X    " with snake at position 15
            snprintf(lcdLines[0], 16, "  VSET PSU %d   ", editingPsu);
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "VSET: XX.XXV [>]"
            uint8_t d0 = getDigitFromValue(editVoltage, 0);
            uint8_t d1 = getDigitFromValue(editVoltage, 1);
            uint8_t d2 = getDigitFromValue(editVoltage, 2);
            uint8_t d3 = getDigitFromValue(editVoltage, 3);

            snprintf(lcdLines[1], 17, "VSET: %d%d.%d%dV   ",
                     d0, d1, d2, d3);

            // Add > button at end
            if (editCursorPos == 4)
                snprintf(&lcdLines[1][13], 4, "[>]");
            else
                snprintf(&lcdLines[1][13], 4, " > ");

            // Handle blinking for cursor/edit mode
            const uint8_t digitPos[] = {6, 7, 9, 10};
            if (editCursorPos < 4)
            {
                uint8_t pos = digitPos[editCursorPos];
                if (editMode)
                {
                    if (blinkState)
                        lcdLines[1][pos] = ' ';
                }
                else
                {
                    if (blinkState)
                        lcdLines[1][pos] = '_';
                }
            }

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_PARAMETERS_CSET:
        {
            // Update blink state
            if (millis() - blinkTimer >= BLINK_INTERVAL_MS)
            {
                blinkTimer = millis();
                blinkState = !blinkState;
            }

            // Top line: " CSET PSU X    " with snake at position 15
            snprintf(lcdLines[0], 16, "  CSET PSU %d   ", editingPsu);
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "CSET: XX.XXA [>]"
            // Individual PSU current range is 100-5000 (1.00A - 50.00A)
            snprintf(lcdLines[1], 17, "CSET: %2d.%02dA   ",
                     editCurrent / 100, editCurrent % 100);

            // Add > button at end
            if (editCursorPos == 4)
                snprintf(&lcdLines[1][13], 4, "[>]");
            else
                snprintf(&lcdLines[1][13], 4, " > ");

            // Handle blinking for cursor/edit mode
            // Digit positions: 6, 7, 9, 10 (for XX.XX format)
            const uint8_t digitPos[] = {6, 7, 9, 10};
            if (editCursorPos < 4)
            {
                uint8_t pos = digitPos[editCursorPos];
                if (editMode)
                {
                    if (blinkState)
                        lcdLines[1][pos] = ' ';
                }
                else
                {
                    if (blinkState)
                        lcdLines[1][pos] = '_';
                }
            }

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_FORGET_PSU:
        {
            // Top line: "  Forget PSU   " with snake at position 15
            snprintf(lcdLines[0], 16, "  Forget PSU   ");
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "[NO] YES" or "NO [YES]"
            if (forgetSelection == FORGET_SEL_NO)
                snprintf(lcdLines[1], 17, "    [NO]  YES   ");
            else
                snprintf(lcdLines[1], 17, "     NO  [YES]  ");

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_BACKLIGHT_SELECT:
        {
            // Top line: "   Backlight   " with snake at position 15
            snprintf(lcdLines[0], 16, "   Backlight   ");
            lcdLines[0][15] = 0; // Snake char

            // Bottom line: "[Always] OnAlert" or " Always [OnAlrt]"
            if (backlightSelection == BACKLIGHT_ALWAYS_ON)
                snprintf(lcdLines[1], 17, "[Always] OnAlert");
            else
                snprintf(lcdLines[1], 17, " Always [OnAlrt]");

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }

        case MENU_MESSAGE_DISPLAY:
        {
            // Display message and check timeout
            snprintf(lcdLines[0], 17, " NOT AVAILABLE  ");
            snprintf(lcdLines[1], 17, " IN GROUP MODE  ");

            if (millis() - messageStartTime >= MESSAGE_DISPLAY_MS)
            {
                currentScreen = returnScreen;
            }

            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;
        }
        }
        break;

    case LCD_STATE_SNAKE_CHECK:
        // Update snake animation every 227ms (5000ms / 22 frames)
        if (millis() - lastSnakeUpdate >= 227)
        {
            lastSnakeUpdate = millis();
            // Load frame data from PROGMEM
            for (int i = 0; i < 8; i++)
            {
                snakeCharData[i] = pgm_read_byte(&snakeFrames[snakeFrame][i]);
            }
            snakeFrame++;
            if (snakeFrame >= 22)
                snakeFrame = 0;
            snakeRowIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_WRITE_ROW;
        }
        else
        {
            lcdLoopState = LCD_STATE_SETCURSOR_LINE_1;
        }
        break;

    case LCD_STATE_SNAKE_WRITE_ROW:
        // Write one row of custom char at a time
        lcd.command(0x40 + snakeRowIndex); // Set CGRAM address for char 0
        lcd.write(snakeCharData[snakeRowIndex]);
        snakeRowIndex++;
        if (snakeRowIndex >= 8)
        {
            lcdLoopState = LCD_STATE_SETCURSOR_LINE_1;
        }
        break;

    case LCD_STATE_SETCURSOR_LINE_1:
        lcd.setCursor(lcdCharIndex, 0);
        lcdLoopState = LCD_STATE_WRITE_LINE_1;
        break;

    case LCD_STATE_WRITE_LINE_1:
        lcd.write(lcdLines[0][lcdCharIndex]);
        lcdCharIndex++;
        if (lcdCharIndex >= 16)
        {
            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SETCURSOR_LINE_2;
        }
        else
        {
            lcdLoopState = LCD_STATE_SETCURSOR_LINE_1;
        }
        break;

    case LCD_STATE_SETCURSOR_LINE_2:
        lcd.setCursor(lcdCharIndex, 1);
        lcdLoopState = LCD_STATE_WRITE_LINE_2;
        break;

    case LCD_STATE_WRITE_LINE_2:
        lcd.write(lcdLines[1][lcdCharIndex]);
        lcdCharIndex++;
        if (lcdCharIndex >= 16)
        {
            lcdLoopState = LCD_STATE_PROCESS_CURRENT_SCREEN;
        }
        else
        {
            lcdLoopState = LCD_STATE_SETCURSOR_LINE_2;
        }
        break;
    }
}

void loop()
{
    psu_loop();
    lcd_loop();
    uint32_t now = millis();

    // Handle encoder rotation
    if (encoder)
    {
        encoder->tick();
        long newPosition = encoder->getPosition();

        if (newPosition != lastEncoderPosition)
        {
            int8_t delta = (newPosition > lastEncoderPosition) ? 1 : -1;
            lastEncoderPosition = newPosition;

            // Track encoder activity for backlight timeout
            lastEncoderActivityTime = now;

            if (currentScreen == MENU_HOMESCREEN)
            {
                // Select PSU on homescreen
                encoderPosition += delta;
            }
            else if (currentScreen == MENU_OPTIONS)
            {
                // Cycle through options menu items
                int8_t newOption = (int8_t)selectedOption + delta;
                if (newOption < 0) newOption = 4;
                if (newOption > 4) newOption = 0;
                selectedOption = (OptionsMenuItem)newOption;
            }
            else if (currentScreen == MENU_LOADSHARING_SELECT)
            {
                // Cycle through < ON OFF
                int8_t newSel = (int8_t)lsSelection + delta;
                if (newSel < 0) newSel = 2;
                if (newSel > 2) newSel = 0;
                lsSelection = (LoadSharingSelection)newSel;
            }
            else if (currentScreen == MENU_LOADSHARING_VGROUP)
            {
                if (editMode)
                {
                    // In edit mode: change the digit value
                    editVoltage = setDigitInValue(editVoltage, editCursorPos, delta,
                                                   PSU_VOLTAGE_MIN, PSU_VOLTAGE_MAX);
                }
                else
                {
                    // In cursor mode: move cursor (0-3 for digits, 4 for >)
                    int8_t newPos = (int8_t)editCursorPos + delta;
                    if (newPos < 0) newPos = 4;
                    if (newPos > 4) newPos = 0;
                    editCursorPos = newPos;
                }
            }
            else if (currentScreen == MENU_LOADSHARING_CGROUP)
            {
                if (editMode)
                {
                    // In edit mode: change the digit value
                    // For current, we have 5 digits (XXX.XX format, value 100-50000)
                    uint16_t multiplier = 1;
                    switch (editCursorPos)
                    {
                    case 0: multiplier = 10000; break;
                    case 1: multiplier = 1000; break;
                    case 2: multiplier = 100; break;
                    case 3: multiplier = 10; break;
                    case 4: multiplier = 1; break;
                    }
                    int32_t newVal = (int32_t)editCurrent + delta * multiplier;
                    // Handle digit wrapping more precisely
                    uint8_t currentDigit = (editCurrent / multiplier) % 10;
                    int8_t newDigit = currentDigit + delta;
                    if (newDigit > 9) newDigit = 0;
                    if (newDigit < 0) newDigit = 9;
                    newVal = editCurrent - (currentDigit * multiplier) + (newDigit * multiplier);
                    if (newVal < GROUP_CURRENT_MIN) newVal = GROUP_CURRENT_MIN;
                    if (newVal > GROUP_CURRENT_MAX) newVal = GROUP_CURRENT_MAX;
                    editCurrent = (uint16_t)newVal;
                }
                else
                {
                    // In cursor mode: move cursor (0-4 for digits, 5 for >)
                    int8_t newPos = (int8_t)editCursorPos + delta;
                    if (newPos < 0) newPos = 5;
                    if (newPos > 5) newPos = 0;
                    editCursorPos = newPos;
                }
            }
            else if (currentScreen == MENU_PARAMETERS_PSU_SELECT)
            {
                // Cycle through 0-9 and < (10 = back)
                int8_t newSel = (int8_t)paramSelectedPsu + delta;
                if (newSel < 0) newSel = 10;
                if (newSel > 10) newSel = 0;
                paramSelectedPsu = newSel;
            }
            else if (currentScreen == MENU_PARAMETERS_ONOFF)
            {
                // Cycle through < ON OFF
                int8_t newSel = (int8_t)paramOnOff + delta;
                if (newSel < 0) newSel = 2;
                if (newSel > 2) newSel = 0;
                paramOnOff = (ParamOnOffSelection)newSel;
            }
            else if (currentScreen == MENU_PARAMETERS_VSET)
            {
                if (editMode)
                {
                    // In edit mode: change the digit value
                    editVoltage = setDigitInValue(editVoltage, editCursorPos, delta,
                                                   PSU_VOLTAGE_MIN, PSU_VOLTAGE_MAX);
                }
                else
                {
                    // In cursor mode: move cursor (0-3 for digits, 4 for >)
                    int8_t newPos = (int8_t)editCursorPos + delta;
                    if (newPos < 0) newPos = 4;
                    if (newPos > 4) newPos = 0;
                    editCursorPos = newPos;
                }
            }
            else if (currentScreen == MENU_PARAMETERS_CSET)
            {
                if (editMode)
                {
                    // In edit mode: change the digit value (individual PSU current: 100-5000)
                    editCurrent = setDigitInValue(editCurrent, editCursorPos, delta,
                                                   PSU_CURRENT_MIN, PSU_CURRENT_MAX);
                }
                else
                {
                    // In cursor mode: move cursor (0-3 for digits, 4 for >)
                    int8_t newPos = (int8_t)editCursorPos + delta;
                    if (newPos < 0) newPos = 4;
                    if (newPos > 4) newPos = 0;
                    editCursorPos = newPos;
                }
            }
            else if (currentScreen == MENU_FORGET_PSU)
            {
                // Toggle between NO and YES
                forgetSelection = (forgetSelection == FORGET_SEL_NO) ? FORGET_SEL_YES : FORGET_SEL_NO;
            }
            else if (currentScreen == MENU_BACKLIGHT_SELECT)
            {
                // Toggle between Always On and On Alert
                backlightSelection = (backlightSelection == BACKLIGHT_ALWAYS_ON) ? BACKLIGHT_ON_ALERT : BACKLIGHT_ALWAYS_ON;
            }
        }
    }

    // Handle button press with debounce and long press detection
    bool currentButton = buttonRawState;
    if (currentButton != lastButtonState)
    {
        if (now - lastButtonChange >= BUTTON_DEBOUNCE_MS)
        {
            lastButtonState = currentButton;
            lastButtonChange = now;

            if (currentButton)
            {
                // Button just pressed - record start time
                buttonPressStart = now;
                longPressHandled = false;
                // Track activity for backlight timeout
                lastEncoderActivityTime = now;
            }
            else
            {
                // Button released - handle short press if long press wasn't triggered
                if (!longPressHandled)
                {
                    if (currentScreen == MENU_HOMESCREEN)
                    {
                        // Short press on homescreen: cycle through bottom line modes
                        bottomLineMode = static_cast<HomescreenBottomLineMode>(
                            (bottomLineMode + 1) % 4
                        );
                    }
                    else if (currentScreen == MENU_OPTIONS)
                    {
                        // Short press on options: select current option
                        if (selectedOption == OPTIONS_EXIT)
                        {
                            currentScreen = MENU_HOMESCREEN;
                            selectedOption = OPTIONS_LOAD_SHARING; // Reset for next time
                        }
                        else if (selectedOption == OPTIONS_LOAD_SHARING)
                        {
                            // Enter LoadSharing submenu
                            currentScreen = MENU_LOADSHARING_SELECT;
                            lsSelection = eepromConfig.groupMode ? LS_SEL_ON : LS_SEL_OFF;
                        }
                        else if (selectedOption == OPTIONS_PARAMETERS)
                        {
                            // Parameters only available when group mode is off
                            if (eepromConfig.groupMode)
                            {
                                // Show "NOT AVAILABLE" message
                                currentScreen = MENU_MESSAGE_DISPLAY;
                                messageStartTime = millis();
                                returnScreen = MENU_OPTIONS;
                            }
                            else
                            {
                                // Enter Parameters submenu
                                currentScreen = MENU_PARAMETERS_PSU_SELECT;
                                paramSelectedPsu = 0;
                            }
                        }
                        else if (selectedOption == OPTIONS_FORGET_PSU)
                        {
                            // Enter Forget PSU submenu
                            currentScreen = MENU_FORGET_PSU;
                            forgetSelection = FORGET_SEL_NO;
                        }
                        else if (selectedOption == OPTIONS_BACKLIGHT)
                        {
                            // Enter Backlight mode submenu
                            currentScreen = MENU_BACKLIGHT_SELECT;
                            backlightSelection = eepromConfig.alwaysOnBacklight ? BACKLIGHT_ALWAYS_ON : BACKLIGHT_ON_ALERT;
                        }
                    }
                    else if (currentScreen == MENU_LOADSHARING_SELECT)
                    {
                        if (lsSelection == LS_SEL_BACK)
                        {
                            // Go back to OPTIONS
                            currentScreen = MENU_OPTIONS;
                        }
                        else if (lsSelection == LS_SEL_ON)
                        {
                            // Enter VGROUP editing
                            currentScreen = MENU_LOADSHARING_VGROUP;
                            editVoltage = eepromConfig.groupVoltage;
                            editCurrent = eepromConfig.groupCurrent;
                            editCursorPos = 0;
                            editMode = false;
                            blinkTimer = millis();
                        }
                        else // LS_SEL_OFF
                        {
                            // Disable load sharing: set minimum values for all PSUs, disable outputs
                            debugPrintln("[MENU] Disabling load sharing...");
                            eepromConfig.groupMode = false;
                            for (int i = 0; i < 10; i++)
                            {
                                eepromConfig.psuData[i].setVoltage = PSU_VOLTAGE_MIN;
                                eepromConfig.psuData[i].setCurrent = PSU_CURRENT_MIN;
                                eepromConfig.psuData[i].outputEnabled = false;
                            }
                            eepromSave();
                            softwareReset();
                        }
                    }
                    else if (currentScreen == MENU_LOADSHARING_VGROUP)
                    {
                        if (editCursorPos == 4)
                        {
                            // > selected: go to CGROUP
                            currentScreen = MENU_LOADSHARING_CGROUP;
                            editCursorPos = 0;
                            editMode = false;
                            blinkTimer = millis();
                        }
                        else
                        {
                            // Toggle edit mode for current digit
                            editMode = !editMode;
                            blinkTimer = millis();
                        }
                    }
                    else if (currentScreen == MENU_LOADSHARING_CGROUP)
                    {
                        if (editCursorPos == 5)
                        {
                            // > selected: save and reboot
                            debugPrintln("[MENU] Saving load sharing settings...");
                            eepromConfig.groupMode = true;
                            eepromConfig.groupVoltage = editVoltage;
                            eepromConfig.groupCurrent = editCurrent;
                            // Enable all PSUs that were on the bus
                            for (int i = 0; i < 10; i++)
                            {
                                if (eepromConfig.psuData[i].wasOnBus)
                                {
                                    eepromConfig.psuData[i].outputEnabled = true;
                                }
                            }
                            eepromSave();
                            softwareReset();
                        }
                        else
                        {
                            // Toggle edit mode for current digit
                            editMode = !editMode;
                            blinkTimer = millis();
                        }
                    }
                    else if (currentScreen == MENU_PARAMETERS_PSU_SELECT)
                    {
                        if (paramSelectedPsu == 10)
                        {
                            // < selected: go back to OPTIONS
                            currentScreen = MENU_OPTIONS;
                        }
                        else
                        {
                            // PSU selected: go to ON/OFF screen
                            editingPsu = paramSelectedPsu;
                            currentScreen = MENU_PARAMETERS_ONOFF;
                            paramOnOff = psu[editingPsu].setOutputEnable ? PARAM_SEL_ON : PARAM_SEL_OFF;
                        }
                    }
                    else if (currentScreen == MENU_PARAMETERS_ONOFF)
                    {
                        if (paramOnOff == PARAM_SEL_BACK)
                        {
                            // < selected: go back to PSU selection
                            currentScreen = MENU_PARAMETERS_PSU_SELECT;
                        }
                        else if (paramOnOff == PARAM_SEL_OFF)
                        {
                            // OFF selected: disable output and save
                            debugPrintln("[MENU] Disabling PSU output...");
                            psu[editingPsu].setOutputEnable = false;
                            eepromConfig.psuData[editingPsu].outputEnabled = false;
                            eepromSave();
                            // Go back to PSU selection
                            currentScreen = MENU_PARAMETERS_PSU_SELECT;
                        }
                        else // PARAM_SEL_ON
                        {
                            // ON selected: enable output and go to VSET
                            psu[editingPsu].setOutputEnable = true;
                            eepromConfig.psuData[editingPsu].outputEnabled = true;
                            // Load current values for editing
                            editVoltage = eepromConfig.psuData[editingPsu].setVoltage;
                            editCurrent = eepromConfig.psuData[editingPsu].setCurrent;
                            editCursorPos = 0;
                            editMode = false;
                            blinkTimer = millis();
                            currentScreen = MENU_PARAMETERS_VSET;
                        }
                    }
                    else if (currentScreen == MENU_PARAMETERS_VSET)
                    {
                        if (editCursorPos == 4)
                        {
                            // > selected: go to CSET
                            currentScreen = MENU_PARAMETERS_CSET;
                            editCursorPos = 0;
                            editMode = false;
                            blinkTimer = millis();
                        }
                        else
                        {
                            // Toggle edit mode for current digit
                            editMode = !editMode;
                            blinkTimer = millis();
                        }
                    }
                    else if (currentScreen == MENU_PARAMETERS_CSET)
                    {
                        if (editCursorPos == 4)
                        {
                            // > selected: save and apply (no reboot)
                            char debugMsg[60];
                            sprintf(debugMsg, "[MENU] Saving PSU%d: V=%d, C=%d",
                                    editingPsu, editVoltage, editCurrent);
                            debugPrintln(debugMsg);

                            // Save to EEPROM
                            eepromConfig.psuData[editingPsu].setVoltage = editVoltage;
                            eepromConfig.psuData[editingPsu].setCurrent = editCurrent;
                            eepromSave();

                            // Apply to runtime
                            psu[editingPsu].setVoltage = editVoltage;
                            psu[editingPsu].setCurrent = editCurrent;

                            // Go back to PSU selection
                            currentScreen = MENU_PARAMETERS_PSU_SELECT;
                        }
                        else
                        {
                            // Toggle edit mode for current digit
                            editMode = !editMode;
                            blinkTimer = millis();
                        }
                    }
                    else if (currentScreen == MENU_FORGET_PSU)
                    {
                        if (forgetSelection == FORGET_SEL_NO)
                        {
                            // NO selected: go back to OPTIONS
                            currentScreen = MENU_OPTIONS;
                        }
                        else // FORGET_SEL_YES
                        {
                            // YES selected: forget all offline PSUs
                            debugPrintln("[MENU] Forgetting offline PSUs...");
                            for (int i = 0; i < 10; i++)
                            {
                                if (!psu[i].online && psu[i].wasOnBus)
                                {
                                    char debugMsg[40];
                                    sprintf(debugMsg, "[MENU] Forgetting PSU%d", i);
                                    debugPrintln(debugMsg);
                                    psu[i].wasOnBus = false;
                                    eepromConfig.psuData[i].wasOnBus = false;
                                }
                            }
                            eepromSave();
                            currentScreen = MENU_OPTIONS;
                        }
                    }
                    else if (currentScreen == MENU_BACKLIGHT_SELECT)
                    {
                        // Save backlight selection
                        bool newAlwaysOn = (backlightSelection == BACKLIGHT_ALWAYS_ON);
                        if (eepromConfig.alwaysOnBacklight != newAlwaysOn)
                        {
                            eepromConfig.alwaysOnBacklight = newAlwaysOn;
                            eepromSave();
                            debugPrintln(newAlwaysOn ? "[MENU] Backlight: Always On" : "[MENU] Backlight: On Alert");
                        }
                        // Reset backlight state
                        if (newAlwaysOn)
                        {
                            lcd.backlight();
                            backlightCurrentlyOn = true;
                        }
                        lastEncoderActivityTime = now;
                        currentScreen = MENU_OPTIONS;
                    }
                }
            }
        }
    }
    else
    {
        lastButtonChange = now;
    }

    // Check for long press while button is held
    if (lastButtonState && !longPressHandled)
    {
        if (now - buttonPressStart >= LONG_PRESS_MS)
        {
            longPressHandled = true;
            if (currentScreen == MENU_HOMESCREEN)
            {
                // Long press on homescreen: enter OPTIONS menu
                currentScreen = MENU_OPTIONS;
                selectedOption = OPTIONS_LOAD_SHARING;
            }
        }
    }

    // Backlight control for "On Alert" mode
    if (!eepromConfig.alwaysOnBacklight)
    {
        // Check for alert conditions: PSU online with AC lost, or offline with wasOnBus true
        bool alertActive = false;
        for (int i = 0; i < 10; i++)
        {
            if (psu[i].online && !psu[i].acPresent)
            {
                // PSU is online but AC is lost
                alertActive = true;
                break;
            }
            if (!psu[i].online && psu[i].wasOnBus)
            {
                // PSU went offline but was previously on bus
                alertActive = true;
                break;
            }
        }

        // In settings menus, always keep backlight on
        bool inSettingsMenu = (currentScreen != MENU_HOMESCREEN);

        if (alertActive || inSettingsMenu)
        {
            // Keep backlight on during alert or in settings
            if (!backlightCurrentlyOn)
            {
                lcd.backlight();
                backlightCurrentlyOn = true;
            }
        }
        else if (currentScreen == MENU_HOMESCREEN)
        {
            // On homescreen without alert: check timeout
            if (now - lastEncoderActivityTime >= BACKLIGHT_TIMEOUT_MS)
            {
                // Timeout reached, turn off backlight
                if (backlightCurrentlyOn)
                {
                    lcd.noBacklight();
                    backlightCurrentlyOn = false;
                }
            }
            else
            {
                // Within timeout, keep backlight on
                if (!backlightCurrentlyOn)
                {
                    lcd.backlight();
                    backlightCurrentlyOn = true;
                }
            }
        }
    }
    else
    {
        // Always on mode: ensure backlight is on
        if (!backlightCurrentlyOn)
        {
            lcd.backlight();
            backlightCurrentlyOn = true;
        }
    }
}
