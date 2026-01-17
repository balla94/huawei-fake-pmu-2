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
#include <stdarg.h>
#include "UART_Classes.h"

#include "PSU_Constants.h"

#define BUS_RX_BUFFER_LENGTH 64

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
// ENCODER ISR
// =============================================================================

void encoderISR() {
    buttonRawState = !digitalRead(ENCODER_SW);
    if (encoder) encoder->tick();
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


enum BusStates { 
    BUS_IDLE,
    READ_CYCLE_SEND_REQUEST,  // We send 0x120
    READ_CYCLE_READ_DATA, // Receive data from PSU
    READ_CYCLE_WAIT_FOR_SEND_ACK, // We have to wait a bit until we can send ACK
    READ_CYCLE_SEND_ACK, // Send ACK
    READ_CYCLE_BUS_IDLE_AFTER_ACK,
    WRITE_CYCLE_SEND_REQUEST, // We send 0x1CO
    WRITE_CYCLE_WAIT_CLEAR_TO_SEND,
    WRITE_CYCLE_WAIT_AFTER_CLEAR_TO_SEND,
    WRITE_CYCLE_WRITE_REQUEST,
    WRITE_CYCLE_WAIT_FOR_PSU_ACK
 }; 

BusStates busState = BUS_IDLE;

enum busActions { 
    ACTION_PSU_INIT,
    ACTION_GET_PSU_SERIAL,
    ACTION_GET_AC_PARAMETERS,
    ACTION_GET_OUTPUT_PARAMETERS,
    ACTION_GET_AC_STATUS,
    ACTION_SET_OUTPUT_ENABLED,
    ACTION_SET_OUTPUT_PARAMETERS
 }; 


struct PSU {
  bool online;
  uint8_t offline_ignored_poll_cycles;
  uint16_t setVoltage;
  uint16_t setCurrent;
  uint16_t actVoltage;
  uint16_t actCurrent;
  bool setOutputEnable;
  bool isOutputEnable;
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



 uint8_t rotary_test;

void forgePacket(uint8_t magic, uint8_t payloadLength, ...) {
    bus_tx_length = 0;
    
    bus_tx[bus_tx_length++] = 0x00;
    bus_tx[bus_tx_length++] = 2 + payloadLength;
    bus_tx[bus_tx_length++] = 0xC8;
    bus_tx[bus_tx_length++] = magic;
    
    va_list args;
    va_start(args, payloadLength);
    for (int i = 0; i < payloadLength; i++) {
        bus_tx[bus_tx_length++] = (uint8_t)va_arg(args, int);
    }
    va_end(args);
    
    uint8_t crc = 0;
    for (int i = 0; i < bus_tx_length; i++) {
        crc += bus_tx[i];
    }
    bus_tx[bus_tx_length++] = crc;
}

void reset_bus_rx_buffer()
{
    for(int i=0; i < BUS_RX_BUFFER_LENGTH - 1; i++)
    {
        bus_rx[i] = 0x00;
    }
    bus_rx_length = 0;

}

void reset_psu_struct()
{
    for (int i = 0; i < 10; i++) {
    psu[i].online = false;
    psu[i].busAction = ACTION_PSU_INIT;
    psu[i].setVoltage = 5200;
    psu[i].setCurrent = 100;
    psu[i].setOutputEnable = false;
    psu[i].offline_ignored_poll_cycles = 0;
  }
}

uint8_t calculateCRC(const uint8_t* buffer, uint8_t length) {
    if (length < 6) {
        return 0; // Not enough data (at least 5 header bytes + 1 CRC byte)
    }
    
    uint8_t crc = 0;
    
    // Calculate CRC over all bytes except the last one (which is the CRC itself)
    for (int i = 1; i < length - 1; i++) {
        crc += buffer[i];
    }
    
    return crc;
}

void setup() {
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
    reset_psu_struct();
}


void processAnswer(uint8_t psu_id,const uint8_t* buffer, uint8_t length)
{
        char debugMsg[80];  // Buffer for formatted strings

    sprintf(debugMsg, "Length: %d, buffer[3]: 0x%02X, buffer[4]: 0x%02X", 
            length, buffer[3], buffer[4]);
    debugPrintln(debugMsg);

    if(buffer[3] != 0xC8)
    {
        debugPrintln("Magic error");
    }


    switch(buffer[4])
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
            for(int i = 5; i < length - 1; i++)
            {
                if(buffer[i] != 0x00 && buffer[i] != 0xFF)
                {
                    if(buffer[i] == '\n' || buffer[i] == '\r')psu[psu_id].deviceInfo.concat(' ');
                    else psu[psu_id].deviceInfo.concat((char)buffer[i]);
                }
            }
            sprintf(debugMsg, "Device Info: %s", psu[psu_id].deviceInfo.c_str());
            debugPrintln(debugMsg);
            break;
        }

        
        case MAGIC_PSU_SET_VOLTAGE:
        {
            if(buffer[7] == 0x00)
                {
                    psu[psu_id].setVoltageError = true;
                    debugPrintln("VSET error");
                }
                else {
                    psu[psu_id].setVoltageError = false;
                    debugPrintln("VSET OK");
                }
            break;
        }

        
        case MAGIC_PSU_POLL_AC_STATUS: 
        {
            if(buffer[10] == 0x00)
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
            if(buffer[7] == 0x55)
                {
                    psu[psu_id].isOutputEnable = true;
                    debugPrintln("OUT ON");
                }
                else if(buffer[7] == 0xAA)
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
            for(int i = 0; i < length; i++) {
                sprintf(debugMsg, "%02X ", buffer[i]);
                debugPrint(debugMsg);
            }
            debugPrintln("");
            break;

    }

}

void printOfflineReason(const char* string)
{
    debugPrintln(string);
    if(psu[psu_id].online  == true)
    {
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(string);
            rotary_test++;
    }
}

void psu_loop()
{
    if(psu_id >=10)psu_id = 0;
    switch(busState)
    {
        case BUS_IDLE: 
            if(millis() - timer > BUS_IDLE_TIMER)
            {
                busState = READ_CYCLE_SEND_REQUEST;
                timer = millis();
            }
        break;

case READ_CYCLE_SEND_REQUEST:
    // Skip offline PSUs for PSU_IGNORE_OFFLINE_CYCLES cycles
    if(psu[psu_id].online == false && psu[psu_id].offline_ignored_poll_cycles < PSU_IGNORE_OFFLINE_CYCLES)
    {
        psu[psu_id].offline_ignored_poll_cycles = psu[psu_id].offline_ignored_poll_cycles + 1;
        psu_id = psu_id + 1;
        if(psu_id >= 10) psu_id = 0;
        break;
    }
    
    // If offline counter reached limit, reset it to try polling again
    if(psu[psu_id].online == false && psu[psu_id].offline_ignored_poll_cycles >= PSU_IGNORE_OFFLINE_CYCLES)
    {
        psu[psu_id].offline_ignored_poll_cycles = 0;
    }

    // Flush any stale data from UART buffer before sending request
    while(UART9.available()) UART9.read9();

    UART9.write9(PSU_READ_REQUEST + psu_id);
    timer = millis();
    busState = READ_CYCLE_READ_DATA;
    reset_bus_rx_buffer();
break;


        case READ_CYCLE_READ_DATA:
            // If UART buffer has data, read it to buffer and reset the timer
            if(UART9.available()>0)
            {
                timer = millis();
                bus_rx[bus_rx_length] = (uint8_t)(UART9.read9() & 0xFF);
                bus_rx_length = bus_rx_length + 1;
                psu[psu_id].offline_ignored_poll_cycles = 0;
            }
            // If no response, then mark PSU offline and timeout
            else if(millis() - timer > PSU_ANSWER_TIMEOUT)
            {
                timer = millis();
                printOfflineReason("READ_ANSWER");
                psu[psu_id].online = false;
                psu_id = psu_id + 1;
                busState = BUS_IDLE;
                debugPrint("answerTimeout");
                break;
            }
            // If we got a 'nothing to say' byte, just mark the PSU online and continue writing
            if(bus_rx_length == 1 && bus_rx[0] == PSU_NO_DATA_ACK)
            {
                busState = READ_CYCLE_BUS_IDLE_AFTER_ACK;
                timer = millis();
                psu[psu_id].online = true;
                debugPrint("noRx");
            }
            // If we have the same amount in the buffer that we expects from the header,
            // calculate CRC and process it
            if(bus_rx_length > 3 && bus_rx[2] == bus_rx_length - 4 )
            {
                if(calculateCRC(bus_rx,bus_rx_length) == bus_rx[bus_rx_length -1 ])
                {
                    debugPrint("CRC OK");
                    psu[psu_id].online = true;
                    processAnswer(psu_id,bus_rx,bus_rx_length);
                    busState = READ_CYCLE_WAIT_FOR_SEND_ACK;
                    timer = millis();
                }
                else
                {
                    debugPrint("CRC Error");
                    char out[60];
                    sprintf(out,"length: %d crc: %02X exp: %02X\n",
                        bus_rx_length,calculateCRC(bus_rx,bus_rx_length), bus_rx[bus_rx_length -1 ]);
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
            if(millis() - timer > PMU_ANSWER_DELAY)
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
            if(millis() - timer > PMU_READ_WRITE_WAIT)
            {
                busState = WRITE_CYCLE_SEND_REQUEST;
                timer = millis();
            }
        break;

        case WRITE_CYCLE_SEND_REQUEST:
            // Flush any stale data from UART buffer before sending request
            while(UART9.available()) UART9.read9();

            UART9.write9(PSU_WRITE_REQUEST + psu_id);
            timer = millis();
            busState = WRITE_CYCLE_WAIT_CLEAR_TO_SEND;
        break;
        
        case WRITE_CYCLE_WAIT_CLEAR_TO_SEND:
            if(UART9.available()>0)
            {
                uint8_t rx = (uint8_t)(UART9.read9() & 0xFF);
                if(rx == psu_id)
                {
                    debugPrintln("Got CTS");
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
            else if(millis() - timer > PSU_ANSWER_TIMEOUT)
            {
                timer = millis();
                printOfflineReason("WRITE_CTS");
                psu[psu_id].online = false;
                psu_id = psu_id + 1;
                busState = BUS_IDLE;
            }
        break;
        
        case WRITE_CYCLE_WAIT_AFTER_CLEAR_TO_SEND:
            if(millis() - timer > PMU_ANSWER_DELAY)
            {
                busState = WRITE_CYCLE_WRITE_REQUEST;
                timer = millis();
            }
        break;

        case WRITE_CYCLE_WRITE_REQUEST:
            switch(psu[psu_id].busAction)
            {
                case ACTION_PSU_INIT:
                    forgePacket(MAGIC_PSU_INIT, 2, 0xFF, 0xFF);
                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_GET_PSU_SERIAL;
                break;

                case ACTION_GET_PSU_SERIAL:
                    forgePacket(MAGIC_PSU_POLL_SERIAL, 2, 0xFF, 0xFF);
                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_GET_AC_PARAMETERS;
                break;

                case ACTION_GET_AC_PARAMETERS:
                    forgePacket(MAGIC_PSU_POLL_AC_PARAMETERS, 2, 0xFF, 0xFF);
                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_GET_OUTPUT_PARAMETERS;
                break;

                case ACTION_GET_OUTPUT_PARAMETERS:
                    forgePacket(MAGIC_PSU_POLL_OUTPUT, 2, 0xFF, 0xFF);
                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_GET_AC_STATUS;
                break;

                case ACTION_GET_AC_STATUS:
                    forgePacket(MAGIC_PSU_POLL_AC_STATUS, 2, 0xFF, 0xFF);
                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_SET_OUTPUT_ENABLED;
                break;

                case ACTION_SET_OUTPUT_ENABLED:
                    forgePacket(
                        MAGIC_PSU_SET_OUTPUT,
                         4, 0xFF, 0xFF,
                         psu[psu_id].setOutputEnable ? 0x00 : 0x01, 0x00);

                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_SET_OUTPUT_PARAMETERS;
                break;

                case ACTION_SET_OUTPUT_PARAMETERS:

                    forgePacket(
                        MAGIC_PSU_SET_VOLTAGE,
                         6, 0xFF, 0xFF,
                         (psu[psu_id].setVoltage >> 8) & 0xFF, psu[psu_id].setVoltage & 0xFF,
                         (psu[psu_id].setCurrent >> 8) & 0xFF, psu[psu_id].setCurrent & 0xFF
                        );

                    for(int i = 0; i < bus_tx_length; i++)
                    {
                    UART9.write9(bus_tx[i]);  // Use write() for data bytes, not write9()
                    }
                    psu[psu_id].busAction = ACTION_GET_AC_PARAMETERS;
                break;


            }
            timer = millis();
            busState = WRITE_CYCLE_WAIT_FOR_PSU_ACK;
        break;
        
       case WRITE_CYCLE_WAIT_FOR_PSU_ACK:
    if(millis() - timer > PSU_ANSWER_TIMEOUT)
    {
        timer = millis();
        printOfflineReason("WRITE_ACK");
        psu[psu_id].online = false;
        psu_id = psu_id + 1;
        busState = BUS_IDLE;
    }
    else if(UART9.available() > 0)
    {
        uint8_t rx = (uint8_t)(UART9.read9() & 0xFF);
        if(rx == PSU_ACK)
        {
            debugPrintln("Got ACK");
            psu[psu_id].online = true;

            // Only move to next PSU when we've completed the full cycle
            // (when we're back at ACTION_GET_AC_PARAMETERS, which means
            // we just finished ACTION_SET_OUTPUT_PARAMETERS)
            if(psu[psu_id].busAction == ACTION_GET_AC_PARAMETERS)
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

enum lcdLoopStates {
    LCD_STATE_FORMAT_ROTARY,
    LCD_STATE_FORMAT_ACTIVE_PSUS,
    LCD_STATE_FORMAT_VOLTAGE,
    LCD_STATE_FORMAT_TEMP,
    LCD_STATE_SNAKE_CHECK,
    LCD_STATE_SNAKE_WRITE_ROW,
    LCD_STATE_SETCURSOR_LINE_1,
    LCD_STATE_WRITE_LINE_1,
    LCD_STATE_SETCURSOR_LINE_2,
    LCD_STATE_WRITE_LINE_2
 };

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
    {0b00000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b00000, 0b00000}
};

uint8_t snakeFrame = 0;
uint8_t snakeRowIndex = 0;
uint8_t snakeCharData[8];
uint32_t lastSnakeUpdate = 0; 

 lcdLoopStates lcdLoopState;


void lcd_loop()
{
    static char lcdLines[2][17] = {"                ", "                "};  // 16 chars + null
    static uint8_t lcdCharIndex = 0;
    static uint32_t lastLcdUpdate = 0;
    static uint16_t maxInputVoltage = 0;
    static uint16_t maxOutputTemp = 0;
    static char activePsus = '0';

    if(busState != BUS_IDLE && busState != READ_CYCLE_BUS_IDLE_AFTER_ACK && busState != READ_CYCLE_WAIT_FOR_SEND_ACK  ) return;
    lastLcdUpdate = millis();

    switch(lcdLoopState)
    {
        case LCD_STATE_FORMAT_ROTARY:
            sprintf(lcdLines[0], "%3d             ", rotary_test);
            lcdLoopState = LCD_STATE_FORMAT_ACTIVE_PSUS;
            break;

        case LCD_STATE_FORMAT_ACTIVE_PSUS:
            activePsus = '0';
            maxInputVoltage = 0;
            maxOutputTemp = 0;
            for(int i = 0; i < 10; i++) {
                if(psu[i].online) {
                    activePsus++;
                    if(psu[i].inputVoltage > maxInputVoltage) {
                        maxInputVoltage = psu[i].inputVoltage;
                    }
                    if(psu[i].outputTemperature > maxOutputTemp) {
                        maxOutputTemp = psu[i].outputTemperature;
                    }
                }
            }
            lcdLines[0][10] = activePsus;
            lcdLoopState = LCD_STATE_FORMAT_VOLTAGE;
            break;

        case LCD_STATE_FORMAT_VOLTAGE:
            snprintf(lcdLines[1], 8, "%03d.%02dV", maxInputVoltage / 100, maxInputVoltage % 100);
            lcdLines[1][7] = ' ';  // Replace null with space to not cut off rest of string
            lcdLoopState = LCD_STATE_FORMAT_TEMP;
            break;

        case LCD_STATE_FORMAT_TEMP:
            snprintf(&lcdLines[1][8], 7, "%2d\337C  ", maxOutputTemp / 100);
            lcdLines[1][14] = ' ';  // Replace null with space
            lcdLines[1][15] = 0;    // Custom char 0 for snake
            lcdLines[1][16] = '\0';
            lcdCharIndex = 0;
            lcdLoopState = LCD_STATE_SNAKE_CHECK;
            break;

        case LCD_STATE_SNAKE_CHECK:
            // Update snake animation every 227ms (5000ms / 22 frames)
            if(millis() - lastSnakeUpdate >= 227) {
                lastSnakeUpdate = millis();
                // Load frame data from PROGMEM
                for(int i = 0; i < 8; i++) {
                    snakeCharData[i] = pgm_read_byte(&snakeFrames[snakeFrame][i]);
                }
                snakeFrame++;
                if(snakeFrame >= 22) snakeFrame = 0;
                snakeRowIndex = 0;
                lcdLoopState = LCD_STATE_SNAKE_WRITE_ROW;
            } else {
                lcdLoopState = LCD_STATE_SETCURSOR_LINE_1;
            }
            break;

        case LCD_STATE_SNAKE_WRITE_ROW:
            // Write one row of custom char at a time
            lcd.command(0x40 + snakeRowIndex);  // Set CGRAM address
            lcd.write(snakeCharData[snakeRowIndex]);
            snakeRowIndex++;
            if(snakeRowIndex >= 8) {
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
            if(lcdCharIndex >= 16) {
                lcdCharIndex = 0;
                lcdLoopState = LCD_STATE_SETCURSOR_LINE_2;
            } else {
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
            if(lcdCharIndex >= 16) {
                lcdLoopState = LCD_STATE_FORMAT_ROTARY;
            } else {
                lcdLoopState = LCD_STATE_SETCURSOR_LINE_2;
            }
            break;
    }
}

void loop() {
    psu_loop();
    lcd_loop();
    uint32_t now = millis();

    // Handle encoder rotation
    if (encoder) {
        encoder->tick();
        long newPosition = encoder->getPosition();

        if (newPosition != lastEncoderPosition) {
            if (newPosition > lastEncoderPosition) {
                rotary_test++;
            } else {
                rotary_test--;
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
                psu[0].setOutputEnable = !psu[0].setOutputEnable;
            }
        }
    } else {
        lastButtonChange = now;
    }
}
