/******************************************************************************
 *      File: Lab10_Joystick.c
 *  Lab Name: Joystick-Controlled Zumo via Bluetooth
 *    Author: Rene Marius (modified from Nordstrom/Hutton template)
 *   Created: 11/21/2025
 * Processor: ATmega128A (on the ReadyAVR board)
 *
 * This program reads a joystick connected to ADC0 (X-axis) and ADC1 (Y-axis)
 * and sends movement commands (F/B/L/R/S) to the Zumo robot via Bluetooth.
 *
 * UART1 for debug output to Putty Terminal
 * UART0 for Bluetooth communication to Zumo
 * ADC0 (PF0) for Joystick X-axis
 * ADC1 (PF1) for Joystick Y-axis
 ******************************************************************************/

#define F_CPU   7372800UL
#define END_STRING  '\0'
#define CR          0x0D
#define LF          0x0A

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "UARTLibrary.h"
#include "UART0Library.h"

#define BAUDRATE   9600
#define DATABITS      8
#define STOPBITS      1
#define PARITY        0

// Joystick thresholds (adjust based on your joystick's center point)
#define FORWARD_THRESHOLD   180
#define BACKWARD_THRESHOLD   75
#define RIGHT_THRESHOLD     180
#define LEFT_THRESHOLD       75
#define DEADZONE_CENTER     128  // Approximate center value

char text[40];

void send_string(const char *cp) {
    while(*cp != END_STRING) {
        uart_tx(*cp++);
    }
}

// Initialize ADC for joystick reading
void adc_init(void) {
    // Set reference voltage to AVCC (5V)
    ADMUX = (1 << REFS0);
    
    // Enable ADC, set prescaler to 64 (7.3728MHz/64 = 115.2kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

// Read ADC value from specified channel (0 or 1)
uint8_t adc_read(uint8_t channel) {
    // Select ADC channel (0-7)
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return 8-bit result (ADCH contains upper 8 bits)
    return ADCH;
}

// Determine command based on joystick position
char determine_command(uint8_t x_value, uint8_t y_value) {
    // Check Y-axis first (forward/backward has priority)
    if (y_value > FORWARD_THRESHOLD) {
        return 'F';  // Forward
    }
    else if (y_value < BACKWARD_THRESHOLD) {
        return 'B';  // Backward
    }
    // Check X-axis (left/right)
    else if (x_value > RIGHT_THRESHOLD) {
        return 'R';  // Right
    }
    else if (x_value < LEFT_THRESHOLD) {
        return 'L';  // Left
    }
    else {
        return 'S';  // Stop (joystick centered)
    }
}

int main(void) {
    uint8_t x_axis, y_axis;
    char command;
    char lastCommand = 'S';  // Track last command to avoid spam
    
    const char resetMsg[] = "\r\nJoystick Zumo Controller Ready.\r\n\0";
    
    // Initialize UART1 for debug (PuTTY)
    uart_init(BAUDRATE, DATABITS, STOPBITS, PARITY);
    
    // Initialize UART0 for Bluetooth
    uart0_init(BAUDRATE, DATABITS, STOPBITS, PARITY);
    
    // Initialize ADC for joystick
    adc_init();
    
    // Set ADC to left-adjust result (for 8-bit reading from ADCH)
    ADMUX |= (1 << ADLAR);
    
    send_string(resetMsg);
    
    while(1) {
        // Read joystick axes
        x_axis = adc_read(0);  // ADC0 = X-axis
        y_axis = adc_read(1);  // ADC1 = Y-axis
        
        // Determine command from joystick position
        command = determine_command(x_axis, y_axis);
        
        // Only send command if it changed (reduces BT traffic)
        if (command != lastCommand) {
            // Send command to Zumo via Bluetooth
            uart0_tx(command);
            
            // Debug output to PuTTY
            sprintf(text, "X:%3d Y:%3d CMD:%c\r\n", x_axis, y_axis, command);
            send_string(text);
            
            lastCommand = command;
        }
        
        // Small delay to prevent overwhelming the system
        _delay_ms(100);
    }
    
    return 0;
}