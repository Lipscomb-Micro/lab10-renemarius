/**************************************************************************
 *      File: Lab10.c
 *  Lab Name: Bluetooth-Controlled Zumo Robot with Joystick
 *    Author: Dr. Greg Nordstrom & Prof. John Hutton (Modified by Rene Marius)
 *   Created: 12/30/17
 *  Modified: 3/28/19, 1/17/21, 11/13/2023, 12/4/2024
 * Processor: ATmega128A (on the ReadyAVR board)
 *
 * This program reads joystick input via hardware interrupts and sends
 * movement commands ('F', 'B', 'L', 'R', 'S') to the Zumo robot via
 * Bluetooth (UART0). UART1 is used for debug output to PuTTY.
 *
 * Joystick connections (from Lab 5):
 *   - UP:   PORTB.3 / PORTD.3 (INT3)
 *   - DOWN: PORTB.1 / PORTD.1 (INT1)
 *   - LEFT: PORTB.0 / PORTD.0 (INT0) - if wired
 *   - RIGHT: PORTB.2 / PORTD.2 (INT2) - if wired
 *
 * UART0 (PE0/PE1): Bluetooth to Zumo
 * UART1 (PD2/PD3): USB to PuTTY for debugging
 **************************************************************************/

#define F_CPU   7372800UL
#define END_STRING  '\0'
#define CR          0x0D
#define LF          0x0A
#define ESC         0x1B

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "UARTLibrary.h"
#include "UART0Library.h"

#define BAUDRATE   9600
#define DATABITS      8
#define STOPBITS      1
#define PARITY        0

char text[40];
volatile char command = 'S';

void send_string(const char *cp) {
    while(*cp != END_STRING) {
        uart_tx(*cp++);
    }
}

// Joystick UP (INT3) - Forward
ISR(INT3_vect) {
    if (!(PIND & (1<<PIND3))) {
	    // Button pressed (pin is LOW)
	    command = 'F';
	    } else {
	    // Button released (pin is HIGH)
	    command = 'S';
    }
}

// Joystick DOWN (INT1) - Backward
ISR(INT1_vect) {
    if (!(PIND & (1<<PIND1))) {
	    command = 'B';
	    } else {
	    command = 'S';
    }
}

// Joystick RIGHT (INT0) - Turn Right
ISR(INT0_vect) {
    if (!(PIND & (1<<PIND2))) {
	    command = 'R';
	    } else {
	    command = 'S';
    }
}

// Joystick LEFT (INT2) - Turn Left
ISR(INT2_vect) {
    if (!(PIND & (1<<PIND0))) {
	    command = 'L';
	    } else {
	    command = 'S';
    }
}

int main(void) {
    const char resetMsg[] = "\r\nJoystick Zumo Controller Ready.\r\n\0";
    char lastCommand = 'S';
    
    //  debug (PuTTY)
    uart_init(BAUDRATE, DATABITS, STOPBITS, PARITY);
    
    // Bluetooth (Zumo)
    uart0_init(BAUDRATE, DATABITS, STOPBITS, PARITY);
    
    // JOYSTICK SETUP
    DDRB = 0x00;                                    // All inputs
    PORTB = (1<<PINB0) | (1<<PINB1) | (1<<PINB2) | (1<<PINB3);  // Enable pull-ups
    
    // Configure PORTD as inputs with pull-ups (for interrupts)
    DDRD &= ~((1<<PIND0) | (1<<PIND1) | (1<<PIND2) | (1<<PIND3));  // Set as inputs
    PORTD |= (1<<PIND0) | (1<<PIND1) | (1<<PIND2) | (1<<PIND3);    // Enable pull-ups
    
    // Enable external interrupts INT0, INT1, INT2, INT3
    EIMSK = (1<<INT0) | (1<<INT1) | (1<<INT2) | (1<<INT3);
    
    // Configure interrupts for rising edge (button release)
    EICRA = (1<<ISC00) |   // logical change
		(1<<ISC10) |
		(1<<ISC20) |
		(1<<ISC30);
    
    // Enable global interrupts
    sei();
    
    send_string(resetMsg);
    
    while(1) {
        // Check if command has changed
        if (command != lastCommand) {
            // Send command to Zumo via Bluetooth
            uart0_tx(command);
            
            // Debug output to PuTTY
            sprintf(text, "Command sent: %c\r\n", command);
            send_string(text);
            
            lastCommand = command;
        }
        
        _delay_ms(50);  // Small delay
    }
    
    return 0;
}