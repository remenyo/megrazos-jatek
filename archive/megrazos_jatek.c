/*
 * megrazos_jatek.c
 *
 * Created: 2024.03.19. 00:00:24
 *  Author: gpt4-0125-preview
 */ 

#include <avr/io.h>
#include "lcd.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define startpin()  !(PINB&(1<<PB1))
#define endpin()  !(PINB&(1<<PB2))
#define softpwmvalmax 1000

volatile uint16_t softpwmval = 0;
volatile uint16_t best_time = 254; // Initially high to ensure any time beats it
volatile uint8_t current_time = 0;
volatile unsigned char time_changed = 0;

enum State { START, PLAYING, FINISH } state = START;
uint8_t entering_state = 1;

void initHardware() {
    DDRD |= (1<<PD4);
    PORTD &=~ (1<<PD4);
    
    DDRB |= (1<<PB3);
    TCCR2 |= (1<<CS21);
    TIMSK |= (1<<TOIE2);
    
    OCR1A = 46875;
    TCCR1A = 1 << WGM12;
    TIMSK |= 1 << OCIE1A;
    TCCR1B = 1 << CS12;    
}

void lcd_put_uint8_t(uint8_t value) {
    char buffer[4]; // Includes space for 3 digits and null terminator
    sprintf(buffer, "%3u", value); // Formats the value into 3 spaces
    lcd_puts(buffer);
}

void displayStateStart() {
    lcd_clrscr();
    lcd_puts("Ready to play!");
    lcd_gotoxy(0,1);
    lcd_puts("Best time:    s");
    lcd_gotoxy(11,1);
    lcd_put_uint8_t(best_time);
}

void displayStatePlaying() {
    lcd_clrscr();
    lcd_puts("Best time:    s");
    lcd_gotoxy(11,0);
    lcd_put_uint8_t(best_time);
    lcd_gotoxy(0,1);  
    lcd_puts("Current:      s");
    softpwmval = 20; // Start the PWM value
    current_time = 0; // Reset current time
}

void displayStateFinish() {
    lcd_clrscr();
    lcd_puts("Grat!       s");
    lcd_gotoxy(11,0);
    lcd_put_uint8_t(current_time);
    lcd_gotoxy(0,1);
    lcd_puts("Best time:    s");
    lcd_gotoxy(11,1);
    lcd_put_uint8_t(best_time);
}

void transitionState(enum State new_state) {
    state = new_state;
    entering_state = 1;
}

void handleState() {
    switch(state) {
        case START:
            if (entering_state) {
                softpwmval = 0;
                displayStateStart();
                entering_state = 0;
            }
            if (!startpin()) transitionState(PLAYING);
            break;
        
        case PLAYING:
            if (entering_state) {
                displayStatePlaying();
                entering_state = 0;
            }
            if (time_changed) {
                softpwmval += 5;
                if (softpwmval >= softpwmvalmax) softpwmval = softpwmvalmax;
                time_changed = 0;
                cli(); // Temporarily disable interrupts
                lcd_gotoxy(11,1);
                lcd_put_uint8_t(current_time);
                sei(); // Enable interrupts again
                if (startpin()) transitionState(START);
                if (endpin()) transitionState(FINISH);
            }
            break;
        
        case FINISH:
            if (entering_state) {
                softpwmval = 0;
                if (current_time < best_time) 
                    best_time = current_time;
                displayStateFinish();
                entering_state = 0;
            }
            if (startpin()) transitionState(START);
            break;
    }
}

int main(void) {
    initHardware();
    lcd_init(LCD_DISP_ON);
    sei(); // Enable global interrupts
    
    while (1) {
        handleState();
    }
}

ISR (TIMER2_OVF_vect) {
    static uint16_t softpwmcntr = 0;
    softpwmcntr++;
    if (softpwmcntr > softpwmval) PORTB &=~ (1<<PB3);
    else PORTB |= (1<<PB3);
    if (softpwmcntr > 1024) softpwmcntr = 0;
}

ISR(TIMER1_COMPA_vect) {
    current_time++;
    time_changed = 1;
}
