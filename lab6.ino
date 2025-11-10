#include <avr/io.h>        // I/O register definitions
#include <avr/interrupt.h> // Interrupt support
#include <util/delay.h>    // used for initial stability delay

volatile uint8_t digit = 0;   // current digit 
volatile uint8_t paused = 0;  // pause used for button interrupt

// g f e d c b a  = mapped to pins: PB1 PB0 PD7 PD6 PD5 PD4 PD3
// common Cathode: 1 = ON, 0 = OFF
const uint8_t patterns[10] = {
  0b00111111, // 0 → a b c d e f
  0b00000110, // 1 → b c
  0b01011011, // 2 → a b d e g
  0b01001111, // 3 → a b c d g
  0b01100110, // 4 → b c f g
  0b01101101, // 5 → a c d f g
  0b01111101, // 6 → a c d e f g
  0b00000111, // 7 → a b c
  0b01111111, // 8 → a b c d e f g
  0b01101111  // 9 → a b c d f g
};


void displayDigit(uint8_t n) {
    uint8_t val = patterns[n];

    // we clear previous segments, then set new bits
    PORTD = (PORTD & 0b00000111) | ((val & 0b00011111) << 3); // PD3–PD7
    PORTB = (PORTB & 0b11111100) | ((val & 0b01100000) >> 5); // PB0–PB1
}


  //ISR: Timer1 Compare Match A Interrupt
  //Triggered every 0.5 seconds
  //Vector name from Table 11-1: TIMER1_COMPA_vect

ISR(TIMER1_COMPA_vect) {
    if (!paused) {
        digit = (digit + 1) % 10; // count 0→9
        displayDigit(digit);
    }
    PINB |= (1 << PB5); // Toggle debug LED (Table 13-2: PB5 = Arduino LED)
}


  //ISR: External Interrupt INT0 (PD2)
  //Triggered on falling edge (button press)
  //Table 11-1: INT0_vect

ISR(INT0_vect) {
    paused = !paused;
    PINB |= (1 << PB5); // Debug toggle LED
}

void setup() {


    DDRD |= (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); // Segments a–e
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB5);                           // Segments f,g + LED

    DDRD &= ~(1 << PD2);   // set PD2 as input
    PORTD |= (1 << PD2);   // enable pull-up (Table 13-3: writing 1 to PORTD enables pull-up)

    // Enable INT0 on falling edge → Table 12-1: External Interrupt Sense Control
    EICRA |= (1 << ISC01); // ISC01=1, ISC00=0 → falling edge
    EIMSK |= (1 << INT0);  // Enable INT0 interrupt


    // Table 15-5: CTC Mode → WGM12 = 1
    TCCR1A = 0;        //  clear COM1 bits here
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);

    // 16MHz / 256 prescaler → Table 15-6: CS12 = 1
    TCCR1B |= (1 << CS12);

    // OCR1A = (16,000,000 / (2 * 256)) - 1 = 31249  → gives 0.5s toggle
    OCR1A = 31249;

    // Enable Output Compare Match A Interrupt
    TIMSK1 |= (1 << OCIE1A);

 
    sei(); 

    displayDigit(0);  // show 0 immediately at startup
}

void loop() {
  
}
