// lab2_skel.c 
// R. Traylor
// 9.12.08
//
// Modified by Brad Anderson
// 10.12.17
// Worked with Kyle O'Brien and Makenzie Brian
//
//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

// bits used for digit selection
#define SEL0 PB4
#define SEL1 PB5
#define SEL2 PB6
#define PWM PF3

// DEMUX to LED wiring
#define SELD1 (0x0 << SEL0)
#define SELD2 (0x1 << SEL0)
#define SELD3 (0x3 << SEL0)
#define SELD4 (0x4 << SEL0)
#define SELDD (0x2 << SEL0)
#define SELBN (0x7 << SEL0)
#define SELCL !SELBN

// Blank 7segment
#define BLNK 0xFF

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12];

//select digit array
uint8_t digitSelect[8];

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button)
{
    static uint16_t State[8] = {0};     // Static array is initialied once at compile time
    State[button] = (State[button]<<1) | !bit_is_clear(PINA, button) | 0xE000;
    if (State[button] == 0xF000) return TRUE;
    return FALSE;
} //chk_buttons
//******************************************************************************



//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum)
{
    //determine how many digits there are 
    //break up decimal sum into 4 digit-segments
    //blank out leading zero digits 
    //now move data to right place for misplaced colon position

    uint8_t i=0;    // for counter
    uint8_t ldZero = TRUE;

    segment_data[0] = sum % 10;        
    segment_data[1] = sum/10 % 10;        
    segment_data[2] = 10;         // keep colon off; dig10 is mapped to BLNK        
    segment_data[3] = sum/100 % 10;        
    segment_data[4] = sum/1000 % 10;        

    // Covert dec to BCD, ignoring colon and blanking leading zeros
    //ldZero=TRUE -> index has not yet found a non-zero digit
    for (i=4; i > 0; --i)
    {
        if (ldZero && (segment_data[i]==0))
            segment_data[i] = BLNK;
        else
        {
            if (i!=2) ldZero = FALSE;
            segment_data[i] = dec_to_7seg[segment_data[i]];
        }//if
    }//for

    segment_data[0] = dec_to_7seg[segment_data[i]];

    return;
}//segment_sum
//***********************************************************************************



//***********************************************************************************
int main(void) {

    uint16_t sum = 1023;       //number displayed to screen
    uint8_t i = 0;          //count variable

    // select pins for DEMUX in array form
    digitSelect[0] = SELD1;
    digitSelect[1] = SELD2;
    digitSelect[2] = SELBN;
    digitSelect[3] = SELD3;
    digitSelect[4] = SELD4;

    // BCD mapping
    dec_to_7seg[0] = (uint8_t) 0b11000000;
    dec_to_7seg[1] = (uint8_t) 0b11111001;
    dec_to_7seg[2] = (uint8_t) 0b10100100;
    dec_to_7seg[3] = (uint8_t) 0b10110000;
    dec_to_7seg[4] = (uint8_t) 0b10011001;
    dec_to_7seg[5] = (uint8_t) 0b10010010;
    dec_to_7seg[6] = (uint8_t) 0b10000010;
    dec_to_7seg[7] = (uint8_t) 0b11111000;
    dec_to_7seg[8] = (uint8_t) 0b10000000;
    dec_to_7seg[9] = (uint8_t) 0b10010000;
    dec_to_7seg[10] = (uint8_t) 0xFF;

    //set port bits 4-7 B as outputs
    DDRB = (1<<SEL0)|(1<<SEL1)|(1<<SEL2);
    DDRF = (1<<PWM);
    
    // pull pwm to ground for now 
    PORTF &= ~(1<<PWM);

    while(1)
    {
        //insert loop delay for debounce
        _delay_ms(2);

        //make PORTA an input port with pullups 
        DDRA = 0x00;    // 0 is input, 1 is output
        PORTA = 0xFF;   // 0 is float, 1 is pull-up

        //enable tristate buffer for pushbutton switches
        PORTB &= SELCL;
        PORTB |= SELBN;

        //now check each button and increment the count as needed
        for (i=0; i<8; i++)
        {
            if (chk_buttons(i))
                sum = sum + (1 << i);
        }//for

        //bound the count to 0 - 1023
        if (sum >1023)
            sum=1;

        //break up the disp_value to 4, BCD digits in the array: call (segsum)
        segsum(sum);

        //disable tristate buffer for pushbutton switches
        PORTB &= !0x70;
        PORTB |= 0x70;

        //make PORTA an output
        DDRA = 0xFF;

        //bound a counter (0-4) to keep track of digit to display 
        for (i=0; i<5; i++)
        {
            //send PORTB the digit to display
            PORTB &= !0x70;
            
            //update digit to display
            PORTB |= digitSelect[i];
            
            //send 7 segment code to LED segments
            PORTA = segment_data[i];
            
            //dimming/flicker correction
            _delay_ms(3);
        }//for




    }//while

    return 0;

}//main
