//  File:     main.c 
//  Author:   Bradley Anderson
//  Created:  Dec2, 2017
//  
//  Collaboration:
//  None


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "hd44780.h"
#include "button7segFunctions.h"
#include "encoderFunctions.h"


//enum states {DISP_TIME, SET_TIME, ALARM, SNOOZE, SET_ALARM};
enum states {DISP_TIME, SET_TIME, ALARM, SNOOZE, SET_ALARM};
enum states STATE = DISP_TIME;

// Variables for ADC
uint8_t  i;              //dummy variable
uint16_t adc_result;     //holds adc result

// Clock hour, minute, second
volatile uint8_t clock_s=0;
volatile uint8_t clock_m=1;
volatile uint8_t clock_h=12;
volatile uint8_t alarm_s=0;
volatile uint8_t alarm_m=0;
volatile uint8_t alarm_h=12;
volatile uint8_t snuze_s=0;
volatile uint8_t snuze_m=0;
volatile uint8_t snuze_h=12;
uint8_t hours12_24 = 12;
uint8_t am_pm = 0;
uint8_t alarmBeep=0;

// Text to be displayed to LCD
volatile char *lcdText1 = "Welcome";
volatile char *lcdText2 = "Welcome";
volatile uint8_t volume = 128;

// Number displayed to 7seg
volatile uint16_t segNum = 0;

// Number displayed to bargraph
volatile uint8_t barNum = 0;

// Function prototypes
void spiTxRx();
void spi_init();
void timer_init();
void digit_init();
void update7Seg();
void stateSwitcher();



//****************************************************************************
//      -- Serial Peripheral Interface Initialization --
//  Modified from Roger Traylor's source file
//****************************************************************************
void spi_init(void){

    // -- LCD INIT --
    /* Run this code before attempting to write to the LCD.*/ 
    DDRF  |= 0x08;  //port F bit 3 is enable for LCD
    PORTF &= 0xF7;  //port F bit 3 is initially low

    DDRB  |= 0x07;  //Turn on SS, MOSI, SCLK
    PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
    //see: /$install_path/avr/include/avr/iom128.h for bit definitions

    //Master mode, Clock=clk/4, Cycle half phase, Low polarity, MSB first
    SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
    SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)

    // -- Bargraph INIT --
    // Direction Registers
    DDRB |= (1<<RCLK) | (1<<SCLK) | (1<<MOSI) | (1<<PWM_BRT);
    DDRD |= (1<<SHLD_ENC) | (1<<OE_N_BG);

    // SPI Control Register
    SPCR |= (1<<SPE) | (1<<MSTR) | (0<<SPR1)|(1<<SPR0);

    // SPI Status Register
    SPSR |= (1<<SPI2X);

    // SPI Data Register
    PORTB &= ~(1<<OE_N_BG);
}



//******************************************************************************
//      -- Transmits and Receives to/from SPI --
//******************************************************************************
void spiTxRx() {
    static uint8_t textLine = 0;

    //clear_display();
    if (textLine) {
        cursor_home();
        string2lcd((char *)lcdText1);  //write upper half
        textLine = 0;
    } else {
        home_line2();
        string2lcd((char *)lcdText2);  //write upper half
        textLine = 1;
    }
    _delay_us(500);

    // Toggle Encoder Shift/Load
    PORTD &= ~(1<<SHLD_ENC);
    PORTD |= (1<<SHLD_ENC);

    // SPI write from global variable
    SPDR = barNum;

    // Wait for 8 clock cycles
    while(bit_is_clear(SPSR, SPIF)) {}

    // Save the most recent serial reading into global variable
    encoderState = SPDR;

    // Toggle Bargraph Register Clock
    PORTB |= (1<<RCLK);
    PORTB &= ~(1<<RCLK);
}



//******************************************************************************
//      -- Timer 0 Compare Interrupt --i
// Using the internal 32.768 Khz oscillator to implement a clock
//******************************************************************************
ISR(TIMER0_OVF_vect){
    static uint8_t clock=0;
    clock++;
    if (clock == 128){
        clock_s++;
        clock = 0;
    }
    if (clock_s >= 60) {
        clock_s = 0;
        clock_m++;
    }
    if (clock_m >= 60) {
        clock_m = 0;
        clock_h++;
    }
    if (clock_h > hours12_24) {
        clock_h -= hours12_24;
        if (am_pm)
            am_pm = 0;
        else
            am_pm = 1;
    }

    if (clock % 32 == 0){
        alarmBeep = !alarmBeep;
    }
}//ISR TIMER0_OVF_vect


// Needs to be put in a timer:




//*****************************************************************************
//      -- Timer 1 Compare Interrupt: Alarm signal generation -- 
//*****************************************************************************
ISR(TIMER1_COMPA_vect){
    //PORTD ^= (1<<D_BP);
    PORTD ^= (1<<D_BP);
}//ISR



//*****************************************************************************
//      -- Timer 2 Compare Interrupt: 7 Segment Brightness PWM -- 
//*****************************************************************************
// NO ISR NEEDED; PWM GOES STRAIGHT TO OUTPUT PIN



//*****************************************************************************
//      -- Timer 3 Compare Interrupt: Audio Amp Volume to DAC --
//      -- Also use this as a slower interrupt for SPI rx/tx --
//*****************************************************************************
//ISR(TIMER3_COMPA_vect){
//    //PORTE ^= (1<<PWM_VOL);
//    PORTD ^= (1<<PWM_VOL);

//}//ISR



//******************************************************************************
//      -- TIMER Initialization --
//******************************************************************************
void timer_init(){

    // Timer counter 0 setup, running off i/o clock

    // Asynchronous Status Register, pg107
    //    Run off of external clock
    ASSR  |= (1<<AS0);

    // Timer/Counter Interrupt Mask, pg109
    //    Timer 0: overflow interrupt enable
    TIMSK |= (1<<TOIE0) | (1<<OCIE1A) | (1<<OCIE3A);

    // Timer/Counter Control Register, pg104
    // Timer 0: 32kHz osc for internal clock
    //      Normal mode, OC0 disconnected, clkTOS with no prescalar
    TCCR0 = (1<<CS00);

    // Timer 1 init
    // Clear on compare, prescale and value set to give ~5kHz signal
    TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
    OCR1A = 0X00EF;

    // Timer 2: Phase-corrected PWM for 7 seg brightness, no prescale
    TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20);

    // Timer 3: PWM for audio volume, but also correct speed for SPI reading
    // CTC mode, Clear on match
    DDRE |= (1<<PWM_VOL);
    TCCR3A |= (1<<WGM33)|(1<<WGM32)|(1<<WGM31)|(1<<WGM30) | (1<<COM3A1) | (1<<CS30);
    OCR3A = 0x000F;

}

//******************************************************************************
//      -- Initializes the analog->digital converter --
//******************************************************************************
void adc_init(uint8_t pin) {

    //Initalize ADC and its ports
    DDRF  &= ~(_BV(pin)); //make port F bit 7 is ADC input
    PORTF &= ~(_BV(pin));  //port F bit 7 pullups must be off

    //single-ended, input PORTF bit 7, right adjusted, 10 bits
    // ADC Multiplexer Selection Register
    // Reference Selection = 01: Internal VRef
    // MUX = 00111: ADC7
    ADMUX |= (1<<REFS0) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0);

    //ADC enabled, don't start yet, single shot mode
    // factor is 128 (125khz)
    // ADC Control and Status Register A, ADC Enable, ADC Prescalar Selection = 128;
    ADCSRA |= (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}



//******************************************************************************
//      -- Update 7 seg with the most recent data --
//******************************************************************************
void update7Seg() {

    //make PORTA an output
    DDRA = 0xFF;

    //bound a counter (0-4) to keep track of digit to display 
    for (i=0; i<5; i++)
    {
        // Clear digit select
        PORTB &= SELCL;

        //update digit to display
        PORTB |= digitSelect[i];

        //send 7 segment code to LED segments
        PORTA = segment_data[i];


        //dimming/flicker correction
        //_delay_ms(10);
        _delay_us(1000);
    }//for

    PORTB &= SELCL;
    PORTB |= SELBN;
    PORTB |= digitSelect[2];
    DDRA = 0x00;
} // end update7seg



//******************************************************************************
//      -- Read ADC
//******************************************************************************
void readADC() {


    //poke ADSC and start conversion
    // ADC Control and Status Register A, ADC Start Conversion
    ADCSRA |= (1<<ADSC);

    //spin while interrupt flag not set
    // ADC Control and Status Register A, ADC Interrupt Flag
    while(bit_is_clear(ADCSRA, ADIF)) {}

    //its done, clear flag by writing a one 
    // ADC Interrupt Flag
    ADCSRA |= (1<<ADIF);

    //read the ADC output as 16 bits
    adc_result = ADC;      
} // end readADC


//******************************************************************************
//      -- Handles State Logic
//******************************************************************************
void stateSwitcher() {
    //volatile static char buffer[4] = "    ";
    //uint8_t temp = volume;

    switch (buttonState){
        case 0b01:
            STATE = SET_TIME;
            break;
        case 0b10:
            STATE = SET_ALARM;
            break;
        case 0b100:
            if ((STATE == SNOOZE) || (STATE == ALARM)) {
                STATE = DISP_TIME;
                buttonState = 0;
                snuze_s = alarm_s;
                snuze_m = alarm_m;
                snuze_h = alarm_h;
            } else
                STATE = ALARM;
            break;
        default:
            if (STATE == ALARM) {
                if (buttonState != 0)
                    STATE = SNOOZE;
                snuze_s = clock_s + 10;
                snuze_m = clock_m;
                snuze_h = clock_h;
            } else {
                if (STATE != SNOOZE) 
                    STATE = DISP_TIME;
                buttonState = 0;
            }
            break;
    }

    switch (STATE) {
        case DISP_TIME: // Display Time
            lcdText1 = "Displaying time ";
            lcdText2 = "                ";
            //strcat(lcdText2, itoa(temp, buffer, 10));
            //lcdText2[11] = 0;
            break;
        case SET_TIME:
            lcdText1 = "Use dials to    ";
            lcdText2 = "change time     ";
            break;
        case ALARM:
            lcdText1 = "     ALARM!     ";
            lcdText2 = "   (Snooze?)    ";
            buttonState = 0;
            break;
        case SET_ALARM:
            lcdText1 = "Use dials to    ";
            lcdText2 = "change alarm    ";
            break;
        case SNOOZE:
            lcdText1 = "Snoozing........";
            lcdText2 = "zzZzzZZZzzzZzZZz";
            break;
        default:
            lcdText1 = "    Welcome     ";
            lcdText2 = " (State error)  ";
            break;
    } // end switch
} // end stateSwitcher




//******************************************************************************
//                                  main
//  Does main stuff
//******************************************************************************
int main(void) {

    digit_init();
    timer_init();
    spi_init();
    spi_init();
    lcd_init();
    adc_init(CDS);
    clear_display();


    sei();

    while(1) {

        // State Machine Control!
        stateSwitcher();

        if (STATE != SET_ALARM) {
            if ((alarm_s + 100*alarm_m + 10000*alarm_h == clock_s + 100*clock_m + 10000*clock_h) 
                    || (snuze_s + 100*snuze_m + 10000*snuze_h == clock_s + 100*clock_m + 10000*clock_h)) 
                STATE = ALARM;

        }

        if (alarm_m >= 60)
            alarm_m = 0;
        if (alarm_h >= 60)
            alarm_h = 0;

        spiTxRx();

        interpret_encoders();
        // -- READ BUTTONS --
        toggle_button_bus();


        // -- 7 SEG BRIGHTNESS --
        // Read the value of the photo resistor
        readADC();
        // Set the brightness of the LCD
        OCR2 = 255- (adc_result)/4;



        // -- TIME DISPLAY --

        // Display the button latch state on the bargraph
        barNum = clock_s;
        //barNum = encoderState;

        // Convert minutes and hours to a number for displaying
        if (STATE == SET_ALARM)
            segNum = alarm_m + 100*alarm_h;
        else 
            segNum = clock_m + 100*clock_h;

        OCR3A = volume;

        // Update number to digitSelect[i]
        segsum(segNum);
        if (am_pm)
            segment_data[2] &= ~(1<<COLON3);
        else
            segment_data[2] |= (1<<COLON3);

        // colon blink
        switch (clock_s % 2) {
            case 0:
                if (STATE == ALARM)
                    segment_data[2] |= (1<COLON1) | (1<<COLON2);
                break;
            case 1:
                segment_data[2] &= ~((1<<COLON1) | (1<<COLON2));
                break;
            default:
                break;
        }

        // Prints new values to the 7 seg display
        update7Seg();

        // Alarm Beeping
        if (STATE == ALARM) {
            if (alarmBeep)
                DDRD |= (1<<D_BP);
            else    
                DDRD &= ~(1<<D_BP);
        } else 
            DDRD &= ~(1<<D_BP);

    }//while

    return 0;

}//main
