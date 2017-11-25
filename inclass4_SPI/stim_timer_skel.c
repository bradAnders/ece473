//
// stim_timer_skel.c
//
// Created: 10/14/2014 12:19:20 PM
//  Author: Justin Goins
//  Skeletonized for class use: R. Traylor 11.3.2014
//  Modifed to use hd44780_driver, R. Traylor 11.29.2016

// Gives practice in using both 8 and 16 bit timer counters as well as
// the use of a state machine to keep track of how button resources
// are used. Interrupts are not used but interrupt flags are manually
// manipulated.
//
// TCNT0 is used to count out a random time to wait for a users input.
// TCNT1 is used to count the time before the user hits a button
//
// Lines with code to be supplied by student are marked with a "*" in col 1.
//

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "hd44780.h"

enum states {SR_WELCOME, SR_RANDOM_DELAY, SR_TIMING_USER, SR_RESULTS};

//******************************************************************************
//                            spi_init                               
//Initalizes the SPI port to allow LCD access.
//******************************************************************************
void spi_init(void){

  DDRB |=  0x07;  //Turn on SS, MOSI, SCLK

  //* SPI Control Register
  //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, no interrupts 
  SPCR = (1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
  SPSR = (1<<SPI2X); //SPI at 2x speed (8 MHz)  

}//spi_init


int main(void) {
  uint16_t numticks = 0;
  enum states state = SR_WELCOME;// set default state

  spi_init();     //set up SPI
  lcd_init();     //set up LCD

  DDRB |= 0x80;   //port B bit 7 LED is the user signal 
  PORTB &= 0x7f;  //clear the LED
  DDRD = 0x00;    //all the pushbutton switches are inputs

  while(1) {
    switch (state) {
      case SR_WELCOME:
        {
          clear_display();
          string2lcd("REFLEX TESTER");
          line2_col1(); 
          string2lcd("Press any button");
          _delay_ms(100); // force minimum 100ms display time
          while (PIND == 0xFF) {};
          state = SR_RANDOM_DELAY; // progress to RANDOM_DELAY state
          break;
        }

      case SR_RANDOM_DELAY:
        {
          lcd_init();
          string2lcd("Press any button");
          line2_col1(); 
          string2lcd("after LED lights");

          // Set up TC0

          //* Timer/Counter Control Register 0      
          // set TC0 timer into normal mode and disable clock
          TCCR0 &= ~( (1<<WGM01)|(1<<WGM00) | (1<<COM01)|(1<<COM00) | (1<<CS02)|(1<<CS01)|(1<<CS00) );

          //* Timer/Counter Interrupt Mask Register
          // disable TC0 interrupts
          TIMSK &= ~( (1<<OCIE0)|(1<<TOIE0) );

          //* Timer/Counter Interrupt Flag Register
          // manually clear the TC0 overflow flag
          TIFR |= (1<<TOV0);

          //* Timer/Counter Control Register 0
          // start the timer with a 1024 prescaler, 16MHz/1024 = 15.625 KHz
          // Clock Select = 111
          TCCR0 |= (1<<CS02)|(1<<CS01)|(1<<CS00);    

          //Now we need to randomly wait between 2-10 seconds.  Since it takes 1.64 ms 
          //for the 8 bit timer to overflow, we need to loop between 122 - 610 times.
          //The following code is supposedly more random than other methods 
          uint16_t numIterations = rand() / (RAND_MAX / 488 + 1); // pick number between (0 - 487)

          numIterations += 122; // numIterations should now be between (122 - 610)

          do {

            //*
            while (bit_is_clear(TIFR,TOV0)) {}; // wait until the TC0 overflow flag is set

            //* Timer/Counter Interrupt Flag Register
            // manually clear the TC0 overflow flag
            TIFR |= (1<<TOV0);

            // note that the counter will automatically keep counting upward again
            numIterations--; // decrement number of iterations

          } while (numIterations > 0);

          //* Timer/Counter Control Register 0
          // disable the TC0 timer
          // Clock Select = 000
          TCCR0 &= ~( (1<<CS02)|(1<<CS01)|(1<<CS00) );


          state = SR_TIMING_USER; // progress to TIMING_USER state

          break;
        }
      case SR_TIMING_USER:
        {
          // Use 16 bit TC1 to measure the user's reaction time

          //* Timer/Counter Control Register 1B
          // disable noise canceler, set WGM1{3,2} to 0, and disable clock
          TCCR1B &= ~( (1<ICNC1) | (1<<WGM13)|(1<<WGM12) | (1<<CS12)|(1<<CS11)|(1<<CS00) );

          //* Timer/Counter Control Register 1A
          // disable all of the output compare pins and set WGM1{1,0} to 0
          TCCR1A &= ~( (1<<COM1A1)|(1<<COM1A0) | (1<<COM1B1)|(1<<COM1B0) | (1<<COM1C1)|(1<<COM1C0) | (1<<WGM11)|(1<<WGM10) );

          //* Timer/Counter Intrrupt Mask Register
          // disable TC1 interrupts in TIMSK
          TIMSK &= ~( (1<<OCIE1A)|(1<<OCIE1B) | (1<<TOIE1) );

          //* Extended Timer/Counter Interrupt Mask Register
          // disable TC1 interrupts	in ETIMSK
          ETIMSK &= ~( (1<<OCIE1C) ) ;

          //* Timer/Counter Interrupt Flag Register
          // manually clear the TC1 overflow flag
          TIFR |= (1<<TOV1);

          //* Timer/Counter Register
          // initialize the TC1 counter to 0
          TCNT1 = 0x0000;

          //Count the number of ticks until a button is pressed. Start the timer with a 1024 prescaler.
          //16MHz / 1024 = 15.625 KHz

          //* Timer/Counter Control Register 1B
          // start TC1 counter
          TCCR1B |= ( (1<<CS12)|(0<<CS11)|(1<<CS10) );


          PORTB |= 0x80; // light MSB LED so the user knows to push the button

          while ( ((TIFR & (1 << TOV1)) == 0) && (PIND == 0xFF) ) {}; // wait until button pressed or TC1 OVF set
          
          numticks = TCNT1;
          
          //* Timer/Counter Control Register 1B
          // stop the TC1 counter
          TCCR1B &= ~( (1<<CS12)|(1<<CS11)|(1<<CS10) );
          
          // note that the count is now stored in TCNT1
          state = SR_RESULTS; // progress to RESULTS state
          
          break;
        }

      case SR_RESULTS:
        {
          // Now we compute the results without using floating point arithmetic. The timer runs at 15.625KHz so there 
          // are 15.625 ticks in a millisecond. We can use this information to determine the user's reaction time.
          PORTB &= 0x7F; //disable LED
          //To compute milliseconds, we multiply by 8/125. Since we are multiplying a 16 bit number,
          //be sure to perform the math using a 32 bit number.
          uint32_t numMilliseconds = ((uint32_t)numticks * 8) / 125;

          clear_display();
          if (((TIFR & (1 << TOV1)) == 0) && (numMilliseconds == 0)) {
            // overflow wasn't triggered but numMilliseconds = 0, the user held down the button
            string2lcd("No cheating!!");
            line2_col1(); 
            string2lcd("Retry?");
            _delay_ms(1000);
          } else if ((TIFR & (1 << TOV1)) == 0) {
            // overflow wasn't triggered
            // display the time
            string2lcd("Your time:");
            line2_col1(); 
            lcd_int16(numMilliseconds, 3, 0, 0, 0);
            string2lcd(" ms");
          } else {
            // overflow was triggered
            // user took too long
            string2lcd(" Timer expired.");
            line2_col1(); 
            string2lcd("Press btn to rst");
          }

          _delay_ms(300); // add delay to avoid switch bouncing issues
          while (PIND == 0xFF) {}; // wait until a button is pressed
          state = SR_RANDOM_DELAY; // move back to the random delay state
          break;
        }
    }
  }
}
