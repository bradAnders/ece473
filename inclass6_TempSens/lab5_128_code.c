//	File: lab4_code.c
//	Author: Makenzie Brian
//	Date: 11.13.17
//	Description: Alarm Clock; Keeps time once set with encoders in button mode 1;
//	can set alarm to time with encoders in button mode 2; can turn alarm on/off 
//	with button 3; can snooze alarm with button 4 for 10 seconds; display changes brightness 
//	with ambient light; changes volume

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display and to the pushbuttons
//  PORTA.0 corresponds to segment A and pushbutton 0, 
//	PORTA.1 corresponds to segment B and pushbutton 1, etc.
//	The pushbuttons and the 7-segment are connected via pins A_2, B_2, etc.
//	on the 7-segmeant board and controlled via tri-state buffer.
//  PORTB bits 4-6 go to A,B,C inputs of the 74HC138 Decoder.
//	PORTB bits 2-3 are connected to the Bargraph Board using SPI.
//	Encoders are connected to PORTB, PORTC, and PORTE, see beloww for 
//	details.

//	ATmega128 	to 	7-Segment
//	PORTA.0		->	A
//	PORTA.1		->	B
//	PORTA.2		->	C
//	PORTA.3		->	D
//	PORTA.4		->	E
//	PORTA.5		->	F
//	PORTA.6		->	G
//	PORTA.7		->	DP
//	PORTB.4		->	SEL0
//	PORTB.5		->	SEL1
//	PORTB.6		->	SEL2
//	PORTB.7		->	PWM
//	GND			->	EN_N
//	GND			->	GND
//	VCC			->	EN
//	VCC			->	VDD

//	7-Segment 	to 	Pushbuttons
//	A_2			->	J1
//	B_2			->	J2
//	C_2			->	J3
//	D_2			->	J4
//	E_2			->	J5
//	F_2			->	J6
//	G_2			->	J7
//	DP_2		->	J8
//	DEC7		->	COM_EN

//	ATmega128	to	Bargraph Board
//	PORTD.2		->	regclk
//	PORTB.1		->	srclk
//	PORTB.2		->	sdin
//	PORTC.3		->	oe_n
//	GND			->	GND
//	VCC			->	VDD
//	NC			->	sd_out

//	ATmega128	to	Encoders
//	PORTB.3		->	SOUT
//	GND			->	SIN			//PC4
//	PORTC.2		->	CKINH
//	PORTB.1		->	SCK
//	PORTE.6		->	SH/LD
//	GND			->	GND
//	VCC			->	VDD

//	ATmega128	to	Pushbuttons
//	GND			->	COM_LVL		//PC1
//	GND			->	GND
//	VCC			->	VDD

//	ATmega128	to	Photocell
//	VCC			->	TopVoltageDivider
//	PORTF.0		->	MidVoltageDivider
//	GND			->	BotVoltageDivider

//	ATmega128	to	Audio
//	VCC			->	VDD
//	GND			->	GND
//	See schematic for others: uses PE3, PD1

//#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "uart_functions.h"


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];	

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//binary values for 7 seg into array
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF, 0xFF}; 

int8_t time[4] = {0};	//hour and minute for time and alarm

volatile uint8_t mode = 0;	//holds mode
volatile uint8_t enc1[2];		//value of first encoder
volatile uint8_t enc2[2];		//value of second encoder
volatile uint8_t prev_enc1[2] = {0};	//previous value of enc 1
volatile uint8_t prev_enc2[2] = {0};	//previous value of enc 2

volatile uint8_t alarm_state = 0;				//state of alarm (ON or OFF)
uint8_t alarm_sound = 0;				//whether to sound the alarm or not
volatile uint8_t snooze = 0;			//snooze on/off
volatile uint8_t snooze_counter = 0;	//counts snooze cycle
volatile uint8_t button = 10;
volatile uint16_t second = 0;		//hold sec
volatile uint8_t minute = 0;		//hold min

uint16_t adc_val = 0;		//collected value from adc

volatile uint8_t chk;
char* lcd_string_array = "Alarm off       "			//starts off
						 "In 00F Out 11F  ";

extern uint8_t lm73_wr_buf[2];	//array, writing to temp
extern uint8_t lm73_rd_buf[2];	//array, reading from temp



//*****************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//****************************************************************************
uint8_t chk_buttons(uint8_t button){
	static uint16_t state[8] = {0}; //holds present state
	//updates state of button
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	//buttons pressed if held for 12 cycles, not bounce
	if (state[button] == 0xF000){
		return 1;
	}//if
	return 0;
}//chk_buttons



//*****************************************************************************
//                              display_bar_graph   
//	Displays mode on bar graph via SPI.
//*****************************************************************************

void display_bar_graph(){
	switch(mode){
		case 0x00:			//no moed
			SPDR = 0x00;
		case 0x01:			//clock
			SPDR = mode;
			break;
		case 0x02:			//alarm
			SPDR = mode;
			break;
		default:
			break;
	}//switch

	SPDR = mode; 	//mode out
		
	while(bit_is_clear(SPSR,SPIF)){} 	//wait for transfer complete

	//data out
	PORTD |= 0x04;
	PORTD &= ~0x04;
	
}//display_bar_graph



//***********************************************************************************
//									spi_read
//	Reads spi to see encoder state and changes. Saves to changes to volatiles 
//	based on mode.
//***********************************************************************************
void spi_read(uint8_t mode){
	uint8_t temp = 0;
	uint8_t shift = 0;

	switch(mode){
		case 0x00:	//no mode
			return;
		case 0x01:	//clock
			shift = 0;
			break;
		case 0x02:	//alarm
			shift = 2;
			break;
		default:
			break;
	}//switch
	
	//save previous value of encoders for comparison
	prev_enc1[0] = enc1[0];
	prev_enc1[1] = enc1[1];
	prev_enc2[0] = enc2[0];
	prev_enc2[1] = enc2[1];

	//shifting data in from encoders
	PORTE = 0x00;		//parallel input to encoder (SH/LD low)
	PORTC |= 0x04;		//ignore serial output(clk inh high)
	SPDR = 0x00;		//send junk data, gets clk running for SPI
	PORTC &= ~0x04;		//allows for encoder to output data(clk inh low)
	PORTE = 0xFF;		//shift data out of encoder (SH/LD high)
	while(!(SPSR & (1<<SPIF))){}	//wait for transfer complete
	temp = SPDR;		//store encoder data

	enc1[0] = temp & 0x01;			//first encoder bit 1
	enc1[1] = temp & 0x02;			//first encoder bit 2
	enc2[0] = (temp & 0x04)>>2;		//second encoder bit 1
	enc2[1] = (temp & 0x08)>>2;		//second encoder bit 2
	
	//check enc 1 dir to set hour
	if(prev_enc1[0]==enc1[0]){
		if(enc1[0]==1 && (prev_enc1[1]<enc1[1])){
			time[shift+1]++;		//CW = increment
			
			if(time[shift+1]>23){	//limits to 24 hour clock
				time[shift+1] -= 24;
				}//if
		}//if
		if(enc1[0]==1 && (prev_enc1[1]>enc1[1])){
			time[shift+1]--;		//CCW = decrement
			
			//limits to 24 hour clock (not less than 0)
			if(time[shift+1]<0){
				time[shift+1] += 24;
			}//if
		}//if
	}//if
		
	//check enc 2 dir to set minute
	if(prev_enc2[0]==enc2[0]){
		if(enc2[0]==1 && (prev_enc2[1]<enc2[1])){
			time[shift]++;			//CW = increment

			if(time[shift]>59){		//limits to 60 min
				time[shift] -= 60;
			}//if
		}//if
		if(enc2[0]==1 && (prev_enc2[1]>enc2[1])){
			time[shift]--;			//CCW = decrement
			
			if(time[shift]<0){		//limits to 60 min
				time[shift] += 60;
			}//if
		}//if
	}//if
}//spi_read



//************************************************************************
//                              segment_sum                            
//Takes a 16-bit binary input value and places the appropriate 
//equivalent 4 digit BCD segment code in the array segment_data 
//for display. 
//     
//Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//************************************************************************
void segsum(uint8_t hour, uint8_t minute, uint8_t blink){
	segment_data[0] = dec_to_7seg[minute % 10];		//data to segs
	segment_data[1] = dec_to_7seg[minute / 10];
	segment_data[3] = dec_to_7seg[hour % 10];
	segment_data[4] = dec_to_7seg[hour / 10];


	if(hour / 10 == 0){		//blank leading zero
		segment_data[4] = 0xFF;
	}//if

	//indicates if there is an alarm on the LED display
	switch(alarm_state){
		case 0x00:
			segment_data[2] |= 0x04;
			break;

		case 0x01:
			segment_data[2] &= ~0x04;
			break;

		default:
			break;
	}//switch
}//segsum



//*************************************************************************
//							check_mode
//Checks buttons to see what mode has been selected
//*************************************************************************
void check_mode(){
	
	uint8_t i = 0;

	//PORTA all inputs
	PORTA = 0xFF;
	DDRA = 0x00;

	//enable tristate buffer
	PORTB |= 0x70;

	//check buttons for mode/actions
	for(i = 0; i < 7; i++){
		if(chk_buttons(i)){
			switch(i){
				case 0:	//first button, set time mode
					if(bit_is_clear(mode, 1)){	
						mode ^= 0x01;
					}//if
					second = 0;
					break;

				//if the second button is pressed: changes alarm time
				case 1: //second button, set alarm mode
					if(bit_is_clear(mode, 0)){
						mode ^= 0x02;;
					}//if
					break;

				case 2: //third button, alarm on/off
					alarm_state ^= 0x01;
					break;

				case 3: //fourth button, snooze alarm
					if(alarm_sound == 1){
						snooze = 0x01;
					}//if
					break;

				case 4: //fifth button, lower speaker volume
					if(OCR3A > 100){
						OCR3A = OCR3A - 10;
					}//if
					break;

				case 5: //sixth button, raise speaker volume
					if(OCR3A < 190){
						OCR3A = OCR3A + 10;
					}//if
					break;

				default: //other buttons, do nothing
					break;
			}//switch
		}//if
	}//for

	//disable tristate
	PORTB &= ~0x70;
	DDRA = 0xFF;
}//check_mode



//*****************************************************************************
//								check_alarm
//	Checks if alarm is or should be going off. Checks and resets snooze
//	values if over 10 seconds. Says what to send to LCD.
//****************************************************************************
void check_alarm(){
	if((alarm_sound == 0) && (alarm_state == 0x01)){	//alarm on
		lcd_string_array[6] = 'O';
		lcd_string_array[7] = 'n';
		lcd_string_array[8] = ' ';
		lcd_string_array[9] = ' ';
		lcd_string_array[10] = ' ';
		lcd_string_array[11] = ' ';

		if((time[0] == time[2]) && (time[1] == time[3])){ //if now
			alarm_sound = 1;
		}//if
	}//if

	else if(alarm_state == 0x00){	//alarm off
		alarm_sound = 0;			
		lcd_string_array[6] = 'O';
		lcd_string_array[7] = 'f';
		lcd_string_array[8] = 'f';
		lcd_string_array[9] = ' ';
		lcd_string_array[10] = ' ';
		lcd_string_array[11] = ' ';
	}//else if

	if(snooze == 0x01){				//if snooze
		alarm_sound = 0;			//no sound
		lcd_string_array[6] = 'S';
		lcd_string_array[7] = 'n';
		lcd_string_array[8] = 'o';
		lcd_string_array[9] = 'o';
		lcd_string_array[10] = 'z';
		lcd_string_array[11] = 'e';
	}//if

	if(snooze_counter >= 10){	//reset snooze and snooze counter
		snooze_counter = 0;
		snooze = 0x00;
		alarm_sound = 1;
		clear_display();
	}//if

	if(alarm_sound == 1){		//alarm going off
		TIMSK |= (1 << OCIE1A);
		lcd_string_array[6] = 'B';
		lcd_string_array[7] = 'E';
		lcd_string_array[8] = 'E';
		lcd_string_array[9] = 'P';
		lcd_string_array[10] = 'S';
		lcd_string_array[11] = ' ';
	}//if
	
	//turns off the alarm tone
	else if(alarm_sound == 0){
		TIMSK &= ~(1 << OCIE1A);
	}//else if	
	
	refresh_lcd(lcd_string_array);
}//check_alarm



//*****************************************************************************
//                           	   display   
//	Diplays digits on 7 seg based on mode and time.
//*****************************************************************************
void display(uint8_t mode){
	uint8_t i;			//counter variable for loops
	uint8_t shift = 0;	//shifts based on mode to know which time to use
	uint8_t blink = 0;	//blinks in alarms set mode
	
	switch(mode){
		case 0x01:		//clock
			shift = 0;
			blink = 1;
			break;
		case 0x02:		//alarm
			shift = 2;
			blink = 1;
			break;
		default:
			break;
	}//switch

	//init PORTB display first digit
	PORTB = 0x00;

	//get display values
	segsum(time[shift+1],time[shift],blink);

}//display



//***************************************************************************
//									check_adc()
//Checks the brightness seen by the photoresistor to control the dimming of
//the LEDs
//***************************************************************************
void check_adc(){
	ADCSRA |= (1<<ADSC);				//start transfer
	while(bit_is_clear(ADCSRA,ADIF));	//wait for transfer complete
	ADCSRA |= (1<<ADIF);				//clear flag
	adc_val = ADCH;						//take in value
	
	if(adc_val>100){					//compare value to range
		OCR2 = 50;
	}//else if
	else if(adc_val>90){
		OCR2 = 65;
	}//else if
	else if(adc_val>80){
		OCR2 = 80;
	}//else if
	else if(adc_val>70){
		OCR2 = 105;
	}//else if
	else if(adc_val>60){
		OCR2 = 135;
	}//else if
	else if(adc_val>50){
		OCR2 = 175;
	}//else if
	else if(adc_val>40){
		OCR2 = 205;
	}//else if
	else if(adc_val>30){
		OCR2 = 230;
	}//else if
	else{
		OCR2 = 250;
	}//else
}//check_adc



//***************************************************************************
//									lm73_read
//Gets data from sensor over I2C and converts data to temperature in F.
//***************************************************************************
void lm73_read(){

	uint16_t lm73_temp;

	uint8_t temperature[1] = {0};
	char temp1[2];	

	twi_start_rd(0x90, lm73_rd_buf, 2); //send a temp from sensor
	_delay_ms(2);

	lm73_temp = lm73_rd_buf[0];			//read in data
	lm73_temp = lm73_temp << 8;
	lm73_temp |= lm73_rd_buf[1];

	lm73_temp_convert(temperature, lm73_temp, 1); //convert to temp
	temperature[0] = ((temperature[0] * 9) / 5) + 32;	//to Farenheit
	itoa(temperature[0], temp1, 10);
	
	lcd_string_array[19] = temp1[0];	//inside temp
	lcd_string_array[20] = temp1[1];	//to LCD

}//lm73_read



//*************************************************************************
// 								Timer Interrupt 0
//	Keeps time of clock by incrementing seconds, and following values if 
//	necessary.
//*************************************************************************
ISR(TIMER0_OVF_vect){
	chk = 1;
	
	second++;

		if(second == 128){				//if 60 add to minute
			minute++;
			segment_data[2] ^= 0x03;	//blink colon
			lm73_read();				//get temp

			if(snooze == 0x01){			//add to snooze
				snooze_counter++;
			}//if

			if(minute == 60){			//add 1 minute to held time
				time[0]++;
				minute = 0;				//seconds to 0
			}//if

			if(time[0] == 60){			//add 1 hour to held time
				time[1]++;

				if(time[1] == 25){		//limits to 24 hour clock
					time[1] = 0;
				}
			
				time[0] = 0;			//minutes to 0
			}//if

			second = 0;
		}//if
}//ISR



//***************************************************************************
//								ISR - Timer 1 Compare
// PWM for alarm sound if alarm on.
//***************************************************************************
ISR(TIMER1_COMPA_vect){
	PORTD ^= (1<<PD3);
}

 

//************************************************************************
//							Timer Interrupt 2
// Updates based on peripherals and status.
//************************************************************************
ISR(TIMER2_OVF_vect){
	static uint8_t i = 0;

	if(chk == 1) {
		check_mode();
		spi_read(mode);
	}//if
		
	chk = 0;

	check_alarm();			//check mode stuff
	check_adc();			//check adc
	display_bar_graph();	//display mode on bargraph
	display(mode);			//display 7seg things

	DDRA = 0xFF;
	PORTA = segment_data[i];
	PORTB = (i << 4);

	i++;			//cycles through bit
	if(i > 5){
		i = 0;
	}//if

	refresh_lcd(lcd_string_array);	//update message
}//ISR



//*************************************************************************
//								init
//Initializes the timer/counters, ADC and SPI communication
//*************************************************************************
void init_all_the_things(){
	DDRB = 0xFF;		//PORTB output, low
	PORTB = 0x00;
	
	DDRC = 0xFF;		//PORTC low for pushbuttons
	PORTC = 0x00;

	DDRD = (1<<PD2);	//PD2 as output
	DDRD |= 0x03;

	DDRE = 0xFF;		//PORTE output, low
	PORTE = 0x00;

	DDRF |= 0x08;

	//init timer 0
	ASSR |= (1<<AS0);
	TIMSK |= (1<<TOIE0);
	TCCR0 |= (1<<CS00);

	//init timer 1
	DDRD |= (1<<PD3);	
	TCCR1B |= (1<<WGM12)|(1<<CS11)|(1<<CS10);
	OCR1A = 0x00FF;

	//init timer 2
	DDRB |= (1<<PB7);
	TCCR2 |= (1<<WGM21)|(1<<WGM20)|(1<<COM21)|(1<<COM20)|(1<<CS21)|(1<<CS20);
	TIMSK |= (1<<TOIE2);	

	//init SPI
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	SPSR = (1<<SPI2X);

	//init adc
	DDRF &= ~(1<<PF0);
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADMUX |= (1<<REFS0)|(1<<ADLAR);
	ADCSRA |= (1<<ADEN);


	//init timer 3
	TCCR3A |= (1<<WGM30)|(1<<COM3A1);
	TCCR3B |= (1<<WGM32)|(1<<CS32);
	OCR3A = 130;	//controls volume

	lcd_init();		//set up LCD

	init_twi();		//Init I2C

	uart_init();	//Init uart

	lm73_wr_buf[0] = 0b00000000;	//sensor to read temp
	twi_start_wr(0x90, lm73_wr_buf, 2);

	sei();			//enables interrupts

}//init_all_the_things



//***************************************************************************
void main(){
	dec_to_7seg[10] = 1;		//disp first digit

	segment_data[2] = 0xFF;		//init colon off

	time[1] = 1;	//start at 1:00

	init_all_the_things();	//all inits

	//loop forever
	while(1){
		
		uart_putc('s');
		lcd_string_array[27] = uart_getc();		//outside temp
		lcd_string_array[28] = uart_getc();		//for LCD

		_delay_ms(1000);
	}//while
}//main	
