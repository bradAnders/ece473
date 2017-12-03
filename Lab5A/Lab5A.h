#ifndef MAIN
#define MAIN

//  File:     main.h
//  Author:   Bradley Anderson
//  Created:  Dec-2, 2017
//
//  Collaboration:
//  None

// Pin in/out
#define RCLK PB0
#define SCLK PB1
#define MOSI PB2
#define MISO PB3
#define SEL0 PB4
#define SEL1 PB5
#define SEL2 PB6
#define PWM_BRT  PB7
#define SHLD_ENC PD2
#define OE_N_BG PD3
#define D_BP PD4
#define PWM_VOL PE3
#define CDS  PF7


// DEMUX to LED wiring
#define SELD1 (0x0 << SEL0)
#define SELD2 (0x1 << SEL0)
#define SELD3 (0x3 << SEL0)
#define SELD4 (0x4 << SEL0)
#define SELDD (0x2 << SEL0)
#define SELBN (0x7 << SEL0)
#define SELCL !SELBN
#define COLON1 0b00
#define COLON2 0b01
#define COLON3 0b10



// Hard code boolean type
#define TRUE 1
#define FALSE 0
#define true 1
#define false 0
#define True 1
#define False 0
typedef unsigned char bool;



// Global variables from other files
extern uint8_t segment_data[5];
//extern uint8_t dec_to_7seg[12];
extern uint8_t digitSelect[8];
extern volatile uint8_t buttonState;
// Holds state of encoders
extern volatile uint8_t encoderState;

enum states {DISP_TIME, SET_TIME, ALARM, SNOOZE, SET_ALARM};

#endif // MAIN