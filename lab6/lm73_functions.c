// lm73_functions.c       
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <stdlib.h>
#include <util/delay.h>

volatile uint8_t lm73_wr_buf[2];
volatile uint8_t lm73_rd_buf[2];

//******************************************************************************** 
uint8_t lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c){
    uint8_t temp = 0;
    // Only read positive temperatures; check sign bit
    if (((lm73_temp) & (1<<15)) == 0) {
        temp = (lm73_temp>>7) & 0xFF;

        if (f_not_c != 0) {
            temp = (int)((temp * 1.8) + 32);
        }
    }
    
    itoa(temp, temp_digits, 10);
    return temp;
    
    /*
    // Cast as signed int
    int16_t signedInt = (int16_t) lm73_temp;
    uint8_t radixInt = (uint8_t) (((uint16_t)lm73_temp) & 0x00FF);
    radixInt = radixInt << 1;

    // Right shift in 0s or 1s depending on +/-
    if (signedInt >= 0) {
        signedInt = signedInt >> 7;
    } else {
        signedInt = (signedInt >> 7) | (0b11111110 << 8);
    }

    int8_t digitCount = 0;
    int bcdSum = 0;*/
/*
    while (digitCount < 4){
        
        bcdSum += (signedInt & 10) * (10^digitCount);
        signedInt = signedInt / 10;
        digitCount ++;
    }*/
    //itoa(signedInt, temp_digits, 10);
}
