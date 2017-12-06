#ifndef ENCODER_FUNCTIONS
#define ENCODER_FUNCTIONS

void interpret_encoders();
uint8_t checkDirection(uint8_t, uint8_t);
void increment(uint8_t);
void decrement(uint8_t);
//enum states; //{DISP_TIME, SET_TIME, ALARM, SNOOZE, SET_ALARM};



#define RWFN 4
#define LWFN 5
#define RMSK 0b0011
#define LMSK 0b1100
#define LEFT  0b10
#define RIGHT 0b01

#define CWSE 0b01
#define CCWS 0b10
#define bState0  0b110
#define bState1  0b000
#define bState2  0b010
#define bState4  0b100
#define bState5  0b101
#define bState10 0b1010

#endif //ENCODER_FUNCTIONS