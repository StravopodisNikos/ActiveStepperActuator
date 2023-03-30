#ifndef ct_list_h
#define ct_list_h

extern float READ_THE_RESPONSE_ONLY_F; // this cannot be defined as const because data type returned is float! -> defined in initialize() before setup() in .ino file
extern long  READ_THE_RESPONSE_ONLY_L; // this cannot be defined as const because data type returned is float! -> defined in initialize() before setup() in .ino file

// MASTER COMMANDS ID
const unsigned char TOROFF      = 0 ;
const unsigned char PING        = 1;
const unsigned char HOME        = 2;
const unsigned char GOPOS       = 3;
const unsigned char ASSPOS      = 4;
const unsigned char ASSVEL      = 5;
const unsigned char ASSACCEL    = 6;
const unsigned char READPOS     = 7;
const unsigned char READVEL     = 8;
const unsigned char READCUR     = 9;
const unsigned char READTOR     = 10;
const unsigned char READSTATE   = 11;

// SLAVE STATES
const unsigned char DIS         = 20; 
const unsigned char READY       = 21;
const unsigned char SETGOAL     = 22;
const unsigned char ASSIGN      = 23;
const unsigned char EXEC        = 24;

#endif