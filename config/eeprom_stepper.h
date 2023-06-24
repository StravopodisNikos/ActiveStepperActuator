// Arduino MEGA has 4096 bytes available for eeprom memory

// next address = prev address + sizeof(previous data type)!!!

// List of eeprom addresses   // TYPE             SIZE          USED       NEXT       
#define EEPROM_ADDR_ID  0     // unit ID       // 1 byte     // 0       // 1 
#define EEPROM_ADDR_CP  1     // current pos   // 4 bytes    // [1,4]   // 5
#define EEPROM_ADDR_PL  5     // pos limit     // 4 bytes    // [5,8]   // 9
#define EEPROM_ADDR_VL  9     // vel limit     // 4 bytes    // [9,12]  // 13
#define EEPROM_ADDR_AL  13    // accel limit   // 4 bytes    // [13,16] // 17
#define EEPROM_ADDR_TL  17    // torq limit    // 4 bytes    // [17,20] // 21
#define EEPROM_ADDR_CL  21    // current limit // 4 bytes    // [21,24] // 25