#define MS_1        3200L // Microstepping setting switched on the Driver
#define MS_2        4000L // Microstepping setting switched on the Driver
#define MS_1_F      3200.0f

#define GR          40L   // Gear ratio
#define GR_F        40.0f

#define MIN_DQ      0.01f // Min angular velocity of motor-Used when _dq=0, in order to avoid numeric error @ STEPS calculation
#define NEMA34_STEP_PIN     24
#define NEMA34_DIR_PIN      26
#define NEMA34_EN_PIN       22
#define MIN_PULSE_micros    19 

#define HOME_SWITCH_PIN         23
#define LEFT_SWITCH_PIN         25
#define RIGHT_SWITCH_PIN        27
#define SWITCH_DEBOUNCE_millis  100
      