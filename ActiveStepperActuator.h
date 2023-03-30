#ifndef ActiveStepperActuator_h
#define ActiveStepperActuator_h
#include "Arduino.h"
#include <stdlib.h>
#include "AccelStepper.h"
#include "SerialTransfer.h"
#include <SPI.h>

#include <config/pin_conf.h>
#include <config/ct_list.h>
#include <config/error_codes.h>
#include <config/spi_options_list.h>
#include <config/state_machine_list.h>
//#include <config/cmd_state_id.h>

namespace actuator_comm
{ 
    typedef unsigned char debug_error_type;

    enum fn_exec_state {success, failed};
    //enum master_commands {TOROFF, PING, GOPOS, ASSVEL, ASSACCEL, READPOS, READVEL, READCUR, READTOR, READSTATE};
    //enum slave_states {DIS, READY, SETGOAL, ASSIGN, EXEC};
    enum spi_settings {SPI_OPT0, SPI_OPT1, SPI_OPT2, SPI_OPT3};

    class ActiveStepperActuatorTalker: public SPISettings 
    {
    private:
 
        // *** FUNCTION EXEC UTILS ***
        fn_exec_state _fn_state;
        bool _fn_return;
        bool _terminate_condition;
        debug_error_type _act_error;

        // *** MEASUREMENTS ***
        float _cur_position_rad;
        long _cur_position_steps;
        float _cur_velocity_r_s;
        long _cur_velocity_steps_s;
        float _cur_current_A;
        long _cur_current_mA;
        float _cur_torque_nm;
        unsigned char _last_slave_state;

        // *** SPI ***
        unsigned char _SS_PIN_STEPPER;
        unsigned char _SPI_OPT;
        unsigned char _RESPONSE;
        unsigned char _commdata;
        void _setSPISettings();
        bool _isValidState(unsigned char state_received);

        // *** DATA TYPE DEFINITION ***
        template <typename D>
        void _setDataType(const unsigned char CMD_ID, D & data);

        // *** STATE MACHINE TIMERS ***
        unsigned long _t_start_micros;
        unsigned long _t_start_millis; 
        
    public:
        ActiveStepperActuatorTalker(unsigned char ss_pin, unsigned char des_options);
        ~ActiveStepperActuatorTalker();

        typedef struct Data_packet
        {
            unsigned char cmd_st_id;
            float * ptr2data;
        }data_packet;

        typedef union Data_packet_union
        {
            data_packet message;
            unsigned char bytes[ sizeof(data_packet) ];
        } data_packet_union;

        // *** SEND-RECEIVE FUNCTIONS ***
        template <typename T>
        void send1time_single(T & data_send, T & data_received);
        void send1time_packet(Data_packet packet2send);
        template <typename R>
        void ReadSlaveMeasurement(const unsigned char READCMD_ID, R & meas_type_request , R & measurement);
        template <typename S>
        void AssignSlaveProfile(const unsigned char ASSIGNCMD_ID, S & ass_type_request , S & assignment );

        SPISettings *spi_setting_master;

        void SetupMaster();
        void Connect2Slave();
        void ReadSlaveState();

        // *** COMMAND SLAVE TO MOVE ***
        bool setActuatorGoalPosition();

    };

    class ActiveStepperActuatorListener: public AccelStepper
    {
        private:
            unsigned char _cur_state_listener;

        public:
            ActiveStepperActuatorListener(/* args */);
            ~ActiveStepperActuatorListener();

            void SetupSlave();
    };
        
    

}
#endif