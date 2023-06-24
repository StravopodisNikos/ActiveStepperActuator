#ifndef ActiveStepperActuator_h
#define ActiveStepperActuator_h
/*
 *  Developed to RUN on Arduino MEGA Board.
 *  
 *  Author: Stravopodis N.
 */
#include "Arduino.h"
#include "AccelStepper.h"
#include "EEPROM.h"
#include "SoftwareSerial.h"
#include <config/pin_conf.h>
#include <config/error_codes.h>
#include <config/spi_options_list.h>
#include <config/state_machine_list.h>
#include <config/operation.h>
#include <config/stepper.h>
#include <config/eeprom_stepper.h>
#include <config/cmd_state_id.h>

#include "uart_comm_ovidius.h"
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\sync_codes.h>
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\packets.h>
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\ports.h>

namespace actuator_comm 
{ 
    typedef unsigned char debug_error_type;
    typedef enum ROT_DIR{CCW = 0, CW = 1}; // matches the typedef ln.563 in Accelstepper.h
    enum fn_exec_state {success, failed};

    class ActiveStepperActuator: public EEPROMClass, public AccelStepper, public uart_comm_ns::uart_comm_ovidius
    {
        private:
            AccelStepper StepperMotor;
            EEPROMClass access2eeprom;
            bool _fn_return;

            uint8_t _unit_id;

            // *** FUNCTION EXEC UTILS ***
            fn_exec_state _fn_state;
            bool _terminate_condition;
            debug_error_type _act_error;
            bool _talker_fn_return;
            bool _free2setSlaveGoal;

            // *** MEASUREMENTS ***
            float _cur_position_rad;
            long _cur_position_steps;
            float _cur_velocity_r_s;
            long _cur_velocity_steps_s;
            float _cur_current_A;
            long _cur_current_mA;
            float _cur_torque_nm;
            unsigned char _cur_state;

            // stepper settings
            int _micro_step;
            uint8_t _gear_ratio;
            unsigned int _min_pulse_width;
            volatile bool _left_limit_triggered;
            volatile bool _right_limit_triggered;
            volatile bool _home_triggered;
            volatile byte _current_dir;
            float _pos_limit_rad;
            float _vel_limit_rads;
            float _accel_limit_rads2;
            float _torque_limit_nm;
            long _cur_limit_mA;

            // *** STATE MACHINE TIMERS ***
            unsigned long _t_start_micros;
            unsigned long _t_start_millis; 
        
            // *** DATA TYPE DEFINITION ***
            //template <typename D>
            //void _setDataType(const unsigned char CMD_ID, D & data);

            long _ConvertRad2Step(float RAD);
            float _ConvertStep2Rad(long STEP);
            bool _isValidState(const unsigned char state_received);
            void _initCurrentPosition();
            void _saveCurrentPosition();
            void _assignVelocityProfile_rads(float vel);
            void _assignUnitID();

        public:
            ActiveStepperActuator(uint8_t ID);
            ~ActiveStepperActuator();
            void assignActuatorLimits(float pos, float vel, float accel,  float torq_nm, long cur_mA);
            
            void wake_up();
            void setup_unit();
            void home_unit();
    };
        
}
#endif