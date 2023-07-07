#ifndef ActiveStepperActuator_h
#define ActiveStepperActuator_h
/*
 *  Developed to RUN on Arduino DUE Board.
 *  
 *  Author: Stravopodis N.
 */
#include "Arduino.h"
#include "AccelStepper.h"
//#include "EEPROM.h"
//#include "SoftwareSerial.h"
#include "ezButton.h"
//#include <config/pin_conf.h>
#include <config/error_codes.h>
#include <config/spi_options_list.h>
#include <config/state_machine_list.h>
#include <config/operation.h>
#include <config/stepper.h>
#include <config/actuator_sensors.h>
#include <config/eeprom_stepper.h>
#include <config/cmd_state_id.h>

#include "uart_comm_ovidius.h"
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\sync_codes.h>
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\packets.h>
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\ports.h>

namespace actuator_ns 
{ 
    typedef unsigned char debug_error_type;
    typedef enum ROT_DIR{CCW = 0, CW = 1}; // matches the typedef ln.563 in Accelstepper.h
    enum fn_exec_state {success, failed};

    class ActiveStepperActuator: public AccelStepper, public uart_comm_ns::uart_comm_ovidius
    {
        private:
            AccelStepper _StepperMotor;
            int _stepPin;
            int _dirPin;
            int _enPin;
            //EEPROMClass access2eeprom;
            ezButton* UnitSwitches[3];

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
            long _micro_step;
            long _gear_ratio;
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
            //void _initCurrentPosition();
            //void _saveCurrentPosition();
            void _assignVelocityProfile_rads(float vel);
            //void _assignUnitID();
            //void _setup_motor();
            void _measureCurrentACS712_A();
            bool _checkSafetyBounds(debug_error_type &error_code);

            long  _voltage_mapped_mV;
            float _voltage_mapped_V;

        public:
            ActiveStepperActuator(uint8_t ID, int mode, int step_pin, int dir_pin, int en_pin);
            ~ActiveStepperActuator();
            //void assignActuatorLimits(float pos, float vel, float accel,  float torq_nm, long cur_mA);
            //void printActuatorLimits(Stream &comm_serial);
            void ConversionTester(float RAD, long STEPS, Stream &comm_serial);

            void wake_up();
            //void setup_unit(AccelStepper *ptr2stepper);
            void setup_unit();
            void home_unit(Stream &comm_serial);
            //bool go2GoalPos_vel(AccelStepper *ptr2stepper ,float abs_pos, float des_vel, Stream &comm_serial);
            bool go2GoalPos_vel(float abs_pos, float des_vel, Stream &comm_serial);
    };

    class CustomAccelStepper
    {
    private:
        float _q0_hat_rad;
        float _q1_hat_rad;
        float _h;
        float _V0_hat_rs;
        float _V1_hat_rs;
        float _Vv_rs;
        float _Vmax_rs;
        float _Amax_rs2;
        float _A_rs2;
        float _Ta_s;
        unsigned long _Ta_ms;
        float _Td_s;
        unsigned long _Td_ms;
        float _Tct_s;
        unsigned long _Tct_ms;
        float _T_s;
        unsigned long _T_ms;  
        float _sigma;

        void _extract_displacement();
        void _extract_sigma();
        bool _trajectory_existance_check();
        float _calculate_Vv_4dur_accel();
              
    public:
        CustomAccelStepper(/* args */);
        ~CustomAccelStepper();
    };
            
}


#endif