#ifndef ActiveStepperActuator_h
#define ActiveStepperActuator_h
/*
 *  Developed to RUN on Arduino DUE Board.
 *  
 *  Author: Stravopodis N.
 */
#include "Arduino.h"
//#include "AccelStepper.h"
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
#include <config/vel_profile.h>
#include "uart_comm_ovidius.h"
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\sync_codes.h>
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\packets.h>
#include <C:\Users\Nikos\Documents\Arduino\libraries\uart_comm_ovidius\utility\ports.h>

namespace actuator_ns 
{ 
    typedef unsigned char debug_error_type;
    typedef enum ROT_DIR{CCW = 0, CW = 1}; // matches the typedef ln.563 in Accelstepper.h
    enum class PROF_PHASE{ACCEL_PHASE = 0, DECEL_PHASE = 1, CT_VEL_PHASE = 2};
    enum fn_exec_state {success, failed};
    //class actuator_ns::CustomAccelStepper; // Forward declaration of the base class

    class CustomAccelStepper
    {
    private:
        unsigned long _step_cnt;
        unsigned long _last_step_micros;
        int _stepPin;
        int _dirPin;
        int _enPin;
        uint8_t _min_pulse_width;

        float _q0_hat_rad;
        float _q1_hat_rad;
        float _h;
        float _V0_hat_rs;
        float _V1_hat_rs;
        float _Vv_rs;
        float _Vmax_rs;
        float _Amax_rs2;
        float _Alim_rs2;
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
        float _sigma_v;
        float _sigma_v0;
        float _sigma_v1;
        float _dq;
        float _cur_dq_rad;          // last velocity achieved by executed trajectory
        float _q;
        float _cur_q_rad;           // last position achieved by executed trajectory
        float _q_prev;
        float _t0_s;
        float _dt_th_s;
        uint8_t _step_k;
        PROF_PHASE _cur_phase;
        unsigned long _steps2move;
        double _micro_step_rad;
        unsigned long _gear_ratio;
        float _sd_s;
        unsigned long _sd_micros;
        unsigned long _net_sd_micros;

        unsigned long _ConvertRad2Step(float RAD);
        float _ConvertStep2Rad(unsigned long STEP);
        void _extract_displacement();
        void _extract_sigma();
        void _extract_sigma_v0();
        void _extract_sigma_v1();
        bool _trajectory_existance_check();
        void _calculate_Vv_4dur_accel();
        void _calculate_Alim_4dur_accel();
        void _calculate_Ta();
        void _calculate_Td();
        void _set_theoretical_delta_t();  
        void _set_sigma_v();      
        void _calculate_dq();
        void _calculate_q();
        void _calculate_steps2move();
        void _calculate_micro_step_rad();
        void _calculate_single_delay_s();
        void _calculate_single_delay_micros();
        void _calculate_net_single_delay_micros();
        void _setTrajectoryPosCon(float q1);                // Sets the position constraints for the desired trajectory, cons are set based on user provided data
        void _setTrajectoryVelCon(float v1);
        void _setTrajectoryTargets(float Tdur, float Vd, float Ad); // The targets may be overwritten based on the trajectory type

        void _stepVarLength();

    public:
        CustomAccelStepper() = default;
        CustomAccelStepper(int step_pin, int dir_pin, int en_pin);
        ~CustomAccelStepper();

        void extractTrajData_4dur_accel();
        bool executeTraj2GoalPos_4dur_accel(float Qgoal, float Time, float Accel, float v_con1);
    };

    //class ActiveStepperActuator: public AccelStepper, public uart_comm_ns::uart_comm_ovidius
    class ActiveStepperActuator: public actuator_ns::CustomAccelStepper, public uart_comm_ns::uart_comm_ovidius
    {
        private:
            //AccelStepper _StepperMotor;
            actuator_ns::CustomAccelStepper _StepperMotor;

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


            bool _isValidState(const unsigned char state_received);
            //void _initCurrentPosition();
            //void _saveCurrentPosition();
            //void _assignVelocityProfile_rads(float vel);
            //void _assignUnitID();
            //void _setup_motor();
            void _measureCurrentACS712_A();
            bool _checkSafetyBounds(debug_error_type &error_code);

            long  _voltage_mapped_mV;
            float _voltage_mapped_V;

        public:
            ActiveStepperActuator(uint8_t ID, int STEP_PIN, int DIR_PIN, int EN_PIN);
            ~ActiveStepperActuator();
            //void assignActuatorLimits(float pos, float vel, float accel,  float torq_nm, long cur_mA);
            //void printActuatorLimits(Stream &comm_serial);
            //void ConversionTester(float RAD, long STEPS, Stream &comm_serial);

            void wake_up();
            //void setup_unit(AccelStepper *ptr2stepper);
            void setup_unit();
            //void home_unit(Stream &comm_serial);
            //bool go2GoalPos_vel(AccelStepper *ptr2stepper ,float abs_pos, float des_vel, Stream &comm_serial);
            //bool go2GoalPos_vel(float abs_pos, float des_vel, Stream &comm_serial);
    };

    
            
}


#endif