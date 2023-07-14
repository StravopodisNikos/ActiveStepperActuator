#ifndef ActiveStepperActuator_h
#define ActiveStepperActuator_h
/*
 *  Developed to RUN on Arduino DUE Board.
 *  
 *  Author: Stravopodis N.
 */
#include "Arduino.h"
#include <Stream.h>
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

    class CustomAccelStepper: public uart_comm_ns::uart_comm_ovidius
    {
    private:
        unsigned long _step_cnt;
        unsigned long _last_step_micros;
        int _stepPin;
        int _dirPin;
        int _enPin;
        uint8_t _min_pulse_width;
        float _gear_ratio;
        float _spr;
        float _q0_hat_rad;
        float _q1_hat_rad;
        float _h;
        float _delta_h_seg;
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
        float _q_last_accel;
        float _q_last_ctVel;
        float _cur_q_rad;           // last position achieved by executed trajectory
        float _q_prev;
        float _t0_s;
        float _dt_th_s;
        uint8_t _step_k;
        PROF_PHASE _cur_phase;
        unsigned long _steps2move;
        double _single_micro_step_rad;
        float _sd_s;
        unsigned long _sd_micros;
        unsigned long _net_sd_micros;

        // Flags
        bool _CT_VEL_PH_EXISTS;

        unsigned long _ConvertRad2Step(float RAD);
        float _ConvertStep2Rad(unsigned long STEP);
        void _extract_displacement();
        void _extract_sigma();   // sets the sign of _h and sets the direction rotation Pin
        void _extract_sigma_v0();
        void _extract_sigma_v1();
        bool _trajectory_existance_check();
        bool _ctVelPhase_existence_check();
        void _calculate_Vv_4dur_accel();
        void _calculate_Alim_4dur_accel();
        void _calculate_Vlim_4vel_accel();
        void _calculate_Ta();
        void _calculate_Td();
        void _calculate_T();
        void _set_theoretical_delta_t(float Tper);  
        void _calculate_dq(float V_last_seg);
        void _calculate_q(float q_last_seg, float V_last_seg);
        void _calculate_q_ctV();
        void _calculate_accel_rs2();
        unsigned long _return_steps2move(float delta_h);
        void _calculate_single_micro_step_rad();
        void _calculate_single_delay_s();
        void _calculate_single_delay_micros();
        unsigned long _return_net_single_delay_micros();
        void _setTrajectoryPosCon(float q1);                // Sets the position constraints for the desired trajectory, cons are set based on user provided data
        void _setTrajectoryVelCon(float v1);
        void _setTrajectoryTargets(float Tdur, float Vd, float Ad); // The targets may be overwritten based on the trajectory type
        void _extractSegmentData(uint8_t segment_cnt);
        void _printSegmentData(Stream &comm_serial);
        void _extractTrajData_4dur_accel();
        void _extractTrajData_4vel_accel();
        bool _evaluate_trajectory();

        // Motor stepping functions
        void _stepVarLength();
        bool _run_var_delay(unsigned long steps, unsigned long delay_micros);
        void _runSegment();  // must be called after _extractSegmentData() has been called. completes the segment for the steps/delay calculated
        void _executeTrajPhases(Stream &comm_serial);
        void _executeAccelPhase(Stream &comm_serial);
        void _executeCtVelPhase(Stream &comm_serial);
        void _executeDecelPhase(Stream &comm_serial);

        // Real time Data
        void _update_abs_q_rad();
        void _update_abs_dq_rs();

        // Data logging
        unsigned long _last_log_millis;
        void _data_log_print();

        struct SegmentDataStruct {
            unsigned long tot_steps;
            unsigned long net_single_delay_micros;
            float cur_q_rad;
        };
        SegmentDataStruct _cur_segment_data;

    public:
        CustomAccelStepper() = default;
        CustomAccelStepper(int step_pin, int dir_pin, int en_pin, float gr, float spr);
        ~CustomAccelStepper();

        bool executeTraj2GoalPos_4dur_accel(float Qgoal, float Time, float Accel, float v_con1, Stream &comm_serial);
        bool executeTraj2GoalPos_4vel_accel(float Qgoal, float Vel, float Accel, float v_con1, Stream &comm_serial);

        // REAL TIME VARS FOR POS/VEL DATA LOGGING
        float _ABS_Q_RAD;
        float _ABS_DQ_RS;
        float _ABS_Q_RAD_STEP0; // the abs pos when step counter is set to ), before each segment execution!

        // DIR IS CHANGED USING FEEDBACK FROM ACTUATOR LIMIT SWITCHES
        volatile uint8_t stp_dir;
    };

    //class ActiveStepperActuator: public AccelStepper, public uart_comm_ns::uart_comm_ovidius
    //class ActiveStepperActuator: public actuator_ns::CustomAccelStepper, public uart_comm_ns::uart_comm_ovidius
    class ActiveStepperActuator: public actuator_ns::CustomAccelStepper
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
            //void _importInitializationData(Stream& comm_serial); // PYTHON DOESNT WORK. DEPRECATED
            void _exportFloat_to_Logger(uint8_t cmd2logger, float exported_data);
            void _importFloat_from_Logger(uint8_t cmd2logger, float &imported_data);
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
            ActiveStepperActuator() = default;
            ActiveStepperActuator(uint8_t ID, int STEP_PIN, int DIR_PIN, int EN_PIN, float GEAR, float SPR);
            ~ActiveStepperActuator();
            void printActuatorLimits(Stream &comm_serial);
            //void ConversionTester(float RAD, long STEPS, Stream &comm_serial);

            void wake_up();
            //void setup_unit(AccelStepper *ptr2stepper);
            void setup_unit(Stream& comm_serial);
            float printCurrentAbsPosition();
            void  saveCurrentAbsPosition();
            //void home_unit(Stream &comm_serial);
            //bool go2GoalPos_vel(AccelStepper *ptr2stepper ,float abs_pos, float des_vel, Stream &comm_serial);
            //bool go2GoalPos_vel(float abs_pos, float des_vel, Stream &comm_serial);
    };

    
            
}


#endif