#include "Arduino.h"
#include <ActiveStepperActuator.h>

using namespace actuator_comm;

/*
 *  ***************      ActiveStepperActuator      ***************
 */

ActiveStepperActuator::ActiveStepperActuator(uint8_t ID) : AccelStepper(1, STEP_PIN, DIR_PIN, 0, 0, true) {
    // User defined based on stepper/unit configured
    _unit_id = ID;
    _assignUnitID(); // reset unit eeprom if ID has changed

    // Default for current stepper used in Ovidius joint 1 robot
    _micro_step = MS_1;
    _gear_ratio = GR;
    _min_pulse_width = MIN_PULSE_micros;

    _home_triggered = false;
    _left_limit_triggered = false;
    _right_limit_triggered = false;

};


ActiveStepperActuator::~ActiveStepperActuator() {}

/*
 *  ***************     Private Methods     ***************
 */
bool ActiveStepperActuator::_isValidState(const unsigned char state_received)
{
    if ( (state_received == DIS) || (state_received == READY) || (state_received == GOALSET) || (state_received == ASSIGN) || (state_received == EXEC) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

long ActiveStepperActuator::_ConvertRad2Step(float RAD) {
    return (long) (_micro_step * _gear_ratio * 0.1592 * RAD); // 0.1592 = 1/6.2832, 6.2832 = 2*π
}

float ActiveStepperActuator::_ConvertStep2Rad(long STEP) {
    return (float) (6.2832f * STEP) / (_micro_step * _gear_ratio) ; 
}

/*
void ActiveStepperActuator::GiveCurrentState()
{

}

template <typename R>
void ActiveStepperActuator::ReadSlaveMeasurement(const unsigned char READCMD_ID, R & meas_type_request ,R & measurement)
{
    
}  
template void ActiveStepperActuator::ReadSlaveMeasurement<float&>(const unsigned char READCMD_ID, float&, float&);
template void ActiveStepperActuator::ReadSlaveMeasurement<long&>(const unsigned char READCMD_ID, long&, long&);
template void ActiveStepperActuator::ReadSlaveMeasurement<unsigned char&>(const unsigned char READCMD_ID, unsigned char&, unsigned char&);

template <typename S>
void ActiveStepperActuator::AssignMotionProfile(const unsigned char ASSIGNCMD_ID, S & ass_type_request , S & assignment )
{

}  
template void ActiveStepperActuator::AssignMotionProfile<float&>(const unsigned char READCMD_ID, float&, float&);
template void ActiveStepperActuator::AssignMotionProfile<long&>(const unsigned char READCMD_ID, long&, long&);


template <typename O>
void ActiveStepperActuator::SetActuatorGoalPosition( O target_pos, debug_error_type & master_error )
{

}  
template void ActiveStepperActuator::SetActuatorGoalPosition<float>(float, debug_error_type & master_error);


*/

void ActiveStepperActuator::wake_up()
{

}

void ActiveStepperActuator::setup_unit()
{
    StepperMotor.setEnablePin(EN_PIN);
    StepperMotor.enableOutputs();
    StepperMotor.setMinPulseWidth(_min_pulse_width);

}

void ActiveStepperActuator::_initCurrentPosition() {
    access2eeprom.get(EEPROM_ADDR_CP, _cur_position_rad);
}

void ActiveStepperActuator::_saveCurrentPosition() {
    access2eeprom.put(EEPROM_ADDR_CP, _cur_position_rad);
}

void ActiveStepperActuator::_assignVelocityProfile_rads(float vel) {
    StepperMotor.setSpeed((float) _ConvertRad2Step(vel));
}

void ActiveStepperActuator::assignActuatorLimits(float pos, float vel, float accel,  float torq_nm, long cur_mA) {
    _pos_limit_rad = pos;
    access2eeprom.put(EEPROM_ADDR_PL, _pos_limit_rad);
    _vel_limit_rads = vel;
    access2eeprom.put(EEPROM_ADDR_VL, _vel_limit_rads); 
    _accel_limit_rads2 = accel;
    access2eeprom.put(EEPROM_ADDR_AL, _accel_limit_rads2);   
    _torque_limit_nm = torq_nm;
    access2eeprom.put(EEPROM_ADDR_TL, _torque_limit_nm); 
    _cur_limit_mA = cur_mA;
    access2eeprom.put(EEPROM_ADDR_CL, _cur_limit_mA);     
}

void ActiveStepperActuator::_assignUnitID() {
    access2eeprom.write(EEPROM_ADDR_ID, _unit_id);
}

void ActiveStepperActuator::home_unit() {
    /*
    do
    {
        if (_left_limit_triggered || _right_limit_triggered)
        {
            
        }
        
    } while (!_home_triggered); */
    
    _cur_position_rad = 0;
    _cur_position_steps = 0;
    StepperMotor.setCurrentPosition(_cur_position_steps);
    _saveCurrentPosition();
}