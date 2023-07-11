#include "Arduino.h"
#include <ActiveStepperActuator.h>

using namespace actuator_ns;

/*
 *  ***************** ActiveStepperActuator *****************
 */

//ActiveStepperActuator::ActiveStepperActuator(uint8_t ID, int mode, int step_pin, int dir_pin, int en_pin ) : _StepperMotor(mode, step_pin, dir_pin) {
ActiveStepperActuator::ActiveStepperActuator(uint8_t ID, int STEP_PIN, int DIR_PIN, int EN_PIN, float GEAR, float SPR) : _StepperMotor(STEP_PIN, DIR_PIN, EN_PIN, GEAR, SPR), uart_comm_ns::uart_comm_ovidius() {    
//ActiveStepperActuator::ActiveStepperActuator(uint8_t ID) {
    // User defined based on stepper/unit configured
    _unit_id = ID;
    //_assignUnitID(); // reset unit eeprom if ID has changed

    // Create class objects for the limit switches
    UnitSwitches[0] = new ezButton(HOME_SWITCH_PIN);
    UnitSwitches[1] = new ezButton(LEFT_SWITCH_PIN); 
    UnitSwitches[2] = new ezButton(RIGHT_SWITCH_PIN);

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

/*
void ActiveStepperActuator::_setup_motor() {

    // Configure the Driver Pins
    _StepperMotor.setPinsInverted(false, false, true);
    _StepperMotor.setEnablePin(_enPin);
    _StepperMotor.enableOutputs();
    // Safety microstep implementation threshold
    _StepperMotor.setMinPulseWidth(_min_pulse_width);
    // Safety stepper operation limits
    access2eeprom.get(EEPROM_ADDR_AL, _accel_limit_rads2);
    _StepperMotor.setAcceleration( 500000.0f );
    access2eeprom.get(EEPROM_ADDR_VL, _vel_limit_rads);
    _StepperMotor.setMaxSpeed( 7000.0f );     

}
*/

//void ActiveStepperActuator::setup_unit(AccelStepper *ptr2stepper)
void ActiveStepperActuator::setup_unit()
{
    // Configure Stepper Motor
    //_setup_motor();
    //_StepperMotor.setCurrentPosition(0);
    //ptr2stepper->setCurrentPosition(0);

    // Configure Limit Switches
    UnitSwitches[0]->setDebounceTime(SWITCH_DEBOUNCE_millis);
    UnitSwitches[1]->setDebounceTime(SWITCH_DEBOUNCE_millis);
    UnitSwitches[2]->setDebounceTime(SWITCH_DEBOUNCE_millis);

    // Configure ACS Current Sensor Module
    pinMode(ACS_VOLTAGE_AN_PIN, INPUT);
}

/*
void ActiveStepperActuator::_initCurrentPosition() {
    access2eeprom.get(EEPROM_ADDR_CP, _cur_position_rad);
}

void ActiveStepperActuator::_saveCurrentPosition() {
    access2eeprom.put(EEPROM_ADDR_CP, _cur_position_rad);
}
*/
/*
void ActiveStepperActuator::_assignVelocityProfile_rads(float vel) {
    _StepperMotor.setSpeed((float) _ConvertRad2Step(vel));
}
*/
/*
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


void ActiveStepperActuator::printActuatorLimits(Stream &comm_serial) {
    comm_serial.println("READING EEPROM...");
    access2eeprom.get(EEPROM_ADDR_PL, _pos_limit_rad);
    comm_serial.print("POSITION LIMIT[rad]: [ "); comm_serial.print(_pos_limit_rad,4); comm_serial.println(" ]");
    access2eeprom.get(EEPROM_ADDR_VL, _vel_limit_rads);
    comm_serial.print("VELOCITY LIMIT[rs]: [ "); comm_serial.print(_vel_limit_rads,4); comm_serial.println(" ]");   
    access2eeprom.get(EEPROM_ADDR_AL, _accel_limit_rads2);
    comm_serial.print("ACCELERATION LIMIT[rs2]: [ "); comm_serial.print(_accel_limit_rads2,4); comm_serial.println(" ]");  
    access2eeprom.get(EEPROM_ADDR_TL, _torque_limit_nm);
    comm_serial.print("TORQUE LIMIT[Nm]: [ "); comm_serial.print(_torque_limit_nm,4); comm_serial.println(" ]");
    access2eeprom.get(EEPROM_ADDR_CL, _cur_limit_mA);
    comm_serial.print("CURRENT LIMIT[mA]: [ "); comm_serial.print(_cur_limit_mA); comm_serial.println(" ]");        
}

void ActiveStepperActuator::_assignUnitID() {
    access2eeprom.write(EEPROM_ADDR_ID, _unit_id);
}
*/
/*
void ActiveStepperActuator::home_unit(Stream &comm_serial) {

    UnitSwitches[0]->loop();
    UnitSwitches[1]->loop();
    UnitSwitches[2]->loop();

    if (UnitSwitches[0]->isReleased())
    {   
        _home_triggered = false;

        // Start moving motor CW
        _StepperMotor.move(HOMING_STEPS);

        do
        {
            //comm_serial.println("Motor running...");
            _StepperMotor.run();

            if (UnitSwitches[1]->isPressed())
            {
                _left_limit_triggered = true;
            }
            if (UnitSwitches[2]->isPressed())
            {
                _right_limit_triggered = true;
            }
            if (UnitSwitches[1]->isReleased())
            {
                _left_limit_triggered = false;
            }
            if (UnitSwitches[2]->isReleased())
            {
                _right_limit_triggered = false;
            }
            if (_left_limit_triggered || _right_limit_triggered)
            {
                _StepperMotor.move(-HOMING_STEPS);
            }
            
            if (UnitSwitches[0]->isPressed())
            {
                _home_triggered = true;
            }
        } while (!_home_triggered); 
    }

    _cur_position_rad = 0;
    _cur_position_steps = 0;
    _StepperMotor.setCurrentPosition(_cur_position_steps);
    //_saveCurrentPosition();
}
*/
//bool ActiveStepperActuator::go2GoalPos_vel(AccelStepper *ptr2stepper , float abs_pos, float des_vel, Stream &comm_serial) {
/*
bool ActiveStepperActuator::go2GoalPos_vel(float abs_pos, float des_vel, Stream &comm_serial) {

    // set targets
    comm_serial.print("STEPS to move = "); comm_serial.println(_ConvertRad2Step(abs_pos));
    float speed = (float) _ConvertRad2Step(des_vel); 
    _StepperMotor.setSpeed( speed );
    comm_serial.print("CT VEL = "); comm_serial.println( speed );
    
    _StepperMotor.moveTo(_ConvertRad2Step(abs_pos));
    delay(1000);

    // step the motor
    do
    {
        //ptr2stepper->run();
        _StepperMotor.runSpeed();

        //comm_serial.print("REMAINING STEPS = "); comm_serial.println(_StepperMotor.distanceToGo());
    } while ( !(_StepperMotor.distanceToGo() == 0 ));

    _cur_position_rad = abs_pos;
}
*/
/*
void ActiveStepperActuator::ConversionTester(float RAD, long STEPS, Stream &comm_serial) {
    comm_serial.print("RAD 2 STEPS: "); comm_serial.println(_ConvertRad2Step(RAD));
    comm_serial.print("STEPS 2 RAD: "); comm_serial.println(_ConvertStep2Rad(STEPS));
}
*/

void ActiveStepperActuator::_measureCurrentACS712_A()
{   
    long sum_analog_voltage_measurement = 0;
    long final_analog_voltage_measurement;
    long analog_voltage_measurement;
    
    for (size_t i = 0; i < ACS_SAMPLES; i++)
    {
        analog_voltage_measurement = (long) analogRead(ACS_VOLTAGE_AN_PIN);
        sum_analog_voltage_measurement = sum_analog_voltage_measurement + analog_voltage_measurement;
        delayMicroseconds(ACS_tr_1nF_micros);
    }
    final_analog_voltage_measurement = (long) (sum_analog_voltage_measurement / ACS_SAMPLES);
    
    // analog -> voltage
    _voltage_mapped_mV = map( final_analog_voltage_measurement, 0, 1024, 0, 5000);
    _voltage_mapped_V =  float ( (_voltage_mapped_mV) / 1000.0f) ;

    // direct current calculation formula
    _cur_current_A = fabs( ( _voltage_mapped_V - ACS_VOLTAGE_START) / ACS_30A_SENSITIVITY );
    _cur_current_mA = 1000L * _cur_current_A;

    return;
}

bool ActiveStepperActuator::_checkSafetyBounds(debug_error_type &error_code) {
// Checks if POS-VEL-CUR-TOR bounds are validated and returns corresponding code
    error_code = ACT_NO_ERROR;

    if (_cur_current_mA > _cur_limit_mA)
    {
        error_code = CUR_LIMIT_REACHED;
    }

    if (abs(_cur_position_rad) > _pos_limit_rad)
    {
        error_code = POS_LIMIT_REACHED;
    }   

    if (abs(_cur_velocity_r_s) > _vel_limit_rads)
    {
        error_code = VEL_LIMIT_REACHED;
    }

    if (abs(_cur_torque_nm) > _torque_limit_nm)
    {
        error_code = TOR_LIMIT_REACHED;
    }    

    if (error_code == ACT_NO_ERROR)
    {
        return true;
    } else {
        return false;
    }
    
}

/*
 *  ***************** CustomAccelStepper *****************
 */

CustomAccelStepper::CustomAccelStepper(int step_pin, int dir_pin, int en_pin, float gr, float spr)
{   
    // Set the Motor Driver Configuration
    _spr = spr;
    _gear_ratio = gr;

    // Set the Motor Pins
    _stepPin = step_pin;
    _dirPin  = dir_pin;
    _enPin   = en_pin;
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_enPin, LOW);
    stp_dir = CCW;
    digitalWrite(_dirPin, stp_dir);

    _min_pulse_width = MIN_PULSE_micros;

    _step_cnt = 0;
    _last_step_micros = 0;

    _CT_VEL_PH_EXISTS = true;

    // Initialization of current position/velocity -> ONLY temporary, until homing function is implemented
    _cur_q_rad = 0.0f;
    _cur_dq_rad = 0.0f;
}

CustomAccelStepper::~CustomAccelStepper()
{
}

unsigned long CustomAccelStepper::_ConvertRad2Step(float RAD) {
    double temp = static_cast<double> (0.15915f * abs(RAD)); 
    return round(_spr * _gear_ratio * temp);  // 0.1592 = 1/6.2832, 6.2832 = 2*π
}

float CustomAccelStepper::_ConvertStep2Rad(unsigned long STEP) {
    // the sign must be determined with another function!
    return (float) (6.2832f * STEP) / (_spr * _gear_ratio) ; 
}

void CustomAccelStepper::_extract_displacement() {
    _h = _q1_hat_rad - _q0_hat_rad;
}

void CustomAccelStepper::_extract_sigma() {
    if ( _h > 0  )
    {
        _sigma = 1.0f;
        stp_dir = CCW;
        digitalWrite(_dirPin, stp_dir);
    }
    else if ( _h < 0 ) {
        _sigma = -1.0f;
        stp_dir = CW;
        digitalWrite(_dirPin, stp_dir);
    } else {
        _sigma = 0; // motor doesn't move=>terminates t rajectory execution functions
    }
}

void CustomAccelStepper::_extract_sigma_v0() {
    if ( _Vv_rs > _V0_hat_rs )
    {
        _sigma_v0 = 1.0f;
    }
    else {
        _sigma_v0 = -1.0f;
    }    
}

void CustomAccelStepper::_extract_sigma_v1() {
    if ( _V1_hat_rs > _Vv_rs )
    {
        _sigma_v1 = 1.0f;
    }
    else {
        _sigma_v1 = -1.0f;
    }    
}

bool CustomAccelStepper::_trajectory_existance_check() {
    // Eq. (3.14) p.71 Melchiorri Book
    if ( ( (_Amax_rs2 * _h) - 0.5f * abs( sq(_V0_hat_rs)-sq(_V1_hat_rs)) ) < 0)
    {
        return false; // Feasible
    } else {
        return true;  // Not feasible
    }
}

void CustomAccelStepper::_calculate_Vv_4dur_accel() {
    // Eq. for Vv in p.72 Melchiorri Book
    float __aT  = _Amax_rs2 * _T_s;
    float __4ah = 4.0f * _Amax_rs2 * _h;
    float __2aT = 2.0f * _Amax_rs2 * (_V0_hat_rs + _V1_hat_rs) * _T_s;
    float __dV  = _V0_hat_rs - _V1_hat_rs;
    _Vv_rs = 0.5f * ( _V0_hat_rs + _V1_hat_rs + __aT - sqrt( sq(__aT)  -  __4ah  +  __2aT  - sq(__dV) ) );
}

void CustomAccelStepper::_calculate_Alim_4dur_accel() {
    // Eq. (3.15) p.72 Melchiorri Book
    float __sV = _V0_hat_rs + _V1_hat_rs;
    float __sV2 = sq(_V0_hat_rs) + sq(_V1_hat_rs);
    _Alim_rs2 = ( (2.0f*_h) - (_T_s*__sV) + sqrt(4.0f*sq(_h) - 4.0f*_h*__sV*_T_s + 2.0f*__sV2*sq(_T_s)) ) / (sq(_T_s));
}

void CustomAccelStepper::_calculate_Ta() {
    _Ta_s = abs(_Vv_rs - _V0_hat_rs) / abs(_Amax_rs2) ;
}

void CustomAccelStepper::_calculate_Td() {
    _Td_s = abs(_V1_hat_rs - _Vv_rs) / abs(_Amax_rs2) ;
}

void CustomAccelStepper::_set_theoretical_delta_t() {
   _dt_th_s = (float) ( _step_k * _Ta_s ) / profile_steps;
}

void CustomAccelStepper::_calculate_dq() {
   _dq = _V0_hat_rs +  _A_rs2 * _dt_th_s; 
}

void CustomAccelStepper::_calculate_q() {
   _q =  _q0_hat_rad + _V0_hat_rs*_dt_th_s + 0.5f * _A_rs2 * sq(_dt_th_s);
}

void CustomAccelStepper::_calculate_accel_rs2() {
    switch (_cur_phase)
    {
    case PROF_PHASE::ACCEL_PHASE:
        _A_rs2 = _sigma_v0 * abs((_Vv_rs-_V0_hat_rs) / _Ta_s);
        break;
    case PROF_PHASE::DECEL_PHASE:
        _A_rs2 = _sigma_v1 * abs((_V1_hat_rs - _Vv_rs) / _Td_s);   
    default:
        break;
    }    
}

unsigned long CustomAccelStepper::_return_steps2move() {
    unsigned long steps2move = _ConvertRad2Step(abs(_q - _q_prev));
    return steps2move;
}

void CustomAccelStepper::_calculate_single_micro_step_rad() {
    _single_micro_step_rad = 6.2832f / (_spr * _gear_ratio);
}

void CustomAccelStepper::_calculate_single_delay_s() {
    if (_dq != 0 )
    {
       _sd_s = abs(_single_micro_step_rad/_dq);
    } else {
       _sd_s = 0;
    }
}

void CustomAccelStepper::_calculate_single_delay_micros() {
    if (_dq != 0 )
    {
       _sd_s = abs(_single_micro_step_rad/_dq);
    } else {
       _sd_s = 0;
    }
    _sd_micros = (unsigned long) ( _sd_s * 1000000L ); 
}

unsigned long CustomAccelStepper::_return_net_single_delay_micros() {
    unsigned long net_sd_micros = (unsigned long) (_sd_micros - _min_pulse_width); 
    return net_sd_micros;
}

void CustomAccelStepper::_setTrajectoryPosCon(float q1) {
    _q0_hat_rad = _cur_q_rad;
    _q1_hat_rad = q1;
    _h = _q1_hat_rad - _q0_hat_rad;
}

void CustomAccelStepper::_setTrajectoryVelCon(float v1) {
    _V0_hat_rs = _cur_dq_rad;
    _V1_hat_rs = v1;  // can be violated, if trajectory data lead to not feasible implementation
}

void CustomAccelStepper::_setTrajectoryTargets(float Tdur, float Vd, float Ad) {
    _T_s = Tdur;
    _Vv_rs = Vd;
    //_A_rs2 = Ad;
    _Amax_rs2 = Ad; // we always consider that Ad = Amax
}

void CustomAccelStepper::_extractTrajData_4dur_accel() {
    // Here, based on theory, the following are calculated:
    // 1. Vv
    _calculate_Vv_4dur_accel(); // Now Vv is assigned the calculated value!
    // 2. Ta, Td, Tct
    _calculate_Ta();
    _calculate_Td();
    _Tct_s = _T_s - (_Ta_s + _Td_s);
    // 3. Alim
    _calculate_Alim_4dur_accel();
    // 4. sigma, sigma_v
    _extract_sigma();
    _extract_sigma_v0();
    _extract_sigma_v1();

    // IV. In order for a ct velocity pahse to exist always, must be: Amax>Alim
    if (_Amax_rs2 <= abs(_Alim_rs2))
    {
        _Amax_rs2 = abs(_Alim_rs2) + Aoff;
        _CT_VEL_PH_EXISTS  = true;
    }
}

void CustomAccelStepper::_extractSegmentData(uint8_t segment_cnt) {
    _step_k = segment_cnt;
    _q_prev = _cur_q_rad;
    _set_theoretical_delta_t();
    _calculate_dq();
    _calculate_q();

    _cur_segment_data.tot_steps = _return_steps2move(); // needs _q_prev, _q
    _calculate_single_delay_micros();
    _cur_segment_data.net_single_delay_micros = _return_net_single_delay_micros();
    _cur_segment_data.cur_q_rad = _q;
}

void CustomAccelStepper::_printSegmentData(Stream &comm_serial) {
    comm_serial.println("TOTAL STEPS - SINGLE DELAY MICROS - Qt");
    comm_serial.print(_cur_segment_data.tot_steps);  comm_serial.print(" - "); comm_serial.print(_cur_segment_data.net_single_delay_micros);
    comm_serial.print(" - ");  comm_serial.println(_cur_segment_data.cur_q_rad,4);
}

void CustomAccelStepper::_runSegment() {
    // Start loop based on the steps/delay it must move for the current segment
    _step_cnt = 0; // resets counter of steps for the new segment
    do
    {
        // Here State machine function can be implemented!

        // runs the motor for a number of steps with the same delay time provided,
    } while ( !_run_var_delay(_cur_segment_data.tot_steps, _cur_segment_data.net_single_delay_micros) ); 
}

void CustomAccelStepper::_executeAccelPhase(Stream &comm_serial) {
    _cur_phase = PROF_PHASE::ACCEL_PHASE;
    for (size_t i = 1; i < PHASE_SEGMENTS_MIN; i++)
    {
        if (i == 1) // compute acceleration only for the 1st segment
        {
            _calculate_accel_rs2(); 
        }
        _extractSegmentData(i);
        _printSegmentData(comm_serial);
        _runSegment();
    }
}

void CustomAccelStepper::_executeDecelPhase(Stream &comm_serial) {
    _cur_phase = PROF_PHASE::DECEL_PHASE;
    for (size_t i = 1; i < PHASE_SEGMENTS_MIN; i++)
    {
        if (i == 1) // compute acceleration only for the 1st segment
        {
            _calculate_accel_rs2(); 
        }
        _extractSegmentData(i);
        _printSegmentData(comm_serial);
        _runSegment();
    }
}

void CustomAccelStepper::_executeCtVelPhase(Stream &comm_serial) {
    _cur_phase = PROF_PHASE::CT_VEL_PHASE;
    for (size_t i = 1; i < PHASE_SEGMENTS_MIN; i++)
    {
        if (i == 1) // compute segment data only for the 1st segment
        {
            _extractSegmentData(i);
            _printSegmentData(comm_serial);
        }
        _runSegment();  
    }
}

void CustomAccelStepper::_executeTrajPhases(Stream &comm_serial) {
    // I. Accel Phase
    _executeAccelPhase(comm_serial);

    // II. Ct Vel Phase
    if (_CT_VEL_PH_EXISTS) {
        _executeCtVelPhase(comm_serial);
    }
    
    // III. Decel Phase
    _executeDecelPhase(comm_serial);
}


bool CustomAccelStepper::executeTraj2GoalPos_4dur_accel(float Qgoal, float Time, float Accel, float v_con1, Stream &comm_serial) {
    // I. Set the constraints
    _setTrajectoryPosCon(Qgoal);
    _setTrajectoryVelCon(v_con1);

    // II. Set the desired values
    _setTrajectoryTargets(Time, 0, Accel); // Vv = 0, since it will be computed internally!

    // III. Check if trajectory is feasible
    if (_trajectory_existance_check()) {
        // III.a. If true, we change the velocity cons in order to be always feasible
        _setTrajectoryVelCon(0.0f);
    }
    // IV. Extract the generic data for the trajectory to be executed next.
    _extractTrajData_4dur_accel();    
    // V. Raedy to execute the trajectory phases. Here we are sure that 3 phases exist!
    _executeTrajPhases(comm_serial);
}


bool CustomAccelStepper::_run_var_delay(unsigned long steps, unsigned long delay_micros) {
    _net_sd_micros = delay_micros;
    
    _stepVarLength();

    if (_step_cnt > steps )
    {
        return true;  // finished
    } else {
        return false;
    }
    

}

void CustomAccelStepper::_stepVarLength() {
    if (micros() - _last_step_micros > _net_sd_micros)
    {
        _step_cnt++;
        digitalWrite(_stepPin, HIGH); // time to finish the step delay and generate the new pulse
        delayMicroseconds(_min_pulse_width);
        digitalWrite(_stepPin, LOW);
        _last_step_micros = micros(); // starts counting time after the FIX PULSE has ended!

        _update_abs_q_rad();
    }

}

void CustomAccelStepper::_update_abs_q_rad()
{
  _cur_q_rad = _cur_q_rad + (_sigma * _ConvertStep2Rad(_step_cnt));

  return;
}

