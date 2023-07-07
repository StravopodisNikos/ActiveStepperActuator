#include "Arduino.h"
#include <ActiveStepperActuator.h>

using namespace actuator_ns;

/*
 *  ***************      ActiveStepperActuator      ***************
 */

ActiveStepperActuator::ActiveStepperActuator(uint8_t ID, int mode, int step_pin, int dir_pin, int en_pin) : _StepperMotor(mode, step_pin, dir_pin) {
//ActiveStepperActuator::ActiveStepperActuator(uint8_t ID) {
    // User defined based on stepper/unit configured
    _unit_id = ID;
    //_assignUnitID(); // reset unit eeprom if ID has changed

    // Create class objects for the limit switches
    UnitSwitches[0] = new ezButton(HOME_SWITCH_PIN);
    UnitSwitches[1] = new ezButton(LEFT_SWITCH_PIN); 
    UnitSwitches[2] = new ezButton(RIGHT_SWITCH_PIN);


    // Default for current stepper used in Ovidius joint 1 robot
    _stepPin = step_pin;
    _dirPin  = dir_pin;
    _enPin   = en_pin;
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
    double temp = static_cast<double> (0.15915f * RAD); 
    return round(_micro_step * _gear_ratio * temp);  // 0.1592 = 1/6.2832, 6.2832 = 2*π
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
    _StepperMotor.setCurrentPosition(0);
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

void ActiveStepperActuator::_assignVelocityProfile_rads(float vel) {
    _StepperMotor.setSpeed((float) _ConvertRad2Step(vel));
}

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

//bool ActiveStepperActuator::go2GoalPos_vel(AccelStepper *ptr2stepper , float abs_pos, float des_vel, Stream &comm_serial) {
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

void ActiveStepperActuator::ConversionTester(float RAD, long STEPS, Stream &comm_serial) {
    comm_serial.print("RAD 2 STEPS: "); comm_serial.println(_ConvertRad2Step(RAD));
    comm_serial.print("STEPS 2 RAD: "); comm_serial.println(_ConvertStep2Rad(STEPS));
}

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


CustomAccelStepper::CustomAccelStepper(/* args */)
{
}

CustomAccelStepper::~CustomAccelStepper()
{
}

void CustomAccelStepper::_extract_displacement() {
    _h = _q1_hat_rad - _q0_hat_rad;
}

void CustomAccelStepper::_extract_sigma() {
    if ( abs(_q1_hat_rad) > abs(_q0_hat_rad) )
    {
        _sigma = 1.0f;
    }
    else {
        _sigma = -1.0f;
    }
}

bool CustomAccelStepper::_trajectory_existance_check() {
    // Eq. (3.14) p.71 Melchiorri Book
    if ( ( (_Amax_rs2 * _h) - 0.5f * abs( sq(_V0_hat_rs)-sq(_V1_hat_rs)) ) < 0)
    {
        return false;
    } else {
        return true;
    }
}

float CustomAccelStepper::_calculate_Vv_4dur_accel() {
    // Eq. for Vv in p.72 Melchiorri Book
    float __aT  = _Amax_rs2 * _T_s;
    float __4ah = 4.0f * _Amax_rs2 * _h;
    float __2aT = 2.0f * _Amax_rs2 * (_V0_hat_rs + _V1_hat_rs) * _T_s;
    float __dV  = _V0_hat_rs - _V1_hat_rs;
    _Vv_rs = 0.5f * ( _V0_hat_rs + _V1_hat_rs + __aT - sqrt( sq(__aT)  -  __4ah  +  __2aT  - sq(__dV) ) );
    return _Vv_rs;
}
