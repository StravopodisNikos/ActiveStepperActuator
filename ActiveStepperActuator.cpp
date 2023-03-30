#include "Arduino.h"
#include <ActiveStepperActuator.h>

using namespace actuator_comm;

ActiveStepperActuatorTalker::ActiveStepperActuatorTalker(unsigned char ss_pin, unsigned char des_options)
{
    _SS_PIN_STEPPER = ss_pin;
    _SPI_OPT = des_options;
}

ActiveStepperActuatorTalker::~ActiveStepperActuatorTalker() {}

template <typename T>
void ActiveStepperActuatorTalker::send1time_single(T & data_send, T & data_received)
{
    SPI.beginTransaction(*spi_setting_master);
    digitalWrite(_SS_PIN_STEPPER, LOW);
    data_received = SPI.transfer(data_send, sizeof(data_send));   
    digitalWrite(_SS_PIN_STEPPER, HIGH); 
    SPI.endTransaction();
    return;
}
template void ActiveStepperActuatorTalker::send1time_single<float&>(float&, float&);
template void ActiveStepperActuatorTalker::send1time_single<long&>(long&, long&);
template void ActiveStepperActuatorTalker::send1time_single<unsigned char&>(unsigned char&,unsigned char&);

template <typename D>
void ActiveStepperActuatorTalker::_setDataType(const unsigned char CMD_ID, D & data)
{
    switch (CMD_ID)
    {
        // READ COMMANDS
        case READPOS:
            {
                data = READ_THE_RESPONSE_ONLY_L;  
                break;  
            }  
        case READVEL:
            {
                data = READ_THE_RESPONSE_ONLY_L;
                break;
            }
        case READCUR:
            {
                data = READ_THE_RESPONSE_ONLY_F;
                break; 
            }    
        case READTOR:
            {
                data = READ_THE_RESPONSE_ONLY_F;
                break; 
            }    
        //   ASSIGN COMMANDS
        case ASSPOS:
            {
                data = READ_THE_RESPONSE_ONLY_L;  
                break;                      
            }   
        case ASSVEL:
            {
                data = READ_THE_RESPONSE_ONLY_F;
                break; 
            }  
        case ASSACCEL:
            {
                data = READ_THE_RESPONSE_ONLY_F;
                break; 
            }                
        default:
            {
                _act_error = ACT_WRONG_CMD_ID;
                break;
            }
    }
    return;
}
template void ActiveStepperActuatorTalker::_setDataType<float&>(const unsigned char READCMD_ID,float&);
template void ActiveStepperActuatorTalker::_setDataType<long&>(const unsigned char READCMD_ID,long&);
template void ActiveStepperActuatorTalker::_setDataType<unsigned char&>(const unsigned char READCMD_ID,unsigned char&);

void ActiveStepperActuatorTalker::_setSPISettings()
{
    switch (_SPI_OPT)
    {
        case SPI_OPT0:
            {
                SPISettings setting0(MAX_SPI_SPEED_DUE, MSBFIRST, SPI_MODE0);
                spi_setting_master = &setting0;
                break;
            }
        case SPI_OPT1:
            {
                SPISettings setting1(MAX_SPI_SPEED_DUE, MSBFIRST, SPI_MODE1);
                spi_setting_master = &setting1;
                break;
            }
        case SPI_OPT2:
            {
                SPISettings setting2(MAX_SPI_SPEED_DUE, MSBFIRST, SPI_MODE2);
                spi_setting_master = &setting2;
                break;
            }
        case SPI_OPT3:
            {
                SPISettings setting3(MAX_SPI_SPEED_DUE, MSBFIRST, SPI_MODE3);
                spi_setting_master = &setting3;
                break;    
            }                
        default:
            {
                break;
            }
    }

    return;
}

bool ActiveStepperActuatorTalker::_isValidState(const unsigned char state_received)
{
    if ( (state_received == DIS) || (state_received == READY) || (state_received == SETGOAL) || (state_received == ASSIGN) || (state_received == EXEC) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ActiveStepperActuatorTalker::SetupMaster()
{
    pinMode(_SS_PIN_STEPPER, OUTPUT);
    digitalWrite(_SS_PIN_STEPPER, HIGH);
    SPI.begin(_SS_PIN_STEPPER);
    _setSPISettings();

    return;
}

void ActiveStepperActuatorTalker::Connect2Slave()
{
    send1time_single( (unsigned char&) PING, _RESPONSE); // Must cast to the same data type, because 2 arguments of the same data type are allowed! _RESPONSE is defined, in L48 @ .h as, unsigned char because MUST change

    return;
}

void ActiveStepperActuatorTalker::ReadSlaveState()
{
    _terminate_condition = false;
    _t_start_micros = micros();
    do
    {
        send1time_single((unsigned char&) READSTATE, _RESPONSE); // Must cast to the same data type, because 2 arguments of the same data type are allowed! _RESPONSE is defined, in L48 @ .h as, unsigned char because MUST change

        _fn_return = _isValidState(_RESPONSE);

        if (_fn_return)
        {
            _last_slave_state = _RESPONSE;
            _terminate_condition = true;
        }

        if ( (micros() - _t_start_micros) > MAX_TIMEOUT_MICROS_100 )
        {
            _terminate_condition = true;
        }
            
    } while (!_terminate_condition);
    return;
}

template <typename R>
void ActiveStepperActuatorTalker::ReadSlaveMeasurement(const unsigned char READCMD_ID, R & meas_type_request ,R & measurement)
{
    
    _terminate_condition = false;

    _t_start_micros = micros();
    do
    {   
        _setDataType(READCMD_ID, meas_type_request);
        send1time_single( meas_type_request,  measurement);  
        if ( (micros() - _t_start_micros) > MAX_TIMEOUT_MICROS_100 )
        {
            _terminate_condition = true;
        }
    } while (!_terminate_condition);
    return;
}  
template void ActiveStepperActuatorTalker::ReadSlaveMeasurement<float&>(const unsigned char READCMD_ID, float&, float&);
template void ActiveStepperActuatorTalker::ReadSlaveMeasurement<long&>(const unsigned char READCMD_ID, long&, long&);
template void ActiveStepperActuatorTalker::ReadSlaveMeasurement<unsigned char&>(const unsigned char READCMD_ID, unsigned char&, unsigned char&);

template <typename S>
void ActiveStepperActuatorTalker::AssignSlaveProfile(const unsigned char ASSIGNCMD_ID, S & ass_type_request , S & assignment )
{
    
    _terminate_condition = false;

    _t_start_micros = micros();
    do
    {   
        _setDataType(ASSIGNCMD_ID, ass_type_request);
        send1time_single( ass_type_request,  assignment);  
        if ( (micros() - _t_start_micros) > MAX_TIMEOUT_MICROS_100 )
        {
            _terminate_condition = true;
        }
    } while (!_terminate_condition);
    return;
}  
template void ActiveStepperActuatorTalker::AssignSlaveProfile<float&>(const unsigned char READCMD_ID, float&, float&);
template void ActiveStepperActuatorTalker::AssignSlaveProfile<long&>(const unsigned char READCMD_ID, long&, long&);

void ActiveStepperActuatorTalker::send1time_packet(Data_packet packet2send)
{
    Data_packet_union packetUnion;
    packetUnion.message = packet2send;

    digitalWrite(_SS_PIN_STEPPER, LOW);
    for (size_t i = 0; i < sizeof(data_packet); i++)
    {
        SPI.transfer(packetUnion.bytes[i]);
    }
    digitalWrite(_SS_PIN_STEPPER, HIGH);
    return;
}

bool ActiveStepperActuatorTalker::setActuatorGoalPosition()
{

}

ActiveStepperActuatorListener::ActiveStepperActuatorListener(/* args */)
{
}

ActiveStepperActuatorListener::~ActiveStepperActuatorListener()
{
}

void ActiveStepperActuatorListener::SetupSlave()
{
    /*
     *  Initializes the state to DIS(no connection established yet)
     */
    _cur_state_listener = DIS;
}