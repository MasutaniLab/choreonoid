/**
   @author Yasuhiro Masutani
*/

#ifndef JoystickDoubleArmV7RTC_H
#define JoystickDoubleArmV7RTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <cnoid/Joystick>
#include <cmath>

class JoystickDoubleArmV7RTC : public RTC::DataFlowComponentBase
{
public:
    JoystickDoubleArmV7RTC(RTC::Manager* manager);
    ~JoystickDoubleArmV7RTC();
    
    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    
protected:
    // DataOutPort declaration
    RTC::TimedDoubleSeq velocities;
    RTC::OutPort<RTC::TimedDoubleSeq> velocitiesOut;

    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;
  
private:
    enum AxisType { STICK, BUTTON };
    enum Joint { MFRAME = 0, BLOCK, BOOM, ARM, TOHKU_PITCH, TOHKU_ROLL, TOHKU_TIP_01, TOHKU_TIP_02, UFRAME, MNP_SWING, 
        MANIBOOM, MANIARM, MANIELBOW, YAWJOINT, HANDBASE, PUSHROD };
    
    struct JointParameter {
        double init;
        double lower;
        double upper;
        JointParameter(double init, double lower, double upper)
        : init(init), lower(lower), upper(upper) {}
    };

    struct OperationAxis {
        Joint joint;
        AxisType type;
        int id;
        double ratio;
        int shift;
        OperationAxis(Joint joint, AxisType type, int id, double ratio, int shift = 0)
            : joint(joint), type(type), id(id), ratio(ratio), shift(shift) { }
    };

    cnoid::Joystick joystick;
    std::vector<JointParameter> jointParameters;
    std::vector<std::vector<OperationAxis>> operationAxes;
    int operationSetIndex;
    bool prevSelectButtonState;

    const int SHIFT_BUTTON = cnoid::Joystick::L_BUTTON;
    int shiftState;
    
    const int SELECT_BUTTON_ID = cnoid::Joystick::LOGO_BUTTON;

    const double dt = 0.001;
};

extern "C"
{
    DLL_EXPORT void JoystickDoubleArmV7RTCInit(RTC::Manager* manager);
};

#endif
