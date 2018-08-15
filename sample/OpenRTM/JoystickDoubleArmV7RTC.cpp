/**
   @author Yasuhiro Masutani
*/

#include "JoystickDoubleArmV7RTC.h"
#include <cmath>

using namespace std;
using namespace cnoid;

namespace {

const char* spec[] =
{
    "implementation_id", "JoystickDoubleArmV7RTC",
    "type_name",         "JoystickDoubleArmV7RTC",
    "description",       "Joystick for DoubleArmV7",
    "version",           "1.0",
    "vendor",            "MasutaniLab",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

}


JoystickDoubleArmV7RTC::JoystickDoubleArmV7RTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      velocitiesOut("dq", velocities),
      anglesOut("qt", angles)
{

}


JoystickDoubleArmV7RTC::~JoystickDoubleArmV7RTC()
{

}


RTC::ReturnCode_t JoystickDoubleArmV7RTC::onInitialize()
{
    // Set OutPort buffer
    addOutPort("dq", velocitiesOut);
    addOutPort("q", anglesOut);

    jointParameters = {
        {   0.0, -687.5, 687.5}, //MFRAME
        {   0.0,  -48.0,  69.5}, //BLOCK
        { -70.0,  -75.1,  53.2}, //BOOM
        { 150.0,   30.7, 152.1}, //ARM
        {   0.0,  -33.7,  90.0}, //TOHKU_PITCH
        {   0.0, -180.0, 180.0}, //TOHKU_ROLL
        {   0.0,  -20.0,   0.0}, //TOHKU_TIP_01
        {   0.0,  -20.0,   0.0}, //TOHKU_TIP_02
        {   0.0, -687.5, 687.5}, //UFRAME
        {   0.0,  -60.0,  60.0}, //MNP_SWING
        {   0.0,    0.0, 115.0}, //MANIBOOM
        {   0.0, -110.0,   0.0}, //MANIARM
        {   0.0,  -90.0,  20.0}, //MANIELBOW
        {   0.0,  -50.48, 50.48}, //YAWJOINT
        {   0.0, -270.0, 270.0}, //HANDBASE
        {   0.0,   -0.0507, 0.0}  //PUSHROD 
    };
    for (int i=0; i<jointParameters.size(); i++) {
        jointParameters[i].init *= M_PI/180;
        jointParameters[i].lower *= M_PI/180;
        jointParameters[i].upper *= M_PI/180;
    }

    operationAxes = {
        {
            { MFRAME,       STICK,  Joystick::L_STICK_H_AXIS, -0.6 },
            { BLOCK,        STICK,  Joystick::R_STICK_H_AXIS, -0.6 },
            { BOOM,         STICK,  Joystick::L_STICK_V_AXIS, -0.6 },
            { ARM,          STICK,  Joystick::R_STICK_V_AXIS,  0.6 },
            { TOHKU_PITCH,  BUTTON, Joystick::A_BUTTON,        0.6 },
            { TOHKU_PITCH,  BUTTON, Joystick::Y_BUTTON,       -0.6 },
            { TOHKU_ROLL,   BUTTON, Joystick::X_BUTTON,        1.0 },
            { TOHKU_ROLL,   BUTTON, Joystick::B_BUTTON,       -1.0 },
            { TOHKU_TIP_01, STICK,  Joystick::R_TRIGGER_AXIS, -0.6 },
            { TOHKU_TIP_02, STICK,  Joystick::R_TRIGGER_AXIS, -0.6 },
            { TOHKU_TIP_01, BUTTON, Joystick::R_BUTTON,        0.5 },
            { TOHKU_TIP_02, BUTTON, Joystick::R_BUTTON,        0.5 }
        },
        {
            { UFRAME,       STICK,  Joystick::L_STICK_H_AXIS, -0.6 },
            { MNP_SWING,    STICK,  Joystick::R_STICK_H_AXIS, -0.6 },
            { MANIBOOM,     STICK,  Joystick::L_STICK_V_AXIS, -0.6 },
            { MANIARM,      STICK,  Joystick::R_STICK_V_AXIS,  0.6 },
            { MANIELBOW,    BUTTON, Joystick::A_BUTTON,        0.6 },
            { MANIELBOW,    BUTTON, Joystick::Y_BUTTON,       -0.6 },
            { YAWJOINT,     BUTTON, Joystick::X_BUTTON,        1.0, 1 },
            { YAWJOINT,     BUTTON, Joystick::B_BUTTON,       -1.0, 1 },
            { HANDBASE,     BUTTON, Joystick::X_BUTTON,       -1.0, 0 },
            { HANDBASE,     BUTTON, Joystick::B_BUTTON,        1.0, 0 },
            { PUSHROD,      STICK,  Joystick::R_TRIGGER_AXIS, -0.04 },
            { PUSHROD,      BUTTON, Joystick::R_BUTTON,        0.04 },
        }
    };

    operationSetIndex = 0;
    prevSelectButtonState = false;


    return RTC::RTC_OK;
}


RTC::ReturnCode_t JoystickDoubleArmV7RTC::onActivated(RTC::UniqueId ec_id)
{
    velocities.data.length(2);
    for(int i=0; i < velocities.data.length(); ++i){
        velocities.data[i] = 0.0;
    }
    angles.data.length(16);
    for(int i=0; i < angles.data.length(); ++i){
        angles.data[i] = jointParameters[i].init;
    }

    return RTC::RTC_OK;
}


RTC::ReturnCode_t JoystickDoubleArmV7RTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t JoystickDoubleArmV7RTC::onExecute(RTC::UniqueId ec_id)
{
    joystick.readCurrentState();

    velocities.data[0] = joystick.getPosition(Joystick::DIRECTIONAL_PAD_H_AXIS);
    velocities.data[1] = joystick.getPosition(Joystick::DIRECTIONAL_PAD_V_AXIS);
    velocitiesOut.write();

    shiftState = joystick.getButtonState(SHIFT_BUTTON) ? 1 : 0;

    bool selectButtonState = joystick.getButtonState(SELECT_BUTTON_ID);
    if(!prevSelectButtonState && selectButtonState){
        operationSetIndex = 1 - operationSetIndex;
    }
    prevSelectButtonState = selectButtonState;

    const vector<OperationAxis>& axes = operationAxes[operationSetIndex];

    for(auto& axis : axes){
        if(axis.shift < 0 || axis.shift == shiftState){
            double& q = angles.data[axis.joint];
            if(axis.type == BUTTON){
                if(joystick.getButtonState(axis.id)){
                    q += axis.ratio * dt;
                }
            } else if(axis.type == STICK){
                double pos = joystick.getPosition(axis.id);
                q += axis.ratio * pos * dt;
            }
            JointParameter &p = jointParameters[axis.joint];
            if(q > p.upper){
                q = p.upper;
            } else if(q < p.lower){
                q = p.lower;
            }
        }
    }
    anglesOut.write();

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void JoystickDoubleArmV7RTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile,
                                 RTC::Create<JoystickDoubleArmV7RTC>,
                                 RTC::Delete<JoystickDoubleArmV7RTC>);
    }
};
