/**
   @author Yasuhiro Masutani
*/

#include "DoubleArmV7JoystickControllerRTC.h"
#include <cmath>

using namespace std;

namespace {

const char* spec[] =
{
    "implementation_id", "DoubleArmV7JoystickControllerRTC",
    "type_name",         "DoubleArmV7JoystickControllerRTC",
    "description",       "DoubleArmV7 Joystick Controller ",
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


DoubleArmV7JoystickControllerRTC::DoubleArmV7JoystickControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      axesIn("axes", axes),
      buttonsIn("buttons", buttons),
      velocitiesOut("dq", velocities),
      anglesOut("q", angles)
{

}


DoubleArmV7JoystickControllerRTC::~DoubleArmV7JoystickControllerRTC()
{

}


RTC::ReturnCode_t DoubleArmV7JoystickControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("axes", axesIn);
    addInPort("buttons", buttonsIn);
    
    // Set OutPort buffer
    addOutPort("dq", velocitiesOut);
    addOutPort("q", anglesOut);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t DoubleArmV7JoystickControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    velocities.data.length(2);
    for(int i=0; i < velocities.data.length(); ++i){
        velocities.data[i] = 0.0;
    }
    angles.data.length(16);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t DoubleArmV7JoystickControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t DoubleArmV7JoystickControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if (axesIn.isNew() && buttonsIn.isNew()) {
        axesIn.read();
        buttonsIn.read();

        velocitiesOut.write();
        anglesOut.write();
    }

    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void DoubleArmV7JoystickControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(profile,
                                 RTC::Create<DoubleArmV7JoystickControllerRTC>,
                                 RTC::Delete<DoubleArmV7JoystickControllerRTC>);
    }
};
