/**
   @author Yasuhiro Masutani
*/

#include <cnoid/BodyIoRTC>
#include <cnoid/Light>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace std;
using namespace cnoid;

namespace {

class DoubleArmV7IoRTC : public BodyIoRTC
{
public:
    DoubleArmV7IoRTC(RTC::Manager* manager);
    ~DoubleArmV7IoRTC(); 

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    BodyPtr ioBody;

    Link* trackL;
    Link* trackR;
    
    // DataInPort declaration
    RTC::TimedDoubleSeq torques;
    RTC::InPort<RTC::TimedDoubleSeq> torquesIn;
 
    RTC::TimedDoubleSeq velocities;
    RTC::InPort<RTC::TimedDoubleSeq> velocitiesIn;
  
    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;
};

const char* spec[] =
{
    "implementation_id", "DoubleArmV7IoRTC",
    "type_name",         "DoubleArmV7IoRTC",
    "description",       "DoubleArmV7 I/O",
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

DoubleArmV7IoRTC::DoubleArmV7IoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      torquesIn("u", torques),
      velocitiesIn("dq", velocities),
      anglesOut("q", angles)
{

}


DoubleArmV7IoRTC::~DoubleArmV7IoRTC()
{

}


bool DoubleArmV7IoRTC::initializeIO(ControllerIO* io)
{
    ioBody = io->body();

    // Set InPort buffers
    addInPort("u", torquesIn);
    addInPort("dq", velocitiesIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);
    int c = 0;
    for(auto joint : ioBody->joints()){
        if(joint->jointId() >= 0 &&
             (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            c++;
        }
    }
    angles.data.length(c);

    return true;
}


bool DoubleArmV7IoRTC::initializeSimulation(ControllerIO* io)
{

    trackL = ioBody->link("TRACK_L");
    trackR = ioBody->link("TRACK_R");
    trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
    trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);

    for(auto joint : ioBody->joints()){
        if(joint->jointId() >= 2 &&
             (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            joint->setActuationMode(Link::JOINT_TORQUE);
        }
    }

    return true;
}


void DoubleArmV7IoRTC::inputFromSimulator()
{
    for(auto joint : ioBody->joints()){
        int index = joint->jointId();
        if (index >= 2) {
            angles.data[index-2] = joint->q();
        }
    }
    anglesOut.write();
}


void DoubleArmV7IoRTC::outputToSimulator()
{
    if(velocitiesIn.isNew()){
        velocitiesIn.read();
        if(velocities.data.length() >= 2){
            trackL->dq() = velocities.data[0];
            trackR->dq() = velocities.data[1];
        }
    }

    if(torquesIn.isNew()){
        torquesIn.read();
        int len = torques.data.length();
        for(auto joint : ioBody->joints()){
            int index = joint->jointId();
            if (2 <= index && index < len-2) {
                joint->u() = torques.data[index-2];
            }
        }
    }
}


extern "C"
{
    DLL_EXPORT void DoubleArmV7IoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<DoubleArmV7IoRTC>, RTC::Delete<DoubleArmV7IoRTC>);
    }
};
