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

class DoubleArmV7PDControllerIoRTC : public BodyIoRTC
{
    Body* body;
    double dt;

    vector<int> armJointIdMap;
    vector<Link*> armJoints;
    vector<double> qref;
    vector<double> qold;
    vector<double> pgain;
    vector<double> dgain;
    double trackgain;

    Link* trackL;
    Link* trackR;
    bool hasPseudoContinuousTracks;
    bool hasContinuousTracks;

public:
    DoubleArmV7PDControllerIoRTC(RTC::Manager* manager);
    ~DoubleArmV7PDControllerIoRTC(); 

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    bool initPseudoContinuousTracks(ControllerIO* io);
    bool initContinuousTracks(ControllerIO* io);
    void initArms(ControllerIO* io);
    void initPDGain();
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    
    // DataInPort declaration
    RTC::TimedDoubleSeq anglesTarget;
    RTC::InPort<RTC::TimedDoubleSeq> anglesTargetIn;
 
    RTC::TimedDoubleSeq velocities;
    RTC::InPort<RTC::TimedDoubleSeq> velocitiesIn;
  
    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;
};

const char* spec[] =
{
    "implementation_id", "DoubleArmV7PDControllerIoRTC",
    "type_name",         "DoubleArmV7PDControllerIoRTC",
    "description",       "DoubleArmV7 with PD Controller I/O",
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

DoubleArmV7PDControllerIoRTC::DoubleArmV7PDControllerIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      velocitiesIn("dq", velocities),
      anglesTargetIn("qt", anglesTarget),
      anglesOut("q", angles)
{
    hasPseudoContinuousTracks = false;
    hasContinuousTracks = false;
}


DoubleArmV7PDControllerIoRTC::~DoubleArmV7PDControllerIoRTC()
{

}


bool DoubleArmV7PDControllerIoRTC::initializeIO(ControllerIO* io)
{

    // Set InPort buffers
    addInPort("qt", anglesTargetIn);
    addInPort("dq", velocitiesIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);

    return true;
}


bool DoubleArmV7PDControllerIoRTC::initializeSimulation(ControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    if(!initPseudoContinuousTracks(io))
        initContinuousTracks(io);
    initArms(io);
    initPDGain();

    return true;
}

bool DoubleArmV7PDControllerIoRTC::initPseudoContinuousTracks(ControllerIO* io)
{
    trackL = body->link("TRACK_L");
    trackR = body->link("TRACK_R");
    if(!trackL) return false;
    if(!trackR) return false;

    if(trackL->actuationMode() == Link::JOINT_SURFACE_VELOCITY &&
    trackR->actuationMode() == Link::JOINT_SURFACE_VELOCITY   ){
        hasPseudoContinuousTracks = true;
        return true;
    }
    return false;
}

bool DoubleArmV7PDControllerIoRTC::initContinuousTracks(ControllerIO* io)
{
    trackL = body->link("WHEEL_L0");
    trackR = body->link("WHEEL_R0");
    if(!trackL) return false;
    if(!trackR) return false;

    trackL->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
    trackR->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
    
    hasContinuousTracks = true;
    return true;
}

void DoubleArmV7PDControllerIoRTC::initArms(ControllerIO* io)
{
    for(auto joint : body->joints()){
        if(joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            joint->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
            armJointIdMap.push_back(armJoints.size());
            armJoints.push_back(joint);
            qref.push_back(joint->q());
        } else {
            armJointIdMap.push_back(-1);
        }
    }
    qold = qref;
    angles.data.length(qref.size());
}

void DoubleArmV7PDControllerIoRTC::initPDGain()
{
    // Tracks
    if(hasPseudoContinuousTracks) {
        trackgain = 1.0;
    } 
    if(hasContinuousTracks){
        trackgain = 2000.0;
    }

    // Arm
    pgain = {
        /* MFRAME */ 200000, /* BLOCK */ 150000, /* BOOM */ 150000, /* ARM  */ 100000,
        /* PITCH  */  30000, /* ROLL  */  20000, /* TIP1 */    500, /* TIP2 */    500,
        /* UFRAME */ 150000, /* SWING */  50000, /* BOOM */ 100000, /* ARM  */  80000,
        /* ELBOW */   30000, /* YAW   */  20000, /* HAND */    500, /* ROD  */  50000};
    dgain = {
        /* MFRAME */ 20000, /* BLOCK */ 15000, /* BOOM */ 10000, /* ARM  */ 5000,
        /* PITCH  */   500, /* ROLL  */   500, /* TIP1 */    50, /* TIP2 */   50,
        /* UFRAME */ 15000, /* SWING */  1000, /* BOOM */  3000, /* ARM  */ 2000,
        /* ELBOW */    500, /* YAW   */   500, /* HAND */    20, /* ROD  */ 5000};
}


void DoubleArmV7PDControllerIoRTC::inputFromSimulator()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        Link* joint = armJoints[i];
        double q = joint->q();
        double dq = (q - qold[i]) / dt;
        joint->u() = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
        qold[i] = q;
        angles.data[i] = q;
    }
    anglesOut.write();
}

void DoubleArmV7PDControllerIoRTC::outputToSimulator()
{
    if (velocitiesIn.isNew()) {
        velocitiesIn.read();
        if (velocities.data.length() >= 2) {
            if (hasPseudoContinuousTracks) {
                trackL->dq() = trackgain * velocities.data[0];
                trackR->dq() = trackgain * velocities.data[1];
            } else {
                trackL->u() = trackgain * velocities.data[0];
                trackR->u() = trackgain * velocities.data[1];
            }
        }
    }

    if(anglesTargetIn.isNew()){
        anglesTargetIn.read();
        size_t len = anglesTarget.data.length();
        for(size_t i=0; i < len; ++i){
            qref[i] = anglesTarget.data[i];
        }
    }
}

extern "C"
{
    DLL_EXPORT void DoubleArmV7PDControllerIoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<DoubleArmV7PDControllerIoRTC>, RTC::Delete<DoubleArmV7PDControllerIoRTC>);
    }
};