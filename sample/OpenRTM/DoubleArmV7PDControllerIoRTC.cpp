/**
   @author Yasuhiro Masutani
*/

#include <cnoid/BodyIoRTC>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/Light>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/EigenUtil>
#include <sstream>

using namespace std;
using namespace cnoid;

namespace {

class DoubleArmV7PDControllerIoRTC : public BodyIoRTC
{
    Body* body;
    double dt;

    Link::ActuationMode mainActuationMode;

    enum TrackType { NO_TRACKS = 0, CONTINOUS_TRACKS, PSEUDO_TRACKS };
    int trackType;
    Link* trackL;
    Link* trackR;
    double trackgain;

    vector<int> armJointIdMap;
    vector<Link*> armJoints;
    vector<double> q_ref;
    vector<double> q_prev;
    vector<double> pgain;
    vector<double> dgain;

    ControllerIO* cio;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

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
    void controlTracks();
    void setTargetArmPositions();
    void controlArms();
    void controlArmsWithTorque();
    void controlArmsWithVelocity();
    void controlArmsWithPosition();

    Link* link(const char* name) { return body->link(name); }
    
    // DataInPort declaration
    RTC::TimedDoubleSeq anglesTarget;
    RTC::InPort<RTC::TimedDoubleSeq> anglesTargetIn;
 
    RTC::TimedDoubleSeq velocities;
    RTC::InPort<RTC::TimedDoubleSeq> velocitiesIn;
  
    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;

    RTC::Acceleration3D accel;
    RTC::OutPort<RTC::Acceleration3D> accelOut;

    RTC::AngularVelocity3D angularVelocity;
    RTC::OutPort<RTC::AngularVelocity3D> angularVelocityOut;
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
      anglesOut("q", angles),
      accelOut("dv", accel),
      angularVelocityOut("w", angularVelocity)
{
    mainActuationMode = Link::ActuationMode::JOINT_TORQUE;
    trackType = NO_TRACKS;
}


DoubleArmV7PDControllerIoRTC::~DoubleArmV7PDControllerIoRTC()
{

}


bool DoubleArmV7PDControllerIoRTC::initializeIO(ControllerIO* io)
{
    io->os() << "DoubleArmV7PDControllerIoRTC::initializeIO()" << endl;
    // Set InPort buffers
    addInPort("qt", anglesTargetIn);
    addInPort("dq", velocitiesIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);
    addOutPort("dv", accelOut);
    addOutPort("w", angularVelocityOut);

    return true;
}


bool DoubleArmV7PDControllerIoRTC::initializeSimulation(ControllerIO* io)
{
    cio = io;
    io->os() << "DoubleArmV7PDControllerIoRTC::initializeSimulation()" << endl;
    body = io->body();
    dt = io->timeStep();

    string option = io->optionString();
    io->os() << "Controller option: " << option << endl;
    istringstream is(option);
    string mode = "";
    while (!is.eof()) {
        string tmp;
        is >> tmp;
        if (tmp == "position" || tmp == "velocity" || tmp == "torque") {
            mode = tmp;
        }
    }
    io->os() << "The actuation mode is ";
    if(mode == "velocity"){
        mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
        io->os() << "JOINT_VELOCITY";
    } else if(mode  == "position"){
        mainActuationMode = Link::ActuationMode::JOINT_DISPLACEMENT;
        io->os() << "JOINT_DISPLACEMENT";
    } else {
        mainActuationMode = Link::ActuationMode::JOINT_EFFORT;
        io->os() << "JOINT_EFFORT";
    }
    io->os() << "." << endl;

    if (initPseudoContinuousTracks(io) == false) {
        initContinuousTracks(io);
    }
        
    initArms(io);
    initPDGain();

    accelSensor = body->findDevice<AccelerationSensor>("ACCEL_SENSOR");
    if (accelSensor == nullptr) {
        io->os() << "加速度センサが見つかりません" << endl;
        return false;
    }
    gyro = body->findDevice<RateGyroSensor>("FRAME_GYRO");
    if (gyro == nullptr) {
        io->os() << "ジャイロセンサが見つかりません" << endl;
        return false;
    }

    return true;
}

bool DoubleArmV7PDControllerIoRTC::initContinuousTracks(ControllerIO* io)
{
    trackL = link("WHEEL_L0");
    trackR = link("WHEEL_R0");

    if(!trackL || !trackR){
        return false;
    }

    if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
        trackL->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
        trackR->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
    } else {
        trackL->setActuationMode(Link::ActuationMode::JOINT_VELOCITY);
        trackR->setActuationMode(Link::ActuationMode::JOINT_VELOCITY);
    }
    
    trackType = CONTINOUS_TRACKS;

    io->os() << "Continuous tracks of " << body->name() << " are detected." << endl;

    return true;
}

bool DoubleArmV7PDControllerIoRTC::initPseudoContinuousTracks(ControllerIO* io)
{
    trackL = link("TRACK_L");
    trackR = link("TRACK_R");

    if(!trackL || !trackR){
        return false;
    }

    if(trackL->actuationMode() == Link::JOINT_SURFACE_VELOCITY 
        && trackR->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        trackType = PSEUDO_TRACKS;
        io->os() << "Pseudo continuous tracks of " << body->name() << " are detected." << endl;
    }

    return (trackType == PSEUDO_TRACKS);
}


void DoubleArmV7PDControllerIoRTC::initArms(ControllerIO* io)
{
    armJointIdMap.clear();
    armJoints.clear();
    q_ref.clear();
    for(auto joint : body->joints()){
        if(joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            joint->setActuationMode(mainActuationMode);
            armJointIdMap.push_back(armJoints.size());
            armJoints.push_back(joint);
            q_ref.push_back(joint->q());
        } else {
            armJointIdMap.push_back(-1);
        }
    }
    q_prev = q_ref;
    angles.data.length(q_ref.size());
}

void DoubleArmV7PDControllerIoRTC::initPDGain()
{
    // Tracks
    if(trackType == CONTINOUS_TRACKS){
        if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
            trackgain = 2000.0;
        } else {
            trackgain = 2.0;
        }
    } else if(trackType == PSEUDO_TRACKS){
        trackgain = 1.0;
    }

    // Arm
    if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
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

    } else if(mainActuationMode == Link::ActuationMode::JOINT_VELOCITY){
        pgain = {
        /* MFRAME */ 100, /* BLOCK */ 100, /* BOOM */ 100, /* ARM  */ 100,
        /* PITCH  */  50, /* ROLL  */  50, /* TIP1 */   5, /* TIP2 */   5,
        /* UFRAME */ 100, /* SWING */ 100, /* BOOM */ 100, /* ARM  */ 100,
        /* ELBOW */   50, /* YAW   */  20, /* HAND */  20, /* ROD  */  50};
    }
}


void DoubleArmV7PDControllerIoRTC::inputFromSimulator()
{
    controlArms();

    for(size_t i=0; i < armJoints.size(); ++i){
        Link* joint = armJoints[i];
        angles.data[i] = joint->q();;
    }
    anglesOut.write();
    
    auto dv = accelSensor->dv();
    accel.ax = dv.x();
    accel.ay = dv.y();
    accel.az = dv.z();
    accelOut.write();

    auto w = gyro->w();
    angularVelocity.avx = w.x();
    angularVelocity.avy = w.y();
    angularVelocity.avz = w.z();
    angularVelocityOut.write();
}

void DoubleArmV7PDControllerIoRTC::outputToSimulator()
{
    if (velocitiesIn.isNew()) {
        velocitiesIn.read();
        if (velocities.data.length() >= 2) {
            controlTracks();
        }
    }

    if(anglesTargetIn.isNew()){
        anglesTargetIn.read();
        setTargetArmPositions();
    }
}

void DoubleArmV7PDControllerIoRTC::controlTracks()
{
    trackL->u() = 0.0;
    trackL->dq_target() = 0.0;
    trackR->u() = 0.0;
    trackR->dq_target() = 0.0;
    if(trackType == CONTINOUS_TRACKS 
        && mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
        trackL->u() = trackgain * velocities.data[0];
        trackR->u() = trackgain * velocities.data[1];
    } else {
        trackL->dq_target() = trackgain * velocities.data[0];
        trackR->dq_target() = trackgain * velocities.data[1];
    }
}
void DoubleArmV7PDControllerIoRTC::setTargetArmPositions()
{
    static const double maxerror = radian(3.0);
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto& q = q_ref[i];
        q = anglesTarget.data[i];
#if 0
        auto q_current = joint->q();
        auto q_lower = std::max(q_current - maxerror, joint->q_lower());
        auto q_upper = std::min(q_current + maxerror, joint->q_upper());
#else
        auto q_lower = joint->q_lower();
        auto q_upper = joint->q_upper();
#endif
        if(q < q_lower){
            q = q_lower;
        } else if(q > q_upper){
            q = q_upper;
        }
    }
}

void DoubleArmV7PDControllerIoRTC::controlArms()
{
    switch(mainActuationMode){
    case Link::ActuationMode::JOINT_DISPLACEMENT:
        controlArmsWithPosition();
        break;
    case Link::ActuationMode::JOINT_VELOCITY:
        controlArmsWithVelocity();
        break;
    case Link::ActuationMode::JOINT_EFFORT:
        controlArmsWithTorque();
        break;
    default:
        break;
    }
}

void DoubleArmV7PDControllerIoRTC::controlArmsWithPosition()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        armJoints[i]->q_target() = q_ref[i];
    }
}

void DoubleArmV7PDControllerIoRTC::controlArmsWithVelocity()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto q_current = joint->q();
        joint->dq_target() = pgain[i] * (q_ref[i] - q_current);
    }
}

void DoubleArmV7PDControllerIoRTC::controlArmsWithTorque()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto q_current = joint->q();
        auto dq_current = (q_current - q_prev[i]) / dt;
        joint->u() = pgain[i] * (q_ref[i] - q_current) + dgain[i] * (0.0 - dq_current);
        q_prev[i] = q_current;
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