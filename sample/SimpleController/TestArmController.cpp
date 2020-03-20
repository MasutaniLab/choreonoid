#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

class TestArmController : public cnoid::SimpleController
{
public:
    Body* body;
    double dt;

    Link::ActuationMode mainActuationMode;

    vector<int> armJointIdMap;
    vector<Link*> armJoints;
    vector<double> q_ref;
    vector<double> q_prev;
    double* q_tip1;
    double* q_tip2;

    SharedJoystickPtr joystick;
    int arm1Mode;
    int arm2Mode;
    int currentJoystickMode;
    const int SHIFT_BUTTON = Joystick::L_BUTTON;
    int shiftState;

    enum AxisType { STICK, BUTTON };

    struct OperationAxis {
        Link* joint;
        AxisType type;
        int id;
        double ratio;
        int shift;
        OperationAxis(Link* joint, AxisType type, int id, double ratio, int shift = 0)
            : joint(joint), type(type), id(id), ratio(ratio), shift(shift) { }
    };

    vector<vector<OperationAxis>> operationAxes;
    int operationSetIndex;

    TestArmController();
    virtual bool initialize(SimpleControllerIO* io) override;
    void initArms(SimpleControllerIO* io);
    void initJoystickKeyBind();
    virtual bool control() override;
    void setTargetArmPositions();

    Link* link(const char* name) { return body->link(name); }
};


TestArmController::TestArmController()
{
    mainActuationMode = Link::ActuationMode::JOINT_DISPLACEMENT;
}

bool TestArmController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    initArms(io);

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    arm1Mode = joystick->addMode();
    arm2Mode = joystick->addMode();
    initJoystickKeyBind();

    return true;
}

void TestArmController::initArms(SimpleControllerIO* io)
{
    for(auto joint : body->joints()){
        if(joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);
            armJointIdMap.push_back(armJoints.size());
            armJoints.push_back(joint);
            q_ref.push_back(joint->q());
        } else {
            armJointIdMap.push_back(-1);
        }
    }
    q_prev = q_ref;
    
    q_tip1 = &q_ref[armJointIdMap[link("TOHKU_TIP_01")->jointId()]];
    q_tip2 = &q_ref[armJointIdMap[link("TOHKU_TIP_02")->jointId()]];
}

void TestArmController::initJoystickKeyBind()
{
    operationAxes = {
        {
            { link("MFRAME"),       STICK,  Joystick::L_STICK_H_AXIS, -0.6 },
            { link("BLOCK"),        STICK,  Joystick::R_STICK_H_AXIS, -0.6 },
            { link("BOOM"),         STICK,  Joystick::L_STICK_V_AXIS, -0.6 },
            { link("ARM"),          STICK,  Joystick::R_STICK_V_AXIS,  0.6 },
            { link("TOHKU_PITCH"),  BUTTON, Joystick::A_BUTTON,        0.6 },
            { link("TOHKU_PITCH"),  BUTTON, Joystick::Y_BUTTON,       -0.6 },
            { link("TOHKU_ROLL"),   BUTTON, Joystick::X_BUTTON,        1.0 },
            { link("TOHKU_ROLL"),   BUTTON, Joystick::B_BUTTON,       -1.0 },
            { link("TOHKU_TIP_01"), STICK,  Joystick::R_TRIGGER_AXIS, -0.6 },
            { link("TOHKU_TIP_02"), STICK,  Joystick::R_TRIGGER_AXIS, -0.6 },
            { link("TOHKU_TIP_01"), BUTTON, Joystick::R_BUTTON,        0.5 },
            { link("TOHKU_TIP_02"), BUTTON, Joystick::R_BUTTON,        0.5 }
        },
        {
            { link("UFRAME"),       STICK,  Joystick::L_STICK_H_AXIS, -0.6 },
            { link("MNP_SWING"),    STICK,  Joystick::R_STICK_H_AXIS, -0.6 },
            { link("MANIBOOM"),     STICK,  Joystick::L_STICK_V_AXIS, -0.6 },
            { link("MANIARM"),      STICK,  Joystick::R_STICK_V_AXIS,  0.6 },
            { link("MANIELBOW"),    BUTTON, Joystick::A_BUTTON,        0.6 },
            { link("MANIELBOW"),    BUTTON, Joystick::Y_BUTTON,       -0.6 },
            { link("YAWJOINT"),     BUTTON, Joystick::X_BUTTON,        1.0, 1 },
            { link("YAWJOINT"),     BUTTON, Joystick::B_BUTTON,       -1.0, 1 },
            { link("HANDBASE"),     BUTTON, Joystick::X_BUTTON,       -1.0, 0 },
            { link("HANDBASE"),     BUTTON, Joystick::B_BUTTON,        1.0, 0 },
            { link("PUSHROD"),      STICK,  Joystick::R_TRIGGER_AXIS, -0.04 },
            { link("PUSHROD"),      BUTTON, Joystick::R_BUTTON,        0.04 },
        }
    };

    operationSetIndex = 0;
}


bool TestArmController::control()
{
    joystick->updateState(arm1Mode);

    if(joystick->mode() == arm1Mode){
        currentJoystickMode = arm1Mode;
        operationSetIndex = 0;
    } else if(joystick->mode() == arm2Mode){
        currentJoystickMode = arm2Mode;
        operationSetIndex = 1;
    } else {
        currentJoystickMode = -1;
    }

    shiftState = joystick->getButtonState(SHIFT_BUTTON) ? 1 : 0;

    setTargetArmPositions();

    for (size_t i = 0; i < armJoints.size(); ++i) {
      armJoints[i]->q_target() = q_ref[i];
    }

    return true;
}


void TestArmController::setTargetArmPositions()
{
    const vector<OperationAxis>& axes = operationAxes[operationSetIndex];

    for(auto& axis : axes){
        if(axis.shift < 0 || axis.shift == shiftState){
            auto joint = axis.joint;
            auto& q = q_ref[armJointIdMap[joint->jointId()]];
            if(axis.type == BUTTON){
                if(joystick->getButtonState(currentJoystickMode, axis.id)){
                    q += axis.ratio * dt;
                }
            } else if(axis.type == STICK){
                auto pos = joystick->getPosition(currentJoystickMode, axis.id);
                q += axis.ratio * pos * dt;
            }
        }
    }

    // Restrict each target position by taking the joint displacement range
    // and the cunnret joint displacement into accout
    double maxerror = radian(5.0);
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto& q = q_ref[i];
        auto q_current = joint->q();
        auto q_lower = std::max(q_current - maxerror, joint->q_lower());
        auto q_upper = std::min(q_current + maxerror, joint->q_upper());
        if(q < q_lower){
            q = q_lower;
        } else if(q > q_upper){
            q = q_upper;
        }
    }

    // Align the positions of the tip joints
    (*q_tip1) = (*q_tip2) = std::max(*q_tip1, *q_tip2);
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TestArmController)
