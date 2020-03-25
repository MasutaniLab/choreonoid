#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

class TestArmController : public cnoid::SimpleController
{
public:
    Body* body;
    double dt;

    Link::ActuationMode mainActuationMode;

    vector<Link*> armJoints;
    vector<double> q_ref;

    SharedJoystickPtr joystick;

    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
};

bool TestArmController::initialize(SimpleControllerIO* io)
{
    mainActuationMode = Link::ActuationMode::JOINT_DISPLACEMENT;
    body = io->body();
    dt = io->timeStep();
    armJoints.clear();
    for (auto joint : body->joints()) {
        if (joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())) {
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);
            armJoints.push_back(joint);
            q_ref.push_back(joint->q());
        }
    }
    io->os() << "armJoints.size(): " << armJoints.size() << endl;

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");

    return true;
}

bool TestArmController::control()
{
    joystick->getModeButtonState() ;
    //std::cout<<"q_ref[0]=" << joystick->getModeButtonState() <<std::endl;   

    q_ref[0] += -0.6 * joystick->getPosition(Joystick::L_STICK_H_AXIS) * dt;
    q_ref[1] += -0.6  * joystick->getPosition(Joystick::R_STICK_H_AXIS) * dt;

    std::cout<<"q_ref[1]=" << joystick->getButtonState(Joystick::A_BUTTON) <<std::endl;
    std::cout<<"q_ref[2]=" << joystick->getPosition(Joystick::L_STICK_H_AXIS) <<std::endl;

    double maxerror = radian(5.0);
    for (size_t i = 0; i < armJoints.size(); ++i) {
      auto joint = armJoints[i];
      auto& q = q_ref[i];
      auto q_current = joint->q();
      auto q_lower = std::max(q_current - maxerror, joint->q_lower());
      auto q_upper = std::min(q_current + maxerror, joint->q_upper());
      if (q < q_lower) {
        q = q_lower;
      } else if (q > q_upper) {
        q = q_upper;
      }
    }

    for (size_t i = 0; i < armJoints.size(); ++i) {
      armJoints[i]->q_target() = q_ref[i];
    }

    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TestArmController)
