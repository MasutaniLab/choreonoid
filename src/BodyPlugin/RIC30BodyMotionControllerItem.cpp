/*!
  @file
  @author Yasuhiro Masutani
*/

#include "RIC30BodyMotionControllerItem.h"
#include "BodyMotionItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/ControllerIO>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include <cnoid/Archive>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class RIC30BodyMotionControllerItemImpl
{
public:
    RIC30BodyMotionControllerItem* self;
    BodyMotionItemPtr motionItem;
    MultiValueSeqPtr qseqRef;
    BodyPtr body;
    int currentFrame;
    int lastFrame;
    int numJoints;
    vector<double> ie;

    RIC30BodyMotionControllerItemImpl(RIC30BodyMotionControllerItem* self);
    bool initialize(ControllerIO* io);
    void output();
};

}


void RIC30BodyMotionControllerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& itemManager = ext->itemManager();
    itemManager.registerClass<RIC30BodyMotionControllerItem>(N_("RIC30BodyMotionControllerItem"));
    itemManager.addCreationPanel<RIC30BodyMotionControllerItem>();
}


RIC30BodyMotionControllerItem::RIC30BodyMotionControllerItem()
{
    impl = new RIC30BodyMotionControllerItemImpl(this);
    pgain_ = 31.25;
    dgain_ = 2.5;
    igain_ = 0.0;
    torquemax_ = 0.5;
    friction_ = 0.10;
}


RIC30BodyMotionControllerItem::RIC30BodyMotionControllerItem(const RIC30BodyMotionControllerItem& org)
    : ControllerItem(org)
{
    impl = new RIC30BodyMotionControllerItemImpl(this);
    pgain_ = org.pgain_;
    dgain_ = org.dgain_;
    igain_ = org.igain_;
    torquemax_ = org.torquemax_;
    friction_ = org.friction_;
}


RIC30BodyMotionControllerItemImpl::RIC30BodyMotionControllerItemImpl(RIC30BodyMotionControllerItem* self)
    : self(self)
{

}


RIC30BodyMotionControllerItem::~RIC30BodyMotionControllerItem()
{
    delete impl;
}


bool RIC30BodyMotionControllerItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool RIC30BodyMotionControllerItemImpl::initialize(ControllerIO* io)
{
    auto mv = MessageView::instance();

    mv->putln(_("RIC30BodyMotionControllerItemImpl::initialize()"));
    mv->putln(format(_("pgain: {}"), self->pgain()));
    mv->putln(format(_("dgain: {}"), self->dgain()));
    mv->putln(format(_("igain: {}"), self->igain()));
    mv->putln(format(_("torquemax: {}"), self->torquemax()));
    mv->putln(format(_("friction: {}"), self->friction()));
    ItemList<BodyMotionItem> motionItems;
    if(!motionItems.extractChildItems(self)){
        mv->putln(
            format(_("Any body motion item for {} is not found."), self->name()),
            MessageView::ERROR);
        return false;
    }
    motionItem = motionItems.front();
    // find the first checked item
    ItemTreeView* itv = ItemTreeView::instance();
    for(int i=0; i < motionItems.size(); ++i){
        if(itv->isItemChecked(motionItems[i])){
            motionItem = motionItems[i];
            break;
        }
    }

    qseqRef = motionItem->jointPosSeq();

    body = io->body();
    currentFrame = 0;
    lastFrame = std::max(0, qseqRef->numFrames() - 1);
    numJoints = std::min(body->numJoints(), qseqRef->numParts());
    if(qseqRef->numFrames() == 0){
        mv->putln(
            format(_("{0} for {1} is empty."), motionItem->name(), self->name()),
            MessageView::ERROR);
        return false;
    }
    if(fabs(qseqRef->frameRate() - (1.0 / io->timeStep())) > 1.0e-6){
        mv->putln(
            format(_("The frame rate of {} is different from the world frame rate."), motionItem->name()),
            MessageView::ERROR);
    }

    ie.resize(numJoints);
    for (int i = 0; i < numJoints; i++) {
        ie[i] = 0.0;
    }

    // Overwrite the initial position and pose
    MultiSE3SeqPtr lseq = motionItem->linkPosSeq();
    if(lseq->numParts() > 0 && lseq->numFrames() > 0){
        SE3& p = lseq->at(0, 0);
        Link* rootLink = body->rootLink();
        rootLink->p() = p.translation();
        rootLink->R() = p.rotation().toRotationMatrix();
    }
    self->output();
    body->calcForwardKinematics();
    
    return true;
}


bool RIC30BodyMotionControllerItem::start()
{
    control();
    return true;
}
    
    
double RIC30BodyMotionControllerItem::timeStep() const
{
    return impl->qseqRef->timeStep();
}
        

void RIC30BodyMotionControllerItem::input()
{

}


bool RIC30BodyMotionControllerItem::control()
{
    if(++impl->currentFrame > impl->lastFrame){
        impl->currentFrame = impl->lastFrame;
        return false;
    }
    return true;
}
        

void RIC30BodyMotionControllerItemImpl::output()
{
    int prevFrame = std::max(currentFrame - 1, 0);
    int nextFrame = std::min(currentFrame + 1, lastFrame);
            
    MultiValueSeq::Frame q0 = qseqRef->frame(prevFrame);
    MultiValueSeq::Frame q1 = qseqRef->frame(currentFrame);
    MultiValueSeq::Frame q2 = qseqRef->frame(nextFrame);
    
    double dt = qseqRef->timeStep();
    double dt2 = dt * dt;
    double pgain = self->pgain();
    double dgain = self->dgain();
    double igain = self->igain();
    double torquemax = self->torquemax();
    double friction = self->friction();
    double iemax;
    if (igain == 0.0) {
        iemax = 0.0;
    } else {
        iemax = torquemax / igain;
    }
    
    for(int i=0; i < numJoints; ++i){
        Link* joint = body->joint(i);
        double q = q1[i];
        double dq = (q2[i] - q1[i]) / dt;
        ie[i] += (q - joint->q())*dt;
        if (ie[i] > iemax) {
            ie[i] = iemax;
        } else if (ie[i] < -iemax) {
            ie[i] = -iemax;
        }
        double u = pgain*(q-joint->q()) + dgain*(dq-joint->dq()) + igain*ie[i];
        if (u<-torquemax) {
            u = -torquemax;
        } else if (u>torquemax) {
            u = torquemax;
        }
        if (abs(joint->dq()) < 1e-6) {
            if (abs(u) < friction ) {
                u = 0;
            } else if (u > 0) {
                u -= friction;
            } else {
                u += friction;
            }
        } else if (joint->dq() > 0) {
            u -= friction;
        } else {
            u += friction;
        }
        joint->u() = u;
    }
}


void RIC30BodyMotionControllerItem::output()
{
    impl->output();
}


void RIC30BodyMotionControllerItem::stop()
{
    impl->qseqRef.reset();
    impl->motionItem = 0;
    impl->body = 0;
}


void RIC30BodyMotionControllerItem::onDisconnectedFromRoot()
{
    stop();
}
    

Item* RIC30BodyMotionControllerItem::doDuplicate() const
{
    return new RIC30BodyMotionControllerItem(*this);
}


void RIC30BodyMotionControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("P gain"), pgain_, changeProperty(pgain_));
    putProperty(_("D gain"), dgain_, changeProperty(dgain_));
    putProperty(_("I gain"), igain_, changeProperty(igain_));
    putProperty(_("Torque max"), torquemax_, changeProperty(torquemax_));
    putProperty(_("Friction"), friction_, changeProperty(friction_));
}


bool RIC30BodyMotionControllerItem::store(Archive& archive)
{
    archive.write("pgain", pgain_);
    archive.write("dgain", dgain_);
    archive.write("igain", igain_);
    archive.write("torquemax", torquemax_);
    archive.write("friction", friction_);
    return true;
}
    

bool RIC30BodyMotionControllerItem::restore(const Archive& archive)
{
    archive.read("pgain", pgain_);
    archive.read("dgain", dgain_);
    archive.read("igain", igain_);
    archive.read("torquemax", torquemax_);
    archive.read("friction", friction_);
    return true;
}
