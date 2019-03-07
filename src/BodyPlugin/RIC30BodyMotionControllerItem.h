/**
   @author Yasuhiro Masutani
*/

#ifndef CNOID_BODY_PLUGIN_RIC30_BODY_MOTION_CONTROLLER_ITEM_H
#define CNOID_BODY_PLUGIN_RIC30_BODY_MOTION_CONTROLLER_ITEM_H

#include "ControllerItem.h"
#include "exportdecl.h"

namespace cnoid {

class RIC30BodyMotionControllerItemImpl;

class CNOID_EXPORT RIC30BodyMotionControllerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    RIC30BodyMotionControllerItem();
    RIC30BodyMotionControllerItem(const RIC30BodyMotionControllerItem& org);
    virtual ~RIC30BodyMotionControllerItem();
        
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;
    double pgain() const { return pgain_; }
    double dgain() const { return dgain_; }
    double igain() const { return igain_; }
    double torquemax() const { return torquemax_; }
    double friction() const { return friction_; }

protected:
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    double pgain_;
    double dgain_;
    double igain_;
    double torquemax_;
    double friction_;
        
private:
    friend class RIC30BodyMotionControllerItemImpl;
    RIC30BodyMotionControllerItemImpl* impl;
};
        
typedef ref_ptr<RIC30BodyMotionControllerItem> RIC30BodyMotionControllerItemPtr;

}

#endif
