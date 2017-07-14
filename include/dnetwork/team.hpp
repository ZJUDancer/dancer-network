#pragma once
#include <dtransmit/dtransmit.hpp>
#include <dprocess/dprocess.hpp>

#include <dmsgs/MotionInfo.h>
#include <dmsgs/BehaviorInfo.h>
#include <dmsgs/VisionInfo.h>

namespace dnetwork {

class Team : public dprocess::DProcess<Team> {
public:
    explicit Team(ros::NodeHandle* n);
    ~Team();
    void tick() override;

private:
    ros::NodeHandle* nh_;
    ros::Subscriber motion_pub_;
    ros::Subscriber behavior_pub_;
    ros::Subscriber vision_pub_;

    // Publish parsed GCInfo
    ros::Publisher pub_;

    // UDP recv GC
    dtransmit::DTransmit* gc_transmit_;

    // UDP recv&send TeamInfo
    dtransmit::DTransmit* team_transmit_;

    void MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg);
    void BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr& msg);
    void VisionCallback(const dmsgs::VisionInfo::ConstPtr& msg);
};

} // namespace dnetwork
