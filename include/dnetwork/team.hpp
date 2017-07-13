#pragma once
#include <dtransmit/dtransmit.hpp>
#include <dprocess/dprocess.hpp>
#include <dmotion/MotionInfo.h>
#include <dbehavior/BehaviorInfo.h>
#include <dvision/VisionInfo.h>

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

    void MotionCallback(const dmotion::MotionInfo::ConstPtr& msg);
    void BehaviorCallback(const dbehavior::BehaviorInfo::ConstPtr& msg);
    void VisionCallback(const dvision::VisionInfo::ConstPtr& msg);
};

} // namespace dnetwork
