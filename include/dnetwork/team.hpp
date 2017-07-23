#pragma once
#include "dconfig/dconstant.hpp"
#include <dprocess/dprocess.hpp>
#include <dtransmit/dtransmit.hpp>

#include <dmsgs/BehaviorInfo.h>
#include <dmsgs/MotionInfo.h>
#include <dmsgs/TeamInfo.h>
#include <dmsgs/VisionInfo.h>
#include <dmsgs/GCInfo.h>

#include <mutex>
#include <ros/ros.h>
#include <string>

namespace dnetwork {
// FIXME(corenel) get from GC?
static const int NUM_ROBOT = 6;

class Team : public dprocess::DProcess<Team>
{
  public:
    explicit Team(ros::NodeHandle* nh);
    ~Team();
    void tick() override;

  private:
    ros::NodeHandle* nh_;
    ros::Subscriber motion_sub_;
    ros::Subscriber behavior_sub_;
    ros::Subscriber vision_sub_;
    ros::Subscriber gc_sub_;

    ros::Publisher pub_;

    // UDP recv&send TeamInfo
    dtransmit::DTransmit* transmitter_;
    std::string udp_broadcast_address;
    dmsgs::TeamInfo info_;

    int player_number_;
    int team_number_;
    bool team_cyan_;

    bool penalised_ = false;
    bool unstable_ = true;

    bool visionReady_ = false;
    bool behaviorReady_ = false;
    bool motionReady_ = false;


    void MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg);
    void BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr& msg);
    void VisionCallback(const dmsgs::VisionInfo::ConstPtr& msg);
    void GCCallback(const dmsgs::GCInfo::ConstPtr& msg);
};

} // namespace dnetwork
