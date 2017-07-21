#include "dnetwork/team.hpp"

namespace dnetwork {

static const int NETWORK_FREQ = 30;
Team::Team(ros::NodeHandle* nh)
  : DProcess(NETWORK_FREQ, false)
  , nh_(nh)
{
    // get params from config server
    if (!nh_->getParam("RobotId", player_numebr_))
        throw std::runtime_error("Can't get player number!");

    if (!nh_->getParam("/ZJUDancer/TeamNumber", team_number_))
        throw std::runtime_error("Can't get team number!");

    if (!nh_->getParam("/ZJUDancer/TeamCyan", team_cyan_))
        throw std::runtime_error("Can't decide team color!");

    if (!nh_->getParam("/ZJUDancer/udpBroadcastAddress", udp_broadcast_address))
        throw std::runtime_error("Can't get udp broadcast address!");

    // ROS subscriber and publisher
    motion_sub_ = nh_->subscribe("/dmotion_" + std::to_string(player_numebr_) + "/MotionInfo", 1, &Team::MotionCallback, this);
    vision_sub_ = nh_->subscribe("/dvision" + std::to_string(player_numebr_) + "/VisionInfo", 1, &Team::VisionCallback, this);
    behavior_sub_ = nh_->subscribe("/dbehavior_" + std::to_string(player_numebr_) + "/BehaviorInfo", 1, &Team::BehaviorCallback, this);
    pub_ = nh_->advertise<dmsgs::TeamInfo>("/humanoid/TeamInfo", 1);

    // dtranmitter
    // TODO(corenel) unique port for each robot? or unified one?
    transmitter_ = new dtransmit::DTransmit(udp_broadcast_address);
    transmitter_->addRawRecv(dconstant::network::TeamInfoBroadcastAddressBase, [this](void* buffer, std::size_t size) {
        dmsgs::TeamInfo msg = *(dmsgs::TeamInfo*)buffer;
        pub_.publish(msg);
    });
}

Team::~Team()
{
}

void
Team::tick()
{
}

void
Team::MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg)
{
}

void
Team::BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr& msg)
{
}

void
Team::VisionCallback(const dmsgs::VisionInfo::ConstPtr& msg)
{
}

} // namespace dnetwork
