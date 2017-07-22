#include "dnetwork/team.hpp"

namespace dnetwork {

static const int NETWORK_FREQ = 30;
Team::Team(ros::NodeHandle* nh)
  : DProcess(NETWORK_FREQ, false)
  , nh_(nh)
{
    // get params from config server
    if (!nh_->getParam("RobotId", player_number_))
        throw std::runtime_error("Can't get player number!");

    if (!nh_->getParam("/ZJUDancer/TeamNumber", team_number_))
        throw std::runtime_error("Can't get team number!");

    if (!nh_->getParam("/ZJUDancer/TeamCyan", team_cyan_))
        throw std::runtime_error("Can't decide team color!");

    if (!nh_->getParam("/ZJUDancer/udpBroadcastAddress", udp_broadcast_address))
        throw std::runtime_error("Can't get udp broadcast address!");

    // ROS subscriber and publisher
    motion_sub_ = nh_->subscribe("/dmotion_" + std::to_string(player_number_) + "/MotionInfo", 1, &Team::MotionCallback, this);
    vision_sub_ = nh_->subscribe("/dvision" + std::to_string(player_number_) + "/VisionInfo", 1, &Team::VisionCallback, this);
    behavior_sub_ = nh_->subscribe("/dbehavior_" + std::to_string(player_number_) + "/BehaviorInfo", 1, &Team::BehaviorCallback, this);
    pub_ = nh_->advertise<dmsgs::TeamInfo>("/dnetwork_" + std::to_string(player_number_) + "/TeamInfo", 1);

    // dtranmitter
    // TODO(corenel) unique port for each robot? or unified one?
    transmitter_ = new dtransmit::DTransmit(udp_broadcast_address);
    transmitter_->addRawRecv(dconstant::network::TeamInfoBroadcastAddress, [this](void* buffer, std::size_t size) {
        if (size == sizeof(dmsgs::TeamInfo)) {
            std::unique_lock<std::mutex> lock(data_lock_);
            dmsgs::TeamInfo team_info = *(dmsgs::TeamInfo*)buffer;
            if (team_info.player_number != player_number_) {
                // ROS_INFO("I heared message from robot %d", team_info.player_number);
                pub_.publish(team_info);
            }
        }
    });
    transmitter_->startService();
}

Team::~Team()
{
}

void
Team::tick()
{
    info_ = dmsgs::TeamInfo();
    info_.timestamp = ros::Time::now();
    info_.player_number = player_number_;
    info_.team_number = team_number_;
    info_.team_cyan = team_cyan_;
    info_.incapacitated = false;

    if (motion_info_ready && vision_info_ready && behavior_info_ready) {
        transmitter_->sendRaw(dconstant::network::TeamInfoBroadcastAddress, (void*)&info_, sizeof(info_));
        motion_info_ready = false;
        vision_info_ready = false;
        behavior_info_ready = false;
        // ROS_INFO("team info is sent");
    }
}

void
Team::MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg)
{
    // TODO(corenel) lock?
    dmsgs::MotionInfo motion_info = *msg;
    info_.gait_type = motion_info.action.bodyCmd.gait_type;
    motion_info_ready = true;
    // ROS_INFO("motion info ready");
}

void
Team::BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr& msg)
{
    // TODO(corenel) lock?
    dmsgs::BehaviorInfo behavior_info = *msg;
    info_.current_role = behavior_info.current_role;
    info_.skill = behavior_info.skill;
    info_.ball_lost_time = behavior_info.ball_lost_time;
    info_.time_to_reach_ball = behavior_info.time_to_reach_ball;
    info_.dest = behavior_info.dest;
    info_.final_dest = behavior_info.final_dest;
    vision_info_ready = true;
    // ROS_INFO("behavior info ready");
}

void
Team::VisionCallback(const dmsgs::VisionInfo::ConstPtr& msg)
{
    // TODO(corenel) lock?
    dmsgs::VisionInfo vision_info = *msg;
    info_.see_ball = vision_info.see_ball;
    info_.ball_field = vision_info.ball_field;
    info_.ball_global = vision_info.ball_global;
    behavior_info_ready = true;
    // ROS_INFO("vision info ready");
}

} // namespace dnetwork
