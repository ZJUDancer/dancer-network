#include <dmsgs/GCInfo.h>
#include "dnetwork/team.hpp"

namespace dnetwork {

static const int NETWORK_FREQ = 30;
Team::Team(ros::NodeHandle *nh)
    : DProcess(NETWORK_FREQ, false), nh_(nh) {
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
    vision_sub_ = nh_->subscribe("/dvision_" + std::to_string(player_number_) + "/VisionInfo", 1, &Team::VisionCallback, this);
    behavior_sub_ = nh_->subscribe("/dbehavior_" + std::to_string(player_number_) + "/BehaviorInfo",
                                   1,
                                   &Team::BehaviorCallback,
                                   this);
    gc_sub_ = nh_->subscribe("/dnetwork_" + std::to_string(player_number_) + "/GCInfo", 1, &Team::GCCallback, this);
    pub_ = nh_->advertise<dmsgs::TeamInfo>("/dnetwork_" + std::to_string(player_number_) + "/TeamInfo", 1);

    transmitter_ = new dtransmit::DTransmit("255.255.255.255");
    transmitter_->addRawRecv(dconstant::network::TeamInfoBroadcastAddress, [this](void *buffer, std::size_t size) {
        if (size == sizeof(dmsgs::TeamInfo)) {
            dmsgs::TeamInfo team_info = *(dmsgs::TeamInfo *) buffer;
            if (team_info.player_number != player_number_) {
                team_info.recv_timestamp = ros::Time::now();
//                 ROS_INFO("I heared message from robot %d", team_info.player_number);
                pub_.publish(team_info);
            }
        }
    });
    transmitter_->startService();
}

Team::~Team() {
}

void
Team::tick() {
    info_.player_number = player_number_;
    info_.incapacitated = false;

    if (unstable_) {
        //std::cout << (int)unstable_ << " " << (int)penalised_ << std::endl;
        info_.incapacitated = true;
    }

    if (motionReady_ && visionReady_ && behaviorReady_) {
        transmitter_->sendRaw(dconstant::network::TeamInfoBroadcastAddress, (void *) &info_, sizeof(info_));
        motionReady_ = false;
        visionReady_ = false;
        behaviorReady_ = false;
    }
//     ROS_INFO("team info is sent");
}

void
Team::MotionCallback(const dmsgs::MotionInfo::ConstPtr &msg) {
    dmsgs::MotionInfo motion_info = *msg;
    unstable_ = !motion_info.stable;
//    ROS_INFO("motion info ready");
    motionReady_ = true;
}

void
Team::BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr &msg) {
    info_.behavior_info = *msg;
//    ROS_INFO("behavior info ready");
    behaviorReady_ = true;
}

void
Team::VisionCallback(const dmsgs::VisionInfo::ConstPtr &msg) {
    dmsgs::VisionInfo vision_info = *msg;
    info_.see_ball = vision_info.see_ball;
    info_.ball_field = vision_info.ball_field;
    info_.ball_global = vision_info.ball_global;
    info_.robot_pos = vision_info.robot_pos;
//    ROS_INFO("vision info ready");
//    std::cout << "see ball: " <<info_.see_ball << std::endl;
    visionReady_ = true;
}

void Team::GCCallback(const dmsgs::GCInfo::ConstPtr &msg) {
    auto gcinfo = *msg;
    penalised_ = gcinfo.penalised;
//    ROS_INFO("GC info ready");
}

} // namespace dnetwork
