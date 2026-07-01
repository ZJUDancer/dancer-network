/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file team.cpp
 * @brief
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#include "dnetwork/team.hpp"
#include <dmsgs/GCInfo.h>
namespace dnetwork {

static const int NETWORK_FREQ = 30;
Team::Team(ros::NodeHandle *nh) : DProcess(NETWORK_FREQ, false), nh_(nh) {
  // get params from config server
  if (!nh_->getParam("RobotId", player_number_))
    throw std::runtime_error("Can't get player number!");

  if (!nh_->getParam("/ZJUDancer/TeamNumber", team_number_))
    throw std::runtime_error("Can't get team number!");

  if (!nh_->getParam("/ZJUDancer/TeamCyan", team_cyan_))
    throw std::runtime_error("Can't decide team color!");

  if (!nh_->getParam("/ZJUDancer/udpBroadcastAddress", udp_broadcast_address))
    throw std::runtime_error("Can't get udp broadcast address!");

  if (!nh_->getParam("TEST", test))
    throw std::runtime_error("Can't get test");

  // 获取单播目标 IP
  if (!nh_->getParam("/ZJUDancer/UnicastTargetAddress", unicast_target_address_))
        unicast_target_address_ = "192.168.1.100"; // 默认 fallback 地址

  // monitor 单播端口固定为 20000 + team_number，不再从配置读取
  unicast_target_port_ = 20000 + team_number_;

  // ROS subscriber and publisher
  motion_sub_ = nh_->subscribe("/dmotion_" + std::to_string(player_number_) +
                                   "/MotionInfo",
                               1, &Team::MotionCallback, this);
  vision_sub_ = nh_->subscribe("/dvision_" + std::to_string(player_number_) +
                                   "/VisionInfo",
                               1, &Team::VisionCallback, this);
  behavior_sub_ = nh_->subscribe(
      "/dbehavior_" + std::to_string(player_number_) + "/BehaviorInfo", 1,
      &Team::BehaviorCallback, this);
  gc_sub_ =
      nh_->subscribe("/dnetwork_" + std::to_string(player_number_) + "/GCInfo",
                     1, &Team::GCCallback, this);
  pub_ = nh_->advertise<dmsgs::TeamInfo>(
      "/dnetwork_" + std::to_string(player_number_) + "/TeamInfo", 1);

  transmitter_ = new dtransmit::DTransmit();
  transmitter_->addRawRecv(dconstant::network::TeamInfoBroadcastAddress(team_number_), [this](void *buffer, std::size_t size) {
    if (size == sizeof(dmsgs::TeamInfo)) {
      std::unique_lock<std::mutex> lock(data_lock_);
      dmsgs::TeamInfo team_info =
          *(dmsgs::TeamInfo *)buffer;
      // if (true) {
      if (team_info.team_number == team_number_) {
      // if (team_info.player_number != player_number_ && team_info.team_number == team_number_) {
        team_info.recv_timestamp = ros::Time::now();
        // ROS_INFO("Heard message from robot %d in team %d\n", team_info.player_number, team_info.team_number);
        pub_.publish(team_info);
      }
    }
  });
  transmitter_->startService();
  last_send_time_ = ros::Time::now(); 
  last_monitor_send_time = ros::Time::now();
}

Team::~Team() {}

void Team::tick() {
  info_.player_number = player_number_;
  info_.incapacitated = false;
  info_.team_number = team_number_;

  if (unstable_) {
    // std::cout << (int)unstable_ << " " << (int)penalised_ << std::endl;
    info_.incapacitated = true;
  }
    // --- 动态频率控制逻辑 ---
  double target_interval = 1; // 默认低频：0.5秒一次 (2Hz)
  double monitor_send_interval = 1; // 10Hz monitor unicast
  if (info_.state == dmsgs::TeamInfo::BALL_HANDLING) {
        // 状态1：持球机器人，最高频 (20Hz)
        target_interval = 0.2; 
    } 
  else if (info_.see_ball) {
        // 状态2：看到球的机器人，根据距离线性或阶梯调整
        double dist = std::sqrt(std::pow(info_.ball_field.x, 2) + std::pow(info_.ball_field.y, 2));
        
        if (dist < 100.0) {      // 1米以内：高频 (10Hz)
            target_interval = 0.4;
        } else if (dist < 300.0) { // 3米以内：中频 (5Hz)
            target_interval = 0.8;
        } else {                 // 3米以外：低中频 (2.5Hz)
            target_interval = 1.0;
        }
    } 
  else {
        // 状态3：看不到球且不持球，最低频 (2Hz)
        target_interval = 1.0;
    }

  bool has_online_teammate = false;
  for (std::size_t i = 0; i < info_.mates_online.size(); ++i) {
    if (static_cast<int>(i) + 1 == player_number_) {
      continue;
    }
    if (info_.mates_online[i]) {
      has_online_teammate = true;
      break;
    }
  }
  if (!has_online_teammate) {
    target_interval = 1.5;
  }

  ros::Time now = ros::Time::now();
  const bool allow_broadcast_send = !gc_stopped_ && !penalised_;

  if (allow_broadcast_send &&
      (now - last_send_time_).toSec() >= target_interval) {
    // TODO add lock for message receiving and sending
    // if (motionReady_ && visionReady_ && behaviorReady_) {
    if (behaviorReady_) {
    // if (true) {
      info_.txp_timestamp = ros::Time::now();
      transmitter_->sendRaw(dconstant::network::TeamInfoBroadcastAddress(team_number_),
                            (void *)&info_, sizeof(info_));
      // 更新最后发送时间
      last_send_time_ = now; 

      motionReady_ = false;
      visionReady_ = false;
      behaviorReady_ = false;
    }
    // ROS_INFO("team info is sent");
  }
  if ((now - last_monitor_send_time).toSec() >= monitor_send_interval){
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(unicast_target_port_); // 设置目标端口
            inet_pton(AF_INET, unicast_target_address_.c_str(), &addr.sin_addr); // 设置目标IP

            // 执行单播发送
            sendto(sock, &info_, sizeof(info_), 0, (sockaddr*)&addr, sizeof(addr));
                
            // 打印调试信息 (可选)
            // printf("Unicast TeamInfo to %s:%d\n", unicast_target_address_.c_str(), unicast_target_port_);
                
            close(sock);
            }
        last_monitor_send_time = now;

  }
}

void Team::MotionCallback(const dmsgs::MotionInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::MotionInfo motion_info = *msg;
  unstable_ = !motion_info.stable;
  // ROS_INFO("motion info ready");
  motionReady_ = true;
}

void Team::BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::BehaviorInfo behavior_info = *msg;
  
  info_.role = behavior_info.current_role;
  info_.attack_right = behavior_info.attack_right;

  info_.state = behavior_info.team_play_state;
  info_.kicker_id = behavior_info.kicker_id;
  info_.priority = behavior_info.team_play_priority;
  info_.mates_online = behavior_info.mates_online;

  info_.dest = behavior_info.dest;
  info_.final_dest = behavior_info.final_dest;
  info_.time_since_last_kick = behavior_info.time_since_last_kick;
  info_.time_to_reach_ball = behavior_info.time_to_reach_ball;
  info_.attack_target = behavior_info.attack_target;

  info_.team_ball_global = behavior_info.ball_global;
  // info_.ball_intercept_valid = behavior_info.ball_intercept_valid;
  // info_.ball_intercept_global = behavior_info.ball_intercept_global;

  // --- 新增：Voronoi 数据处理 (动态 -> 固定) ---
  
  // 1. 定义最大容量
  const int MAX_VORONOI_SIZE = 6; 
  // 2. 获取输入数据的实际大小
  int input_size = behavior_info.voronoi.size();
  // 3. 计算实际要拷贝的数量 (取较小值以防止内存溢出)
  int valid_count = std::min(input_size, MAX_VORONOI_SIZE);
  // 4. 赋值长度字段
  info_.voronoi_list_length = (uint8_t)valid_count;
  // 5. 循环搬运数据
  for (int i = 0; i < valid_count; i++) {
      info_.voronoi_list[i] = behavior_info.voronoi[i];
  }

  // ROS_INFO("behavior info ready");
  behaviorReady_ = true;
}

void Team::VisionCallback(const dmsgs::VisionInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::VisionInfo vision_info = *msg;
  info_.see_ball = vision_info.see_ball;
  info_.see_circle = vision_info.see_circle;
  info_.see_goal = vision_info.see_goal;
  info_.robot_pos = vision_info.robot_pos;

  info_.ball_field = vision_info.ball_field;
  info_.ball_global = vision_info.ball_global;

  info_.circle_field = vision_info.circle_field;
  info_.circle_global = vision_info.circle_global;

  info_.goal_field = vision_info.goal_field;
  info_.goal_global = vision_info.goal_global;

  info_.ball_quality = vision_info.ball_quality;
  info_.field_quality = vision_info.field_quality;
  info_.field_consistency = vision_info.field_consistency;
  visionReady_ = true;
}

void Team::GCCallback(const dmsgs::GCInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::GCInfo gc_info = *msg;
  penalised_ = gc_info.penalised;
  gc_stopped_ = gc_info.stopped;
  info_.gc_connected = gc_info.connected;
  info_.gc_state = gc_info.state;
  info_.gamePhase = gc_info.gamePhase;
  info_.setPlay = gc_info.setPlay;
}

} // namespace dnetwork
