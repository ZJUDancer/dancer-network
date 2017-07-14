#pragma once
#include <string>
#include <mutex>
#include <ros/ros.h>
#include <dprocess/dprocess.hpp>
#include <dtransmit/dtransmit.hpp>
#include "dnetwork/RoboCup/RoboCupGameControlData.h"
#include "dmsgs/GCInfo.h"

namespace dnetwork {
class GameController : public dprocess::DProcess<GameController> {
public:
    explicit GameController(ros::NodeHandle* nh);
    ~GameController();
    void tick() override;

private:
    void ParseData(RoboCupGameControlData& gameData);

    bool IsValidData(RoboCupGameControlData& gameData);

    bool CheckHeader(char* header);

    bool IsThisGame(RoboCupGameControlData& gameData);

    void RawSwapTeams(RoboCupGameControlData& gameData);

    bool GameDataEqual(RoboCupGameControlData& gameData, RoboCupGameControlData& pervious);

private:
    ros::NodeHandle* nh_;
    dtransmit::DTransmit* transmitter_;
    RoboCupGameControlData data_;
    ros::Publisher pub_;
    dmsgs::GCInfo info_;

    bool connected_;
    bool penalised_;
    ros::Time last_valid_packet_timestamp_;
    bool teamCyan_;
    std::string gameControllerAddress_;
    int playerNumber_;
    int teamNumber_;
    std::mutex dataLock_;
    RoboCupGameControlReturnData ret_;
};
}