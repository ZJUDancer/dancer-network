#include "dnetwork/team.hpp"

namespace dnetwork {

static const int NETWORK_FREQ = 30;
Team::Team(ros::NodeHandle *n)
: DProcess(NETWORK_FREQ, false)
{

}

Team::~Team() {

}

void Team::tick() {

}

}