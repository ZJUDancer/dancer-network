#include "dnetwork/team.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dnetwork_node");
    ros::NodeHandle nh;

    dnetwork::Team t(&nh);
    t.spin();
    t.join();
}
