#include "dnetwork/team.hpp"
#include "dnetwork/gamecontroller.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dnetwork_node");
    ros::NodeHandle nh("~");

    dnetwork::Team t(&nh);
    dnetwork::GameController g(&nh);

    t.spin();
    g.spin();

    t.join();
    g.join();
}
