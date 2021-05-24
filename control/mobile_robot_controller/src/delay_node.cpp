#include <delaySim.hpp>

int main(int argc, char ** argv){
    ros::init(argc, argv, "delayComm_node");
    ros::NodeHandle nh("~");
	DelaySimulator DelayFilters(nh);

    ros::spin();

    return 0;
}
