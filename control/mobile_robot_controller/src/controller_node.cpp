#include <controller.hpp>

int main(int argc, char ** argv){
    ros::init(argc, argv, "mobile_robot_controller_node");
    ros::NodeHandle nh("~");
	MobileRobotController controller(nh);

    ros::spin();

    return 0;
}

