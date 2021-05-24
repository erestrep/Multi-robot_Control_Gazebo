#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>

std::vector<ros::Publisher> pubs;

void gazeboCallback(const gazebo_msgs::ModelStates modelStates)
  {
    int elements = modelStates.name.size();
    ROS_INFO_STREAM("Size = " << elements);
    pubs.resize(elements);
    for (int i=0;i<elements;i++){
        nav_msgs::Odometry odom;
        odom.header.frame_id = "world";
        odom.pose.pose = modelStates.pose[i];
        odom.twist.twist = modelStates.twist[i];
        std::string name = modelStates.name[i];
        if(name.find_first_of(" ") == std::string::npos){
            std::string topic = "/"+modelStates.name[i]+"/odometry";
            ROS_INFO_STREAM("Topic = " << topic);
            ros::NodeHandle nh;
            pubs[i] = nh.advertise<nav_msgs::Odometry>(topic,1);
            pubs[i].publish(odom);
        }        
    }
  }

int main(int argc, char** argv)
{
  // Announce this program to the ROS master
  ros::init(argc, argv, "ground_truth_manager");
  // Start the node resource managers (communication, time, etc)
  ros::NodeHandle nh;
  ros::Subscriber sub_gazebo = nh.subscribe("/gazebo/model_states", 1, gazeboCallback);
    
  // Looping
  double freq_hz = 1000;
  ros::Rate loop_rate(freq_hz);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}
