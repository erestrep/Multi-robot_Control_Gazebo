#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
class Communicator
{
	private:
		ros::NodeHandle nh_;
		std::string my_ns_;
		bool mode_multi_;
        std::vector<ros::Subscriber> sub_odoms_;
        std::vector<nav_msgs::Odometry> odometries_;
        ros::Subscriber sub_my_odom_;
        nav_msgs::Odometry myOdometry_;
        
        ros::Publisher pub_my_status_;
        std::vector<bool> neighbors_status_;
        std::vector<ros::Subscriber> sub_status_;
        ros::Subscriber sub_my_status_;
        ros::Subscriber activation_sub_;
        bool my_status_, applyCtrlFlag_;
        int my_index_;
        
        std::vector<ros::Subscriber> sub_filters_;
        std::vector<std_msgs::Float32MultiArray> filters_;
       
	public:
		Communicator(ros::NodeHandle nh):nh_(nh){
			myNameIsWhat();
			otherSubscribers();
			mySubscribers();
			pub_my_status_ = nh_.advertise<std_msgs::Bool>("activity_status",1);
			my_status_ = true; //default
			applyCtrlFlag_ = false; //default
		};
		
		~Communicator();
		
		void myNameIsWhat(){
			std::string ns = ros::this_node::getNamespace();
			my_ns_ = ns.substr(1,ns.size()); //remove "/"
			//ROS_INFO_STREAM("My namespace is = " << my_ns_);
			mode_multi_ = false;
		};
		
		void otherSubscribers(){
			//Read list of robots
			XmlRpc::XmlRpcValue liste_robots;
			bool have_param = nh_.getParam("liste_robots", liste_robots);
			if(have_param){
				std::vector<std::string> liste_rob;
				int nb_robots = liste_robots.size();
				my_index_ = -1;

				if(nb_robots > 1){   
					  liste_rob.resize(nb_robots);
					  bool I_am_in_the_list = false;
					  for(int i =0;i<nb_robots;i++){
						  liste_rob[i] = (std::string)liste_robots[i];
						  if(my_ns_.compare(liste_rob[i]) == 0){
							  ROS_INFO_STREAM("Robot " << i << " is " << liste_rob[i] << " - this is me !"); my_index_ = i;
							  I_am_in_the_list = true; 
							  mode_multi_ = true; //mode_multi is activated only if several robots in the list AND the current robot is in the list
						  }
						  else{ROS_INFO_STREAM("Robot " << i << " is " << liste_rob[i]);}
					  }
					  if(I_am_in_the_list == false){ROS_INFO_STREAM("I am not in the list - mode mono");}
					  
					if(mode_multi_){
						//Create subscribers to neighbors
						sub_odoms_.resize(nb_robots - 1);
						odometries_.resize(nb_robots - 1);
						neighbors_status_.resize(nb_robots - 1);
						sub_filters_.resize(nb_robots - 1);
						filters_.resize(nb_robots - 1);
						std::string topic;
						int j = 0;
						for(int i = 0 ; i<nb_robots; i++)       
						if(i!=my_index_){
							topic = "/"+liste_rob[i]+"/mobile_robot_controller/current_odometry";
							sub_odoms_.push_back(nh_.subscribe<nav_msgs::Odometry>(topic, 1, boost::bind(&Communicator::odomsCallback, this, _1, j))); // pass neighbor number to subscriber
							neighbors_status_[j] = true; //active by default
							topic = "/"+liste_rob[i]+"/mobile_robot_controller/activity_status";
							sub_status_.push_back(nh_.subscribe<std_msgs::Bool>(topic, 1, boost::bind(&Communicator::statusCallback, this, _1, j))); // pass neighbor number to subscriber	
							//~ topic = "/"+liste_rob[i]+"/mobile_robot_controller/filters";
							topic = "/"+liste_rob[i]+"/mobile_robot_controller/delayed_filters";
							sub_filters_.push_back(nh_.subscribe<std_msgs::Float32MultiArray>(topic, 1, boost::bind(&Communicator::filtersCallback, this, _1, j))); // pass neighbor number to subscriber
							j++;
						}
					}
				}
			}
		};
		
		void mySubscribers(){
			std::string topic = "/"+my_ns_+"/mobile_robot_controller/current_odometry";
			sub_my_odom_ = nh_.subscribe<nav_msgs::Odometry>(topic,1,&Communicator::myOdomCallback, this);
			topic = "/"+my_ns_+"/mobile_robot_controller/activity_status";
			sub_my_status_ = nh_.subscribe<std_msgs::Bool>(topic,1,&Communicator::myStatusCallback, this);
			topic = "/"+my_ns_+"/mobile_robot_controller/cmd_mode";
			activation_sub_ = nh_.subscribe(topic, 1, &Communicator::activation_callback, this);
		};
		
		bool is_multi(){return mode_multi_;};
		
		void odomsCallback(const nav_msgs::OdometryConstPtr& odometry, int neighbor_number){odometries_[neighbor_number] = *odometry;};
		std::vector<nav_msgs::Odometry> getNeighborsOdometries(){return odometries_;};
		void myOdomCallback(const nav_msgs::OdometryConstPtr& odometry){myOdometry_ = *odometry;}
		nav_msgs::Odometry getMyOdometry(){return myOdometry_;};
		
		void statusCallback(const std_msgs::BoolConstPtr& status, int neighbor_number){neighbors_status_[neighbor_number] = status->data;};
		std::vector<bool> getNeighborsStatus(){return neighbors_status_;};
		void myStatusCallback(const std_msgs::BoolConstPtr& status){my_status_ = status->data;};
		bool getMyStatus(){return my_status_;};
		int getMyIndex(){return my_index_;};
		
		void filtersCallback(const std_msgs::Float32MultiArrayConstPtr& filter, int neighbor_number){filters_[neighbor_number] = *filter;};
		std::vector<std_msgs::Float32MultiArray> getNeighborsFilters(){return filters_;};
		
		void activation_callback(const std_msgs::Bool& activation_bool){applyCtrlFlag_ = activation_bool.data;};
		bool getMyMode(){return applyCtrlFlag_;};
};
