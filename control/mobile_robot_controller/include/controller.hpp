#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "control_position_msgs/ControllerReference.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <mobile_robot_controller/MobileRobotControllerConfig.h>

#include <communicator.hpp>

class MobileRobotController
{
	private:
		Communicator *com;
		std::vector<nav_msgs::Odometry> neighbors_odometries_;
		nav_msgs::Odometry my_odometry_;
		std::vector<bool> neighbors_status_;
		std::vector<std_msgs::Float32MultiArray> neighbors_filters_;
		bool my_status_, multi_mode_;
		
        ros::Timer timer, timer_filter;
		dynamic_reconfigure::Server<mobile_robot_controller::MobileRobotControllerConfig> server;
		dynamic_reconfigure::Server<mobile_robot_controller::MobileRobotControllerConfig>::CallbackType f;
        
		ros::NodeHandle nh_;
		ros::Subscriber sub_odometry_ , sub_reference_;
		ros::Publisher pub_cmd_vel_, pub_current_odometry_, pub_cmd_consensus_, rviz_marker_pub_, fov_pub_;
		ros::Publisher pub_left_effort_, pub_right_effort_;
		control_position_msgs::ControllerReference ref_;
        Eigen::Vector3f pose_; // [x,y,yaw]
        Eigen::Vector2f vel_body_; //[v,omega]
		float Te_, Te_filter_, kw_ , kv_ , kb_ , threshold_distance_ , threshold_waypoints_ , threshold_yaw_ , vitesse_max_ , vitesse_angulaire_max_, height_;
		bool flag_yaw_ , first_odom_, flag_distance_constraint_, flag_fov_constraint_;
		bool velocity_control_, applyCtrlFlag_, filters_initialized_;
		double t0_;
		float mass_, inertia_, wheel_radius_, wheel_separation_;
		
		// Filter variables
		Eigen::Vector2f filt_alphavx_;
		Eigen::Vector2f filt_alphavy_;
		Eigen::Vector2f filt_alphaw_;
		float dvft_, dwft_, pvft_, pwft_, ka_, pe_freq_;
		ros::Publisher pub_filters_;

	public:
		MobileRobotController(ros::NodeHandle nh);
		~MobileRobotController();
		void updateControl(const ros::TimerEvent& event);
		void callback_odometry(const nav_msgs::Odometry& odometry);
		void callback_reference(const control_position_msgs::ControllerReference& msg);
		void callback_param(mobile_robot_controller::MobileRobotControllerConfig &config, uint32_t level);
        Eigen::Vector2f nominalControl(Eigen::Vector3f pose, control_position_msgs::ControllerReference ref,float threshold_yaw, bool prediction_mode);
		Eigen::Vector3f calcul_angles(Eigen::Quaternionf q);
		float saturation(float input,float sat);
        void BroadcastCurrentPose(Eigen::Vector3f pose);
        void BroadcastCurrentVel(Eigen::Vector2f vel);
        void BroadcastCurrentOdometry(Eigen::Vector3f pose, Eigen::Vector2f vel_body);
        Eigen::Vector2f consensusControl(int my_index, Eigen::Vector3f pose, Eigen::Vector2f vel_body, Eigen::MatrixXf neighbors_pose, Eigen::MatrixXf neighbors_vel_body, int nb_neighbors, double t);
        Eigen::Vector2f consensusEffortwoVelControl(int my_index, Eigen::Vector3f pose, Eigen::Vector2f vel_body, int nb_neighbors, double t);
        Eigen::Vector2f consensusEffortControl(int my_index, Eigen::Vector3f pose, Eigen::Vector2f vel_body, int nb_neighbors, Eigen::MatrixXf neighbors_pose, double t);
        void init_filters();
        void updateFilter(const ros::TimerEvent& event);
        void rviz_display_robot(Eigen::Vector3f pose, float range, float fov_angle);
};
