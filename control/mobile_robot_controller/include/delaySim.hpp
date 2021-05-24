#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

class DelaySimulator
{
	private:
		ros::NodeHandle nh_;
        std::vector<ros::Subscriber> sub_filters_;
        std::vector<std_msgs::Float32MultiArray> filters_;
        std::vector<ros::Publisher> pub_delayed_filters_;
        
        int nb_robots_;
        float Te_;
        ros::Timer timer_;
        
        struct SFilter{
			double time;
			Eigen::Vector3f value;
		};
		
		struct SBuffer{
			std::deque<SFilter> list;
		};
       
        std::size_t buff_sz_;
        std::vector<SBuffer> buffers_;

	public:
		DelaySimulator(ros::NodeHandle nh);
		~DelaySimulator();
		void subscribeFilters();
		void filtersCallback(const std_msgs::Float32MultiArrayConstPtr& filter, int robot_number);
		void delayFilters(const ros::TimerEvent& event);
		double randomGaussianDelay(double mu, double sigma);
};
