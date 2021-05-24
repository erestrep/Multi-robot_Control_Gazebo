#include <delaySim.hpp>

DelaySimulator::DelaySimulator(ros::NodeHandle nh)
: nh_(nh), Te_(0.01), buff_sz_(100)
{
	subscribeFilters();
	
	// Create Publishers
	pub_delayed_filters_.resize(nb_robots_);
	std::string topic;
	for(int i = 0 ; i<nb_robots_; i++){ 
		int j = i+1; 
		std::string index = std::to_string(j);  
		//~ topic = "taz0"+index+"_delayed_filters";
		// TO DO: replace topic name and do a remap in the .launch file
		topic = "/taz0"+index+"/mobile_robot_controller/delayed_filters";
		pub_delayed_filters_[i] = nh_.advertise<std_msgs::Float32MultiArray>(topic,1);
	}	
	
	buffers_.resize(nb_robots_);
		
	//Timer
	timer_ = nh_.createTimer(ros::Duration(ros::Rate(1/Te_)), &DelaySimulator::delayFilters, this, false, true);
}
		
DelaySimulator::~DelaySimulator(){};
		
void DelaySimulator::subscribeFilters()
{
	//Read list of robots
	XmlRpc::XmlRpcValue liste_robots;
	bool have_param = nh_.getParam("liste_robots", liste_robots);
	if(have_param){
		std::vector<std::string> liste_rob;
		nb_robots_ = liste_robots.size();
		liste_rob.resize(nb_robots_);
		for(int i =0;i<nb_robots_;i++){
			liste_rob[i] = (std::string)liste_robots[i];
		}
			  
		//Create subscribers
		sub_filters_.resize(nb_robots_);
		filters_.resize(nb_robots_);
		std::string topic;
		for(int i = 0 ; i<nb_robots_; i++){    
			topic = "/"+liste_rob[i]+"/mobile_robot_controller/undelayed_filters";
			sub_filters_.push_back(nh_.subscribe<std_msgs::Float32MultiArray>(topic, 1, boost::bind(&DelaySimulator::filtersCallback, this, _1, i))); // pass robot number to subscriber
		}				
	}
}
		
void DelaySimulator::filtersCallback(const std_msgs::Float32MultiArrayConstPtr& filter, int robot_number)
{	
	filters_[robot_number] = *filter;
	
	Eigen::Vector3f filter_var;					
	filter_var << filters_[robot_number].data[0], filters_[robot_number].data[2], filters_[robot_number].data[4];
	
	//Time
	double t = ros::Time::now().toSec();
	
	SFilter current_read = {t,filter_var};
    buffers_[robot_number].list.push_back(current_read);
    
    if (buffers_[robot_number].list.size() > buff_sz_){
		buffers_[robot_number].list.pop_front();
	}
}

void DelaySimulator::delayFilters(const ros::TimerEvent& event)
{	// apply variable delay to the filter signals
    double current_delay = randomGaussianDelay(0.5,0.0003);
    current_delay += -0.2;
    double t = ros::Time::now().toSec();
    double target_time = t - current_delay;
    
    // Vector of target filter values for each robot
	std::vector<SFilter> closest;
	closest.resize(nb_robots_);
    
    if (target_time > 0){
		for (size_t k = 0; k < nb_robots_ ; k++)
		{
			int size_list = buffers_[k].list.size();
				
			for (size_t i = 0; i < size_list ; i++)
			{	
				double diffCurrent = abs(buffers_[k].list[i].time - target_time); // t - \tau(t)
				if (diffCurrent < 0.001){
					closest[k] = buffers_[k].list[i]; // closest time-control pair
					break;
				} else {
					double diffClosest = abs(closest[k].time - target_time);
					
					if (diffCurrent < diffClosest){
						closest[k] = buffers_[k].list[i]; // closest time-control pair
					}
				}
			}
		}
	} else {
		Eigen::Vector3f initialFilter;
		initialFilter << 0.0, 0.0, 0.0;
		SFilter initial_pair = {t,initialFilter};
		for (size_t k = 0; k < nb_robots_ ; k++)
		{
			closest[k] = initial_pair;
		}
	}
	
	// Publish delayed filter variables
	for (size_t k = 0; k < nb_robots_ ; k++)
	{
		std_msgs::Float32MultiArray ft_msg;
		ft_msg.data.push_back(closest[k].value.x());
		ft_msg.data.push_back(closest[k].value.y());
		ft_msg.data.push_back(closest[k].value.z());
		pub_delayed_filters_[k].publish(ft_msg);	
	}
}

double DelaySimulator::randomGaussianDelay(double mu, double var)
{
    constexpr double epsilon = std::numeric_limits<double>::epsilon();

    //initialize the random uniform number generator (runif) in a range 0 to 1
    static std::mt19937 rng(std::random_device{}()); // Standard mersenne_twister_engine seeded with rd()
    static std::uniform_real_distribution<> runif(0.0, 1.0);

    //create two random numbers, make sure u1 is greater than epsilon
    double u1, u2;
    do
    {
        u1 = runif(rng);
        u2 = runif(rng);
    }
    while (u1 <= epsilon);

    //compute z0 and z1
    auto mag = sqrt(var) * sqrt(-2.0 * log(u1));
    auto d0  = mag * cos(2.0 * M_PI * u2) + mu;
    //~ auto d1  = mag * sin(2.0 * M_PI * u2) + mu;
	
	//return normally distributed random number (delay)
    return d0;
}
