#include <controller.hpp>

MobileRobotController::MobileRobotController(ros::NodeHandle nh)
: nh_(nh)
, pose_(0.0,0.0,0.0)
, Te_(0.01)
, Te_filter_(0.01)
, first_odom_(false)
, height_(0.4)
{
  com = new Communicator(nh_);
  //Subscribers
  sub_odometry_ = nh_.subscribe("odometry",1, &MobileRobotController::callback_odometry, this);
  sub_reference_ = nh_.subscribe("reference",1, &MobileRobotController::callback_reference,this);
		
  //Publishers
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
  pub_current_odometry_ = nh_.advertise<nav_msgs::Odometry>("current_odometry",1);
  pub_cmd_consensus_ = nh_.advertise<geometry_msgs::Twist>("cmd_consensus",1);
  pub_left_effort_ = nh_.advertise<std_msgs::Float64>("left_command",1);
  pub_right_effort_ = nh_.advertise<std_msgs::Float64>("right_command",1);
  rviz_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("robot_marker",1);
  fov_pub_ = nh.advertise<std_msgs::Float32MultiArray>("fov",1);
  pub_filters_ = nh_.advertise<std_msgs::Float32MultiArray>("filters",1);
  
  //Dynamic reconfigure and params
  f = boost::bind(&MobileRobotController::callback_param,this,_1,_2);
  server.setCallback(f);
  
  // Init Filter variables
  filt_alphavx_ << 0.0, 0.0;
  filt_alphavy_ << 0.0, 0.0;
  filt_alphaw_ << 0.0, 0.0;
  
  //init flags
  applyCtrlFlag_ = false;
  flag_yaw_ = false;
  //~ velocity_control_ = true; //
  filters_initialized_ = false;
  
  // Inertial parameters
  mass_ = 5.64;
  inertia_ = 3.115;
  wheel_radius_ = 0.179/2;
  wheel_separation_ = 0.314;
  
  //other params
  ros::param::param<bool>("~velocity_control", velocity_control_, true);
      
  //Timer
  timer = nh_.createTimer(ros::Duration(ros::Rate(1/Te_)), &MobileRobotController::updateControl, this, false, true);
  timer_filter = nh_.createTimer(ros::Duration(ros::Rate(1/Te_filter_)), &MobileRobotController::updateFilter, this, false, true);
}
 
MobileRobotController::~MobileRobotController(){};

void MobileRobotController::callback_odometry(const nav_msgs::Odometry& odometry)
{
  Eigen::Vector3f vitesse_B(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,odometry.twist.twist.linear.z);
  
  Eigen::Quaternionf quat;
  quat.x() = odometry.pose.pose.orientation.x;
  quat.y() = odometry.pose.pose.orientation.y;
  quat.z() = odometry.pose.pose.orientation.z;
  quat.w() = odometry.pose.pose.orientation.w;
  Eigen::Vector3f rpy = calcul_angles(quat);
  
  if(rpy.z() > M_PI){rpy.z() = rpy.z() - 2*M_PI;}else if(rpy.z() <= - M_PI){rpy.z() = rpy.z() + 2*M_PI;}
  
  pose_ << odometry.pose.pose.position.x,odometry.pose.pose.position.y,rpy.z();
  vel_body_ << odometry.twist.twist.linear.x, odometry.twist.twist.angular.z;
  
  Eigen::Vector3f vitesse_I = quat * vitesse_B;
  
  //Set first reference at first odom value received
  if(!first_odom_){ref_.position.x = pose_.x(); ref_.position.y = pose_.y(); ref_.yaw = rpy.z(); first_odom_ = true;} 
}

void MobileRobotController::updateControl(const ros::TimerEvent& event)
{
	//Time
	double t = ros::Time::now().toSec();
	
	//Publish Pose and Velocity
	BroadcastCurrentOdometry(pose_,vel_body_);
	  
	Eigen::Vector2f cmd;
	cmd << 0.0, 0.0;
	
	applyCtrlFlag_ = com->getMyMode();
	multi_mode_ = com->is_multi();
	
	if (applyCtrlFlag_){
		if (multi_mode_){
			int my_index = com->getMyIndex();
			
			//Get neighbors odometries and status
			neighbors_odometries_ = com->getNeighborsOdometries();
			
			int nb_neighbors = neighbors_odometries_.size();	
			Eigen::MatrixXf neighbors_pose(nb_neighbors,3);
			Eigen::MatrixXf neighbors_vel_body(nb_neighbors,2);
			
			for(int i=0;i<nb_neighbors;i++){
				Eigen::Quaternionf quat;
				quat.x() = neighbors_odometries_[i].pose.pose.orientation.x;
				quat.y() = neighbors_odometries_[i].pose.pose.orientation.y;
				quat.z() = neighbors_odometries_[i].pose.pose.orientation.z;
				quat.w() = neighbors_odometries_[i].pose.pose.orientation.w;
				Eigen::Vector3f rpy = calcul_angles(quat);
				
				neighbors_pose.row(i) << neighbors_odometries_[i].pose.pose.position.x,neighbors_odometries_[i].pose.pose.position.y,rpy.z();
				neighbors_vel_body.row(i) << neighbors_odometries_[i].twist.twist.linear.x, neighbors_odometries_[i].twist.twist.angular.z;
			}
			
			if (velocity_control_){
				cmd = consensusControl(my_index,pose_,vel_body_,neighbors_pose,neighbors_vel_body,nb_neighbors,t); // Velocity controller (with FOV and distance constraints)
			} else {
				cmd = consensusEffortwoVelControl(my_index,pose_,vel_body_,nb_neighbors,t); // Effort PE Controller without velocities
				//~ cmd = consensusEffortwoVelControl(my_index,pose_,vel_body_,nb_neighbors,neighbors_pose,t); // Effort PE Controller
			}
			
		} else {		
			cmd = nominalControl(pose_, ref_, threshold_yaw_, false); // Velocity waypoint controller
			// TO DO: add single robot effort control !!!
		}
	}
	
	if(isnanf(cmd(0))){cmd(0) = 0.0;}
	if(isnanf(cmd(1))){cmd(1) = 0.0;}
	
	if (velocity_control_){
		//saturations (velocity)
		cmd(0) = saturation(cmd(0),vitesse_max_);
		cmd(1) = saturation(cmd(1),vitesse_angulaire_max_);
	  
		//publish velocity control
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = cmd(0);
		cmd_vel.angular.z = cmd(1);
		pub_cmd_vel_.publish(cmd_vel);
	} else {
		//saturations (acceleration)
		float u_v = saturation(cmd(0),vitesse_max_);
		float u_w = saturation(cmd(1),vitesse_angulaire_max_);
		
		// Input conversion
		cmd(0) = 0.5*(mass_*wheel_radius_)*u_v + ((inertia_*wheel_radius_)/(2*wheel_separation_))*u_w;
		cmd(1) = 0.5*(mass_*wheel_radius_)*u_v - ((inertia_*wheel_radius_)/(2*wheel_separation_))*u_w;
		
		//publish effort control
		std_msgs::Float64 left_cmd, right_cmd;
		left_cmd.data = cmd(0);
		right_cmd.data = cmd(1);
		pub_left_effort_.publish(left_cmd);
		pub_right_effort_.publish(right_cmd);
	}
}

void MobileRobotController::callback_reference(const control_position_msgs::ControllerReference& msg)
{
  float distance_waypoints = sqrt((ref_.position.x-msg.position.x)*(ref_.position.x-msg.position.x) + (ref_.position.y-msg.position.y)*(ref_.position.y-msg.position.y));
  if(distance_waypoints > threshold_waypoints_){ //update reference only if far enough from current reference
      ref_.position.x = msg.position.x;
      ref_.position.y = msg.position.y;
      ref_.position.z = msg.position.z; //not used for mobile robot ...
      flag_yaw_ = false;
  }  
  ref_.yaw = msg.yaw; //msg in rad
}

void MobileRobotController::callback_param(mobile_robot_controller::MobileRobotControllerConfig &config, uint32_t level)
{
  kw_ = config.gain_rotation;
  kv_ = config.gain_vitesse;
  kb_ = config.gain_gradient;
  vitesse_max_ = config.vitesse_max;
  vitesse_angulaire_max_ = config.vitesse_angulaire__max;
  threshold_distance_ = config.threshold_distance;
  threshold_yaw_ = config.threshold_yaw;
  threshold_waypoints_ = config.threshold_waypoints;
  
  if(flag_distance_constraint_ != config.distance_constraint){
      flag_distance_constraint_ = config.distance_constraint; 
      if(flag_distance_constraint_ == true){ROS_INFO_STREAM("Distance constraint activated");}else{ROS_INFO_STREAM("Distance constraint de-activated");}
  }
  
  if(flag_fov_constraint_ != config.fov_constraint){
      flag_fov_constraint_ = config.fov_constraint; 
      if(flag_fov_constraint_ == true){ROS_INFO_STREAM("FOV constraint activated");}else{ROS_INFO_STREAM("FOV constraint de-activated");}
  }
  
  dvft_ = config.gain_d_ftv;
  dwft_ = config.gain_d_ftw;
  pvft_ = config.gain_p_ftv;
  pwft_ = config.gain_p_ftw;
  ka_ = config.gain_pe;
  pe_freq_ = config.frequency_pe;
}

Eigen::Vector3f MobileRobotController::calcul_angles(Eigen::Quaternionf q)
{
  Eigen::Vector3f TB_angles;
  TB_angles.x() = atan2(q.y()*q.z() + q.w()*q.x(), 0.5- (q.x()*q.x()+q.y()*q.y())); //roll
  TB_angles.y() = asin(-2.0*(q.x()*q.z()-q.w()*q.y())); //pitch
  TB_angles.z() = atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - (q.y()*q.y() + q.z()*q.z())); //yaw	
  return TB_angles;
}

float MobileRobotController::saturation(float input, float sat)
{
  float output = input;
  if(input > sat){output = sat;}
  else if(input < -sat){output = - sat;}
  return output;	
}

Eigen::Vector2f MobileRobotController::nominalControl(Eigen::Vector3f pose, control_position_msgs::ControllerReference ref,float threshold_yaw,bool prediction_mode)
{  //Basic waypoint proportional control
  
  Eigen::Vector2f control_input; //[v,w] to be returned
  
  //distance remaining
  float error_x = ref.position.x - pose.x();
  float error_y = ref.position.y - pose.y();
  float yaw = pose.z();
  float distance_remaining = sqrt(error_x*error_x + error_y*error_y);
  float error_yaw = 0.0;  
  control_input(0) = 0.0;
  
  //Step 1 = go to the reference position
  if(distance_remaining > threshold_distance_ && (flag_yaw_ == false || prediction_mode))
  {
	float phi = atan2(error_y, error_x); //LOS angle
	error_yaw = phi - yaw;
    if(error_yaw > M_PI){error_yaw = error_yaw - 2*M_PI;}else if(error_yaw <= - M_PI){error_yaw = error_yaw + 2*M_PI;}
	//Speed control (only if orientation error is below some threshold)
	if(fabs(error_yaw) < threshold_yaw){	control_input(0) = kv_*distance_remaining;	}
  }
  else //Step 2 = align on reference yaw
  {
    if(!prediction_mode){flag_yaw_ = true;}
	error_yaw = ref.yaw - yaw;
    if(error_yaw > M_PI){error_yaw = error_yaw - 2*M_PI;}else if(error_yaw <= - M_PI){error_yaw = error_yaw + 2*M_PI;}
	//compensate small distance error during rotation
	float error_projected = (error_x*cos(yaw) + error_y*sin(yaw));
	control_input(0) = kv_*error_projected;
  }
  
  control_input(1) = kw_*error_yaw;	//angular velocity control input
  
  //saturations
  control_input(0) = saturation(control_input(0),vitesse_max_);
  control_input(1) = saturation(control_input(1),vitesse_angulaire_max_);
  
  return control_input;
}

Eigen::Vector2f MobileRobotController::consensusControl(int my_index, Eigen::Vector3f pose, Eigen::Vector2f vel_body, Eigen::MatrixXf neighbors_pose, Eigen::MatrixXf neighbors_vel_body, int nb_neighbors, double t)
{
	Eigen::Vector2f control_input; //[v,w] to be returned
	
	Eigen::MatrixXf E(nb_neighbors+1,nb_neighbors); //Incidence matrix of the graph
	E.row(0) << 1.0, 1.0, 1.0, 0.0, 0.0;
	E.row(1) << -1.0, 0.0, 0.0, 1.0, 1.0;
	E.row(2) << 0.0, -1.0, 0.0, 0.0, 0.0;
	E.row(3) << 0.0, 0.0, -1.0, 0.0, 0.0;
	E.row(4) << 0.0, 0.0, 0.0, -1.0, 0.0;
	E.row(5) << 0.0, 0.0, 0.0, 0.0, -1.0;
	
	Eigen::MatrixXf E_in(nb_neighbors+1,nb_neighbors); //in-Incidence matrix of the graph
	E_in.row(0) << 0.0, 0.0, 0.0, 0.0, 0.0;
	E_in.row(1) << -1.0, 0.0, 0.0, 0.0, 0.0;
	E_in.row(2) << 0.0, -1.0, 0.0, 0.0, 0.0;
	E_in.row(3) << 0.0, 0.0, -1.0, 0.0, 0.0;
	E_in.row(4) << 0.0, 0.0, 0.0, -1.0, 0.0;
	E_in.row(5) << 0.0, 0.0, 0.0, 0.0, -1.0;
	
	Eigen::MatrixXf E_out(nb_neighbors+1,nb_neighbors); //out-Incidence matrix of the graph
	E_out = E - E_in;
	
	Eigen::MatrixXf poses(nb_neighbors+1,3); // all poses
	Eigen::MatrixXf vels(nb_neighbors+1,2); // all velocities		
	
	int nn = 0;
	for(int i = 0;i<nb_neighbors+1;i++){
		if (i == my_index){
			poses.row(i) << pose.transpose();
			vels.row(i) << vel_body.transpose();
		} else {
			poses.row(i) << neighbors_pose.row(nn);
			vels.row(i) << neighbors_vel_body.row(nn);
			nn++;
		}
	}
	
	Eigen::MatrixXf des_disp(nb_neighbors+1,2);
	//des_disp << -0.5, 0.5, -0.5, -0.5, -0.5, 0.5;	
	des_disp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	
	Eigen::MatrixXf des_poses = Eigen::MatrixXf::Zero(nb_neighbors+1,3);
	des_poses.row(0) << poses(0,0), poses(0,1), 0.0;
	for(int i = 0;i<nb_neighbors;i++){
		for(int j = 0;j<nb_neighbors+1;j++){
			des_poses(i+1,0) += E_out(j,i)*(poses(j,0) + des_disp(i,0));
			des_poses(i+1,1) += E_out(j,i)*(poses(j,1) + des_disp(i,1));
			//~ des_poses(i+1,0) += E_out(j,i)*(poses(j,0) + des_disp(i,0)*cosf(poses(j,2)) - des_disp(i,1)*sinf(poses(j,2)));
			//~ des_poses(i+1,1) += E_out(j,i)*(poses(j,1) + des_disp(i,1)*cosf(poses(j,2)) + des_disp(i,0)*sinf(poses(j,2)));
		}
	}
				
	poses = poses - des_poses;

	Eigen::MatrixXf posErrors = Eigen::MatrixXf::Zero(nb_neighbors,2);
	Eigen::VectorXf rho(nb_neighbors);
	Eigen::VectorXf beta(nb_neighbors);
	Eigen::VectorXf alpha(nb_neighbors);
	Eigen::VectorXf alpha_v(nb_neighbors);
	Eigen::VectorXf alpha_e(nb_neighbors);
	
	for(int i = 0;i<nb_neighbors;i++){
		//~ for(int j = 0;j<nb_neighbors+1;j++){
			//~ posErrors(i,0) += E(j,i)*(poses(j,0));
			//~ posErrors(i,1) += E(j,i)*(poses(j,1));
			posErrors(i,0) = -poses(i+1,0);
			posErrors(i,1) = -poses(i+1,1);
		//~ }
		rho(i) = sqrtf(pow(posErrors(i,0),2.0)+pow(posErrors(i,1),2.0));
		alpha(i) = atan2(posErrors(i,1),posErrors(i,0));
		beta(i) = atan2(posErrors(i,1),posErrors(i,0));
		for(int j = 0;j<nb_neighbors+1;j++){
			alpha(i) += E_in(j,i)*poses(j,2);
			beta(i) += -E_out(j,i)*poses(j,2);
		}
	}
	
	for(int i = 0;i<nb_neighbors;i++){
		if(alpha(i) > M_PI){alpha(i) = alpha(i) - 2*M_PI;}else if(alpha(i) <= - M_PI){alpha(i) = alpha(i) + 2*M_PI;}
		if(beta(i) > M_PI){beta(i) = beta(i) - 2*M_PI;}else if(beta(i) <= - M_PI){beta(i) = beta(i) + 2*M_PI;}
	}
	
	for(int i = 0;i<nb_neighbors;i++){
		alpha_v(i) = atan(-kb_*beta(i));
		alpha_e(i) = alpha(i)-alpha_v(i);
	}
	
	Eigen::VectorXf v_lin = Eigen::VectorXf::Zero(nb_neighbors+1);
	Eigen::VectorXf F = Eigen::VectorXf::Zero(nb_neighbors);
	Eigen::VectorXf vk = Eigen::VectorXf::Zero(nb_neighbors);
	Eigen::VectorXf Psi = Eigen::VectorXf::Zero(nb_neighbors);
	Eigen::VectorXf omega = Eigen::VectorXf::Zero(nb_neighbors+1);
	Eigen::VectorXf dVdr(nb_neighbors);
	Eigen::VectorXf dVda(nb_neighbors);
	Eigen::VectorXf wt(nb_neighbors);
	
	Eigen::VectorXf Delta_r(nb_neighbors);
	Delta_r << 8.0, 8.8, 8.7, 9.3, 9.5; // connectivity limits
	
	Eigen::VectorXf Delta_a(nb_neighbors);
	Delta_a << 0.44, 0.44, 0.44, 0.44, 0.44; // connectivity limits
		
	
	if(flag_distance_constraint_){
		//Consensus control WITH connectivty maintenance (distance constraint)
		for(int i = 0;i<nb_neighbors;i++){
			// Gradient of the barrier function (distance constraints)
			dVdr(i) = rho(i) + rho(i)/(pow(Delta_r(i),2.0)-pow(rho(i),2.0));
		}
	} else {
		// Consensus control WITHOUT connectivity maintenance
		for(int i = 0;i<nb_neighbors;i++){
			dVdr(i) = rho(i);
		}
	}
	
	// Linear velocity input
	for(int i = 0;i<nb_neighbors+1;i++){
		for(int j = 0;j<nb_neighbors;j++){
			v_lin(i) += -kv_*E_in(i,j)*sqrt(1+pow(kb_*beta(j),2.0))*dVdr(j);
		}
	}
	
	if(flag_fov_constraint_){
		//Consensus control WITH connectivty maintenance (fov constraint)
		for(int i = 0;i<nb_neighbors;i++){
			// Gradient of the barrier function (fov constraints)
			dVda(i) = alpha_e(i) + 0.05*((alpha_e(i)+alpha_v(i))/(pow(Delta_a(i),2.0)-pow((alpha_e(i)+alpha_v(i)),2.0)) - alpha_v(i)/(pow(Delta_a(i),2.0)-pow(alpha_v(i),2.0)));
			wt(i) = (1.0 + 0.05*(pow(Delta_a(i),2.0)+pow(alpha_v(i),2.0))/pow((pow(Delta_a(i),2.0)-pow(alpha_v(i),2.0)),2.0))*(alpha_e(i)/dVda(i));
		}
	} else {
		// Consensus control WITHOUT connectivity maintenance
		for(int i = 0;i<nb_neighbors;i++){
			dVda(i) = alpha_e(i);
			wt(i) = 1.0;
		}
	}
	
	for(int i = 0;i<nb_neighbors;i++){
		for(int j = 0;j<nb_neighbors+1;j++){
			vk(i) += E_in(j,i)*v_lin(j);
		}
		F(i) = vk(i)*(1+(kb_/(1+pow(kb_*beta(i),2.0)))*wt(i))*(1/rho(i))*sin(alpha_e(i)+alpha_v(i));
		Psi(i) = (-(dVdr(i)/dVda(i))*(cos(alpha_e(i)+alpha_v(i))-cos(alpha_v(i))) + (beta(i)/(rho(i)*dVda(i)))*(sin(alpha_e(i)+alpha_v(i))-sin(alpha_v(i))))*vk(i);
	}
	
	// Angular velocity input
	for(int i = 0;i<nb_neighbors+1;i++){
		for(int j = 0;j<nb_neighbors;j++){
			omega(i) += -E_in(i,j)*(kw_*dVda(j)-Psi(j)-F(j));
		}
	}
	
	// Leader velocity
	v_lin(0) = -kv_*(tanh(2*(t-10))-1);
	omega(0) = kw_*(tanh(2*(t-10))-1)*sinf(0.5*t);
	
	if (my_index == 0){
		control_input(0) = v_lin(my_index);
		control_input(1) = omega(my_index);
		
		rviz_display_robot(pose,0.0,0.0);
	} else {
		rviz_display_robot(pose,Delta_r(my_index-1),Delta_a(my_index-1));
		
		if(isnanf(alpha(my_index-1))){alpha(my_index-1) = 0.0;}
		if(isnanf(rho(my_index-1))){rho(my_index-1) = 0.0;}
		
		std_msgs::Float32MultiArray fov;
		fov.data.push_back(alpha(my_index-1));
		fov.data.push_back(Delta_a(my_index-1));
		fov.data.push_back(-Delta_a(my_index-1));
		fov.data.push_back(rho(my_index-1));
		fov.data.push_back(Delta_r(my_index-1));
		fov_pub_.publish(fov);
		
		if((rho(my_index-1) <= threshold_distance_)){
			//if (fabs(alpha(my_index-1)) <= threshold_distance_/4){
				control_input(0) = 0.0;
				control_input(1) = 0.0;
			//} else {
				//control_input(0) = v_lin(my_index);
				//control_input(1) = omega(my_index);
			//}
		} else {
			control_input(0) = v_lin(my_index);
			control_input(1) = omega(my_index);
		}
	}
	
	return control_input;
}

Eigen::Vector2f MobileRobotController::consensusEffortwoVelControl(int my_index, Eigen::Vector3f pose, Eigen::Vector2f vel_body, int nb_neighbors, double t)
{
	Eigen::Vector2f control_input; //[\tau_left,\tau_right] to be returned
	
	// Desired formation displacements
	Eigen::MatrixXf des_disp(nb_neighbors+1,2);
	des_disp.row(0) << 2.0, 0.0;
	des_disp.row(1) << 1.0, 2.0;
	des_disp.row(2) << -1.0, 2.0;
	des_disp.row(3) << -2.0, 0.0;
	des_disp.row(4) << -1.0, -2.0;
	des_disp.row(5) << 1.0, -2.0;
	
	Eigen::Vector3f bar_pose; // Error desired position
	bar_pose << pose(0) - des_disp(my_index,0), pose(1) - des_disp(my_index,1), pose(2);
	
	float alpha_pe, pe_fnc, freq;		
	//~ pe_fnc = sin(2*M_PI*pe_freq_*t)+cos(3*2*M_PI*pe_freq_*t)-sin(4*2*M_PI*pe_freq_*t)-cos(5*2*M_PI*pe_freq_*t)+3+sin(0.5*2*M_PI*pe_freq_*t); // persistently exciting function
	pe_fnc = 2.5 + sin(2*M_PI*pe_freq_*t) + 0.3*cos(3*2*M_PI*pe_freq_*t) - 0.5*sin(4*2*M_PI*pe_freq_*t) - 0.1*cos(5*2*M_PI*pe_freq_*t)  + sin(.5*2*M_PI*pe_freq_*t);
	alpha_pe = ka_*pe_fnc*(-sinf(bar_pose(2))*(filt_alphavx_(0)-bar_pose(0))+cosf(bar_pose(2))*(filt_alphavy_(0)-bar_pose(1))); // delta-persistently exciting term
	
	float u_v, u_w;	
	u_v = -kv_*(cosf(bar_pose(2))*(bar_pose(0)-filt_alphavx_(0))+sinf(bar_pose(2))*(bar_pose(1)-filt_alphavy_(0))); // Derivative of the linear velocity (input)
	u_w = -kw_*(bar_pose(2)-filt_alphaw_(0)) + alpha_pe; // Derivative of the angular velocity (input)
	
	control_input(0) = u_v;
	control_input(1) = u_w;

	return control_input;
}

Eigen::Vector2f MobileRobotController::consensusEffortControl(int my_index, Eigen::Vector3f pose, Eigen::Vector2f vel_body, int nb_neighbors, Eigen::MatrixXf neighbors_pose, double t)
{
	Eigen::Vector2f control_input; //[\tau_left,\tau_right] to be returned
	
	// Desired formation displacements
	Eigen::MatrixXf des_disp(nb_neighbors+1,2);
	des_disp.row(0) << 2.0, 0.0;
	des_disp.row(1) << 1.0, 2.0;
	des_disp.row(2) << -1.0, 2.0;
	des_disp.row(3) << -2.0, 0.0;
	des_disp.row(4) << -1.0, -2.0;
	des_disp.row(5) << 1.0, -2.0;
	
	Eigen::Vector3f bar_pose; // Error desired position
	bar_pose << pose(0) - des_disp(my_index,0), pose(1) - des_disp(my_index,1), pose(2);
	
	float alpha_pe, pe_fnc, freq;	
	//~ pe_fnc = sin(2*M_PI*pe_freq_*t)+cos(3*2*M_PI*pe_freq_*t)-sin(4*2*M_PI*pe_freq_*t)-cos(5*2*M_PI*pe_freq_*t)+3+sin(0.5*2*M_PI*pe_freq_*t); // persistently exciting function
	pe_fnc = 2.5 + sin(2*M_PI*pe_freq_*t) + 0.3*cos(3*2*M_PI*pe_freq_*t) - 0.5*sin(4*2*M_PI*pe_freq_*t) - 0.1*cos(5*2*M_PI*pe_freq_*t)  + sin(.5*2*M_PI*pe_freq_*t);
	
	Eigen::MatrixXf A(nb_neighbors+1,nb_neighbors+1); //Adjacency matrix of the graph  !!!!!
	A.row(0) << 0.0, 1.0, 0.0, 1.0, 0.0, 1.0;
	A.row(1) << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
	A.row(2) << 0.0, 1.0, 0.0, 1.0, 0.0, 1.0;
	A.row(3) << 1.0, 0.0, 1.0, 0.0, 1.0, 0.0;
	A.row(4) << 0.0, 0.0, 0.0, 1.0, 0.0, 1.0;
	A.row(5) << 1.0, 0.0, 1.0, 0.0, 1.0, 0.0;
	
	//~ Eigen::MatrixXf A(nb_neighbors+1,nb_neighbors+1); //Adjacency matrix of the graph  !!!!!
	//~ A.row(0) << 0.0, 1.0, 0.0, 1.0, 0.0;
	//~ A.row(1) << 1.0, 0.0, 1.0, 0.0, 0.0;
	//~ A.row(2) << 0.0, 1.0, 0.0, 1.0, 0.0;
	//~ A.row(3) << 1.0, 0.0, 1.0, 0.0, 1.0;
	//~ A.row(4) << 0.0, 0.0, 0.0, 1.0, 0.0;
				
	// Consensus term
    float evx = 0.0;
	float evy = 0.0;
	float ew = 0.0;
	for(int i=0;i<nb_neighbors+1;i++){	
		if (i != my_index){
			evx += A(my_index,i)*(bar_pose(0)-(neighbors_pose(i,0)-des_disp(i,0)));
			evy += A(my_index,i)*(bar_pose(1)-(neighbors_pose(i,1)-des_disp(i,1)));
			ew += A(my_index,i)*(bar_pose(2)-neighbors_pose(i,2));
		}		
	}
	alpha_pe = ka_*pe_fnc*(-sinf(bar_pose(2))*evx+cosf(bar_pose(2))*evy); // delta-persistently exciting term
	
	float u_v, u_w;	
	u_v = -dvft_*vel_body(0)-pvft_*(cosf(bar_pose(2))*evx+sinf(bar_pose(2))*evy); // Derivative of the linear velocity (input)
	u_w = -dwft_*vel_body(1)-pwft_*ew + alpha_pe; // Derivative of the angular velocity (input)
	
	// Inputs
	control_input(0) = u_v;
	control_input(1) = u_w;

	return control_input;
}

void MobileRobotController::init_filters(){
	// Initialization of the command filters
	
	int my_index = com->getMyIndex();
	
	// Desired formation displacements
	Eigen::MatrixXf des_disp(6,2);
	//~ Eigen::MatrixXf des_disp(5,2);
	des_disp.row(0) << 2.0, 0.0;
	des_disp.row(1) << 1.0, 2.0;
	des_disp.row(2) << -1.0, 2.0;
	des_disp.row(3) << -2.0, 0.0;
	des_disp.row(4) << -1.0, -2.0;
	des_disp.row(5) << 1.0, -2.0;
	
	Eigen::Vector3f bar_pose; // Error desired position
	bar_pose << pose_(0) - des_disp(my_index,0), pose_(1) - des_disp(my_index,1), pose_(2);
	
	filt_alphavx_ << bar_pose(0), 0.0;
	filt_alphavy_ << bar_pose(1), 0.0;
	filt_alphaw_ << bar_pose(2), 0.0;
	
	t0_ = ros::Time::now().toSec(); //Initial Time
}
	
void MobileRobotController::updateFilter(const ros::TimerEvent& event){
	
	//Time
	double t = ros::Time::now().toSec();
	
	// Filter update variables
	Eigen::Vector2f new_alphavx;
	Eigen::Vector2f new_alphavy;
	Eigen::Vector2f new_alphaw;
	new_alphavx << 0.0, 0.0;
	new_alphavy << 0.0, 0.0;
	new_alphaw << 0.0, 0.0;
	
	if (applyCtrlFlag_){
		if (multi_mode_){
			if (!velocity_control_){
				// Initialize filter
				if (!filters_initialized_){
					init_filters();
					filters_initialized_ = true;
				}

				int my_index = com->getMyIndex();
				
				//Get neighbors' filter variables
				neighbors_filters_ = com->getNeighborsFilters();
				
				int nb_neighbors = neighbors_filters_.size();	
				Eigen::MatrixXf neighbors_filter_var(nb_neighbors,3);
				
				for(int i=0;i<nb_neighbors;i++){			
					neighbors_filter_var.row(i) << neighbors_filters_[i].data[0],neighbors_filters_[i].data[1],neighbors_filters_[i].data[2];
				}		
			
				Eigen::MatrixXf A(nb_neighbors+1,nb_neighbors+1); //Adjacency matrix of the graph  !!!!!
				A.row(0) << 0.0, 1.0, 0.0, 1.0, 0.0, 1.0;
				A.row(1) << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
				A.row(2) << 0.0, 1.0, 0.0, 1.0, 0.0, 1.0;
				A.row(3) << 1.0, 0.0, 1.0, 0.0, 1.0, 0.0;
				A.row(4) << 0.0, 0.0, 0.0, 1.0, 0.0, 1.0;
				A.row(5) << 1.0, 0.0, 1.0, 0.0, 1.0, 0.0;
				
				//~ Eigen::MatrixXf A(nb_neighbors+1,nb_neighbors+1); //Adjacency matrix of the graph  !!!!!
				//~ A.row(0) << 0.0, 1.0, 0.0, 1.0, 0.0;
				//~ A.row(1) << 1.0, 0.0, 1.0, 0.0, 0.0;
				//~ A.row(2) << 0.0, 1.0, 0.0, 1.0, 0.0;
				//~ A.row(3) << 1.0, 0.0, 1.0, 0.0, 1.0;
				//~ A.row(4) << 0.0, 0.0, 0.0, 1.0, 0.0;
				
				// Consensus term
				float evx = 0.0;
				float evy = 0.0;
				float ew = 0.0;
				for(int i=0;i<nb_neighbors+1;i++){	
					if (i != my_index){
						evx += A(my_index,i)*(filt_alphavx_(0)-neighbors_filter_var(i,0));
						evy += A(my_index,i)*(filt_alphavy_(0)-neighbors_filter_var(i,1));
						ew += A(my_index,i)*(filt_alphaw_(0)-neighbors_filter_var(i,2));
					}		
				}
					
				Eigen::MatrixXf des_disp(nb_neighbors+1,2);
				des_disp.row(0) << 2.0, 0.0;
				des_disp.row(1) << 1.0, 2.0;
				des_disp.row(2) << -1.0, 2.0;
				des_disp.row(3) << -2.0, 0.0;
				des_disp.row(4) << -1.0, -2.0;
				des_disp.row(5) << 1.0, -2.0;
					
				Eigen::Vector3f bar_pose;
				bar_pose << pose_(0) - des_disp(my_index,0), pose_(1) - des_disp(my_index,1), pose_(2);
					
				// Command filter for the linear velocity
				new_alphavx(0) = filt_alphavx_(0) + Te_filter_*filt_alphavx_(1);
				new_alphavx(1) = filt_alphavx_(1) + Te_filter_*(- dvft_*filt_alphavx_(1) - kv_*(filt_alphavx_(0)-bar_pose(0)) - pvft_*evx); 
				new_alphavy(0) = filt_alphavy_(0) + Te_filter_*filt_alphavy_(1);
				new_alphavy(1) = filt_alphavy_(1) + Te_filter_*(- dvft_*filt_alphavy_(1) - kv_*(filt_alphavy_(0)-bar_pose(1)) - pvft_*evy); 
					
				// Command filter for the angular velocity
				new_alphaw(0) = filt_alphaw_(0) + Te_filter_*filt_alphaw_(1);
				new_alphaw(1) = filt_alphaw_(1) + Te_filter_*(- dwft_*filt_alphaw_(1) - kw_*(filt_alphaw_(0)-bar_pose(2)) - pwft_*ew); 
			
			}
		}
	} else {
		filters_initialized_ = false;
	}
	
	// Assign new values
	filt_alphavx_ << new_alphavx(0), new_alphavx(1);
	filt_alphavy_ << new_alphavy(0), new_alphavy(1);
	filt_alphaw_ << new_alphaw(0), new_alphaw(1);

	// Publish filter variables
	std_msgs::Float32MultiArray ft_msg;
	ft_msg.data.push_back(new_alphavx(0));
	ft_msg.data.push_back(new_alphavy(0));
	ft_msg.data.push_back(new_alphaw(0));
	pub_filters_.publish(ft_msg);	
	//~ ROS_INFO_STREAM("I'm here 3" << ft_msg);
}

void MobileRobotController::BroadcastCurrentOdometry(Eigen::Vector3f pose, Eigen::Vector2f vel_body)
{
  //publish Odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "world";
  
  odom_msg.pose.pose.position.x = pose(0);
  odom_msg.pose.pose.position.y = pose(1);
  odom_msg.pose.pose.position.z = height_;
  
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(0.5*pose(2));
  odom_msg.pose.pose.orientation.w = cos(0.5*pose(2));
  
  odom_msg.twist.twist.linear.x = vel_body(0);
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vel_body(1);
  
  pub_current_odometry_.publish(odom_msg);
}

void MobileRobotController::rviz_display_robot(Eigen::Vector3f pose, float range, float fov_angle)
{
  visualization_msgs::MarkerArray marker;
  marker.markers.resize(2); //respectively for positions and FOV
  //sub-markers
  visualization_msgs::Marker marker_pos;
  visualization_msgs::Marker marker_fov;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "world";
  
  //position marker
  marker_pos.header = header;
  marker_pos.ns = "fleet";
  marker_pos.action = visualization_msgs::Marker::ADD;
  marker_pos.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_pos.pose.orientation.w = 1.0;
  marker_pos.scale.x = 0.2;
  marker_pos.scale.y = 0.2;
  marker_pos.scale.z = 0.2;
  marker_pos.color.r = 0.0;
  marker_pos.color.b = 1.0;
  marker_pos.color.g = 0.0;
  marker_pos.color.a = 0.8;
	
  geometry_msgs::Point p;
  p.x = pose(0);
  p.y = pose(1);
  p.z = height_;
  marker_pos.points.push_back(p);
  
  marker.markers[0] = marker_pos;	

  //FOV marker
  marker_fov.header = header;
  marker_fov.ns = "fov";
  marker_fov.action = visualization_msgs::Marker::ADD;
  marker_fov.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker_fov.pose.orientation.w = 1.0;
  marker_fov.points.resize(3);
  marker_fov.scale.x = 1.0;
  marker_fov.scale.y = 1.0;
  marker_fov.scale.z = 1.0;
  marker_fov.color.r = 0.0;
  marker_fov.color.b = 0.0;
  marker_fov.color.g = 1.0;
  marker_fov.color.a = 0.2;
  
  marker_fov.points.push_back(p);
  double yaw = pose(2); 
  p.x = pose(0) + cos(yaw-fov_angle)*range/cos(fov_angle);
  p.y = pose(1) + sin(yaw-fov_angle)*range/cos(fov_angle);
  marker_fov.points.push_back(p);
  p.x = pose(0) + cos(yaw+fov_angle)*range/cos(fov_angle);
  p.y = pose(1) + sin(yaw+fov_angle)*range/cos(fov_angle);
  marker_fov.points.push_back(p);

  marker.markers[1] = marker_fov;
  
  rviz_marker_pub_.publish(marker);
}
