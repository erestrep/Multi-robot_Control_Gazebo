# velocity in m/s
float64 command_velocity_ref
float64 pilot_velocity_ref
float64 current_velocity
float64 current_velocity_max
float64 velocity_max

# yaw in rad
float64 command_yaw_ref
float64 pilot_yaw_ref
float64 current_yaw
float64 traj_yaw
float64 allow_yaw_range
string  yaw_mode

float64 d_current_metric
# value normalized between 0(d_obs=infinity) and 1(dobs=0)
float64 d_current
float64 d_activation
float64 d_des
float64 d_safe

# distance on trajectory
float64 traj_percent
string traj_label