Header header
int32 controller_id

# Message should be expressed in the frame_id specified in the header.
# frame_id == local for relative position/...

geometry_msgs/Vector3 position      # [m]
geometry_msgs/Vector3 velocity      # [m/s]
geometry_msgs/Vector3 acceleration  # [m/s^2]
#geometry_msgs/Vector3 jerk          # [m/s^3]
#geometry_msgs/Vector3 snap          # [m/s^4]
float64 ellipse_width # demi largeur ellipse (m) + => sens trigo, - => sens horaire
float64 yaw                         # [rad]
float64 yaw_rate                    # [rad/s]
