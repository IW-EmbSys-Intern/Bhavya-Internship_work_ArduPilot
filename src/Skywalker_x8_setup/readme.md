# Complete github repo
https://github.com/ArduPilot/SITL_Models

Clone this and then add the files in my folder to the respective folder of the cloned repo
this will add camera and other params to the plane (SKYWALKER_X8)


# Steps to run:
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds

gz sim -v4 -r skywalker_x8_runway.sdf

# In ardupilot directory
sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8.param --console --map

# To run mavproxy
ros2 launch mavros apm.launch fcu_url:=udp://:14550@

# To verify camera exist
gz topic -l | grep camera

# Run the parameter bridge with the exact source topic and mapping
ros2 run ros_gz_bridge parameter_bridge \
"/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
--ros-args \
-r /world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/image:=/camera/image_raw


# Optional (recommended) (USE CORRECT TOPICS)
ros2 run ros_gz_bridge parameter_bridge \
/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
--ros-args \
-r /camera/camera_info:=/world/runway/model/skywalker_x8/link/camera_link/sensor/front_camera/camera_info

# Feed
ros2 run rqt_image_view rqt_image_view







# FOR MINI_TALON_VTAIL:-
sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --console --map --out=udp:127.0.0.1:14550

# Change Servo 2 and Servo 4 to V-Tail functions (typical Mini Talon mapping)
param set SERVO2_FUNCTION 79  # 79 = VTailLeft
param set SERVO4_FUNCTION 80  # 80 = VTailRight

# Save the changes to the simulated EEPROM
param save
