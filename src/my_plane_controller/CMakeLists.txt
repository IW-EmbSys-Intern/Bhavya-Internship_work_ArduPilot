cmake_minimum_required(VERSION 3.8)
project(my_plane_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Generate the Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetDetection.msg"
  "msg/TargetGPS.msg"
  DEPENDENCIES std_msgs
)

# 2. Correctly install the Python package directory 
# We use install(DIRECTORY...) instead of the conflicting ament_python_install_package macro
install(DIRECTORY ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install your executable scripts (YOLO & Flight Node)
# Make sure these filenames match your actual python script names!
install(PROGRAMS
  ${PROJECT_NAME}/yolo_sample.py
  ${PROJECT_NAME}/yolo_static_node.py
  ${PROJECT_NAME}/geolocator_node.py
  ${PROJECT_NAME}/kamikaze.py
  ${PROJECT_NAME}/simple_kamikaze.py
  ${PROJECT_NAME}/square.py
  ${PROJECT_NAME}/circle.py
  ${PROJECT_NAME}/takeoff_using_mavros.py
  ${PROJECT_NAME}/takeoff_using_mavros2.py
  ${PROJECT_NAME}/takeoff_with_geolocation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
