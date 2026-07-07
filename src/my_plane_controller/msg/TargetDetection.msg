# TargetDetection.msg
std_msgs/Header header
bool target_found
float64 pixel_x       # Center of bounding box
float64 pixel_y
float64 gps_lat       # Optional: If you are doing geo-localization
float64 gps_lon
string class_name
float32 confidence
float32 zoom_factor   # Digital zoom applied to the frame before detection.
                      # geolocator_node.py multiplies fx/fy from camera_info
                      # by this before projecting the pixel, since cropping
                      # the frame and resizing it back up increases the
                      # effective focal length by this factor. Set to 1.0
                      # if no digital zoom is applied.