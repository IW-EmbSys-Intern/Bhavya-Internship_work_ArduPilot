from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil
import time
import numpy as np
import math
import pyproj
from math import radians, cos, sin, asin, sqrt
vehicle = connect('127.0.0.1:14550', wait_ready=False)

def arm_and_takeoff(aTargetAltitude):
    print ("Basic pre-arm checks")

    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(2)
    print ("Arming motors")

    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)

        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
arm_and_takeoff(50)

Lat = [-35.36176764,  -35.36087144, -35.36086090,  -35.36178873, -35.36176764]
Lon = [149.16297738, 149.16301617, 149.16520111, 149.16517525, 149.16297738]
# D = []

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

def calculate_bearing(homeLatitude, homeLongitude, destinationLatitude, destinationLongitude):
    geod = pyproj.Geod(ellps='WGS84')
    fwd, back, dist = geod.inv(homeLongitude, homeLatitude, destinationLongitude, destinationLatitude)
    return fwd

def haversine(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    a = sin((lat2-lat1)/2)**2 + cos(lat1) * cos(lat2) * sin((lon2-lon1)/2)**2
    c = 2 * asin(sqrt(a))
    return c * 6371 * 1000

def print_telemetry_data():
    print(f"Latitude: {vehicle.location.global_frame.lat}")
    print(f"Longitude: {vehicle.location.global_frame.lon}")
    print(f"Altitude: {vehicle.location.global_frame.alt}")
    print(f"Airspeed: {vehicle.airspeed}")
    print(f"Groundspeed: {vehicle.groundspeed}")
    print(f"Mode: {vehicle.mode.name}")

# for x in range(len(Lat)-1):
#     bearing = calculate_bearing(Lat[x], Lon[x], Lat[x+1], Lon[x+1])
#     D.append(math.radians(bearing))

for i in range(len(Lat)):
    while True:
        d = haversine(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat, Lon[i], Lat[i])
        if d <= 5:
            print("Reached waypoint " + str(i+1))
            break
        bearing = calculate_bearing(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, Lat[i], Lon[i])
        vel_x = (10) * math.cos(np.radians(bearing))
        vel_y = (10) * math.sin(np.radians(bearing))
        send_ned_velocity(vel_x, vel_y, 0)
        vehicle.gimbal.rotate(-90, 0, 0)
        print_telemetry_data()
        print(math.sqrt(vel_x**2 + vel_y**2))
        time.sleep(1)

time.sleep(1)
print('Path Complete')
vehicle.mode = VehicleMode("RTL")
print("RTL")