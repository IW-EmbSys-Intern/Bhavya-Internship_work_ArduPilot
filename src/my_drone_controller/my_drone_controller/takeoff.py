#!/usr/bin/env python3

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# vehicle = connect('127.0.0.1:5760', wait_ready=True)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):

    print("Waiting for drone")

    while not vehicle.is_armable:
        print(' Waiting for GPS,EKF')
        time.sleep(1)

    print('Guided mode')
    vehicle.mode = VehicleMode("GUIDED")

    vehicle.armed = True
    print("Armed")

    while not vehicle.armed:
        print('waiting for arming')
        time.sleep(1)

    print(f"Taking off to {target_altitude} meters")

    vehicle.simple_takeoff(target_altitude)
    time.sleep(1)

    while True:
        alt=vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f}m")

        if alt>=target_altitude *0.98:
            print('Target height reached')
            break

        time.sleep(1)

def condition_yaw(heading, relative=False, speed=30):

    is_relative = 1 if relative else 0

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,      # param 1: target angle
        speed,        # param 2: yaw speed deg/s
        1,            # param 3: direction (1=cw, -1=ccw)
        is_relative,  # param 4: relative or absolute
        0, 0, 0
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto_point(lat, lon, alt):
    target = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target,airspeed=20,groundspeed=30)

    while True:
        current = vehicle.location.global_relative_frame
        dist_lat = abs(current.lat - lat)
        dist_lon = abs(current.lon - lon)

        if dist_lat < 0.00001 and dist_lon < 0.00001:
            print("Reached target point")
            break

        time.sleep(1)

def offset_latlon(lat, lon, dx, dy):
    new_lat = lat + (dy / 111320.0)
    new_lon = lon + (dx / (111320.0 * math.cos(math.radians(lat))))
    return new_lat, new_lon

def send_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def circle(center_lat, center_lon, radius=1000, altitude=50, duration=70):
    print("Starting circle...")

    start_time = time.time()
    angle = 0
    t=0

    while time.time() - start_time < duration:
        t+=1
        print(t)

        # angular step (controls speed of orbit)
        angle += 0.05

        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)

        target_lat, target_lon = offset_latlon(center_lat, center_lon, dx, dy)

        target = LocationGlobalRelative(target_lat, target_lon, altitude)
        vehicle.simple_goto(target, groundspeed=10)

        time.sleep(0.5)

    print("Circle complete")

def square(coordinates, height):
    for lat, lon in coordinates:
        goto_point(lat,lon,height)

    print("Square complete")

arm_and_takeoff(30)
time.sleep(2)

# goto_point(-35.36137415, 149.16354399,50)
# time.sleep(1)

condition_yaw(0)
print("Yaw corrected to head NORTH")
time.sleep(8)

coordinates = [( -35.36176242, 149.16294813 ), ( -35.36087677, 149.16293521 ), (-35.36085569, 149.16405998  ), ( -35.36175187, 149.16402120  ), ( -35.36176242, 149.16294813 )]

square(coordinates,50)
print("Square completed")
time.sleep(2)

print('Going for second mission')

circle(-35.36089811, 149.16735303)
print('circle complete')
time.sleep(1)

# go_east(50, 30)
# time.sleep(3)

print("Returning home...")
vehicle.mode = VehicleMode("RTL")

vehicle.close()