from dronekit import connect, VehicleMode, mavutil
import time
import math

vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt

        if alt >= target_altitude * 0.95:
            break

        time.sleep(1)

def send_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        0
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

arm_and_takeoff(50)

radius = 50.0
angular_velocity = 0.1
dt = 0.1

for i in range(int(radius / 1.0 / dt)):
    send_ned_velocity(10.0, 0.0, 0.0)
    time.sleep(dt)

start_time = time.time()

# while True:
#     t = time.time() - start_time

#     theta = angular_velocity * t

#     vx = -radius * angular_velocity * math.sin(theta)
#     vy = radius * angular_velocity * math.cos(theta)

#     send_ned_velocity(vx, vy, 0)

#     time.sleep(dt)

while True:
    t = time.time() - start_time
    theta = angular_velocity * t

    # stop after 1 full revolution
    if theta >= 2 * math.pi:
        break

    vx = -radius * angular_velocity * math.sin(theta)
    vy = radius * angular_velocity * math.cos(theta)

    send_ned_velocity(vx, vy, 0)
    time.sleep(dt)

vehicle.mode = VehicleMode("RTL")

while vehicle.mode.name != "RTL":
    time.sleep(1)

print("Switched to RTL")