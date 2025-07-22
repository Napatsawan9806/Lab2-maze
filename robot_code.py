from controller import Robot
import math
import csv
import os

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    wheel_radius = 0.0205
    axle_length = 0.053

    # Motors
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Proximity sensors
    sensors = []
    for i in range(8):
        s = robot.getDevice(f'ps{i}')
        s.enable(timestep)
        sensors.append(s)

    # Position sensors
    l_enc = left_motor.getPositionSensor()
    r_enc = right_motor.getPositionSensor()
    l_enc.enable(timestep)
    r_enc.enable(timestep)

    # Logging setup
    data_log = []
    wall_follow_mode = None
    x, y, theta = 0.0, 0.0, 0.0
    last_l = 0.0
    last_r = 0.0

    while robot.step(timestep) != -1:
        # Sensor values
        ps_vals = [s.getValue() for s in sensors]

        # Wall follow detection
        if wall_follow_mode is None:
            left_wall = ps_vals[5] > 80 or ps_vals[6] > 80
            right_wall = ps_vals[2] > 80 or ps_vals[1] > 80
            front_wall = ps_vals[7] > 80 or ps_vals[0] > 80

            if left_wall and not right_wall:
                wall_follow_mode = 'left'
            elif right_wall and not left_wall:
                wall_follow_mode = 'right'
            elif left_wall and right_wall:
                wall_follow_mode = 'left'
            elif front_wall:
                wall_follow_mode = 'left'

        # Wall following logic
        if wall_follow_mode == 'left':
            left = ps_vals[5] > 80
            corner = ps_vals[6] > 80
            front = ps_vals[7] > 80

            if front:
                state = 'turn_right'
            elif corner:
                state = 'steer_right'
            elif left:
                state = 'drive_forward'
            else:
                state = 'turn_left'

            if state == 'turn_right':
                l_speed, r_speed = max_speed, -max_speed
            elif state == 'steer_right':
                l_speed, r_speed = max_speed, max_speed / 8
            elif state == 'turn_left':
                l_speed, r_speed = max_speed / 8, max_speed
            else:
                l_speed, r_speed = max_speed, max_speed

        elif wall_follow_mode == 'right':
            right = ps_vals[2] > 80
            corner = ps_vals[1] > 80
            front = ps_vals[0] > 80

            if front:
                state = 'turn_left'
            elif corner:
                state = 'steer_left'
            elif right:
                state = 'drive_forward'
            else:
                state = 'turn_right'

            if state == 'turn_left':
                l_speed, r_speed = -max_speed, max_speed
            elif state == 'steer_left':
                l_speed, r_speed = max_speed / 8, max_speed
            elif state == 'turn_right':
                l_speed, r_speed = max_speed, max_speed / 8
            else:
                l_speed, r_speed = max_speed, max_speed
        else:
            l_speed = r_speed = max_speed

        left_motor.setVelocity(l_speed)
        right_motor.setVelocity(r_speed)

        # Odometer update
        l_now = l_enc.getValue()
        r_now = r_enc.getValue()
        dl = (l_now - last_l) * wheel_radius
        dr = (r_now - last_r) * wheel_radius
        last_l, last_r = l_now, r_now

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / axle_length
        theta += dtheta
        x += dc * math.cos(theta)
        y += dc * math.sin(theta)

        # Log every timestep (Webots time)
        sim_time = round(robot.getTime(), 3)  # Time in seconds
        data_log.append([
            sim_time,
            round(x, 4),
            round(y, 4),
            round(math.degrees(theta), 2),
            round(ps_vals[5], 1),
            round(ps_vals[2], 1),
            round(ps_vals[7], 1)
        ])

    # Save when simulation stops
    save_path = os.path.join(os.path.dirname(__file__), "robot_tracking_log.csv")
    with open(save_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(['time_sec', 'x', 'y', 'theta_deg', 'left_dist', 'right_dist', 'front_dist'])
        writer.writerows(data_log)
    print(f"✅ Data saved to {save_path}")

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
