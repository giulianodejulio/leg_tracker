#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import rospy
from leg_tracker.msg import People  # Assuming People is a message type from leg_tracker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

# Dictionary to store history of positions and times for each person
person_history = {}
# Global variable for velocity plot
velocity_history = {"time": [], "vx": [], "vy": []}

def plot_2d_traj(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        for person in msg.people:  # Iterate through the list of people
            plt.plot(person.pose.position.y, person.pose.position.x, '*', label=f"Person {person.id}")
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

def plot_x_and_y_in_time(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9

        for person in msg.people:
            person_id = person.id

            # Initialize history for new persons
            if person_id not in person_history:
                person_history[person_id] = {"x": [], "y": [], "time": []}

            # Append new position and time to the history
            person_history[person_id]["x"].append(person.pose.position.x)
            person_history[person_id]["y"].append(person.pose.position.y)
            person_history[person_id]["time"].append(time)

            # Keep history size reasonable (optional)
            if len(person_history[person_id]["time"]) > 1000:  # Arbitrary max history size
                person_history[person_id]["x"].pop(0)
                person_history[person_id]["y"].pop(0)
                person_history[person_id]["time"].pop(0)

        # Plot all tracked persons' trajectories over time
        plt.clf()  # Clear the previous plot
        for person_id, history in person_history.items():
            plt.plot(history["time"], history["x"], label=f"Person {person_id} (x)")
            plt.plot(history["time"], history["y"], label=f"Person {person_id} (y)")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

def plot_dirty_derivative_in_time(msg):
    """
    Callback to plot velocity over time.
    """
    global velocity_history, counter
    if counter % 10 == 0:
        # Extract timestamp
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9

        # Extract velocity components
        vx = msg.point.x
        vy = msg.point.y

        # Append new data to velocity history
        velocity_history["time"].append(time)
        velocity_history["vx"].append(vx)
        velocity_history["vy"].append(vy)

        # Keep history size reasonable
        if len(velocity_history["time"]) > 1000:  # Arbitrary max history size
            velocity_history["time"].pop(0)
            velocity_history["vx"].pop(0)
            velocity_history["vy"].pop(0)

        # Plot velocity over time
        plt.clf()  # Clear the previous plot
        plt.plot(velocity_history["time"], velocity_history["vx"], label="Velocity X")
        plt.plot(velocity_history["time"], velocity_history["vy"], label="Velocity Y")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.title("Velocity Over Time")
        plt.legend()
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

def plot_kalman_velocity_in_time(msg):
    """
    Callback to plot the velocity field over time for People messages.
    """
    global velocity_history, counter
    if counter % 10 == 0:
        # Extract timestamp
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9

        # Iterate through the people in the People message and extract velocity
        for person in msg.people:
            person_name = person.name
            vx = person.velocity.x
            vy = person.velocity.y

            # Append new velocity data to the history
            velocity_history["time"].append(time)
            velocity_history["vx"].append(vx)
            velocity_history["vy"].append(vy)

            # Keep history size reasonable
            if len(velocity_history["time"]) > 1000:  # Arbitrary max history size
                velocity_history["time"].pop(0)
                velocity_history["vx"].pop(0)
                velocity_history["vy"].pop(0)

        # Plot the velocity field (velocity components) over time
        plt.clf()  # Clear the previous plot
        plt.plot(velocity_history["time"], velocity_history["vx"], label="Kalman Velocity X")
        plt.plot(velocity_history["time"], velocity_history["vy"], label="Kalman Velocity Y")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.title("Kalman Filtered Velocity Over Time")
        plt.legend()
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

def plot_cmd_vel(msg):
    """
    Callback to plot the commanded velocity (cmd_vel) over time.
    """
    global velocity_history, counter
    if counter % 10 == 0:
        # Extract timestamp
        time = rospy.get_time()

        # Extract velocity components
        linear_velocity = msg.linear.x  # Assuming forward velocity is along x-axis
        angular_velocity = msg.angular.z  # Assuming angular velocity is around z-axis

        # Append new velocity data to the history
        velocity_history["time"].append(time)
        velocity_history["vx"].append(linear_velocity)
        velocity_history["vy"].append(angular_velocity)

        # Keep history size reasonable
        if len(velocity_history["time"]) > 1000:  # Arbitrary max history size
            velocity_history["time"].pop(0)
            velocity_history["vx"].pop(0)
            velocity_history["vy"].pop(0)

        # Plot commanded velocities over time
        plt.clf()  # Clear the previous plot
        plt.plot(velocity_history["time"], velocity_history["vx"], label="Linear Velocity (x)")
        plt.plot(velocity_history["time"], velocity_history["vy"], label="Angular Velocity (z)")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity")
        plt.title("Commanded Velocities Over Time (cmd_vel)")
        plt.legend()
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    # rospy.Subscriber("/people_tracked", PersonArray, plot_2d_traj)
    # rospy.Subscriber("/people_tracked", PersonArray, plot_x_and_y_in_time)
    # rospy.Subscriber("/dirty_derivative", PointStamped, plot_dirty_derivative_in_time)
    # rospy.Subscriber("/people", People, plot_kalman_velocity_in_time)
    rospy.Subscriber("/moca_red/move_base/cmd_vel", Twist, plot_cmd_vel)

    plt.ion()
    plt.show()
    rospy.spin()
