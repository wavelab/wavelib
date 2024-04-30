#!/usr/bin/env python3
import csv
import matplotlib.pylab as plt

QUADROTOR_ATTITUDE_CONTROLLER_OUTPUT = "/tmp/quadrotor_attitude_controller.dat"
QUADROTOR_POSITION_CONTROLLER_OUTPUT = "/tmp/quadrotor_position_controller.dat"
QUADROTOR_VELOCITY_CONTROLLER_OUTPUT = "/tmp/quadrotor_velocity_controller.dat"


def plot_quadrotor_attitude_data():
    attitude_data = {
        "t": [],
        "roll": [], "pitch": [], "yaw": []
    }

    attitude_output = open(QUADROTOR_ATTITUDE_CONTROLLER_OUTPUT, "r")
    attitude_reader = csv.reader(attitude_output, delimiter=",")
    next(attitude_reader)

    for row in attitude_reader:
        attitude_data["t"].append(float(row[0]))
        attitude_data["roll"].append(float(row[1]))
        attitude_data["pitch"].append(float(row[2]))
        attitude_data["yaw"].append(float(row[3]))

    plt.plot(attitude_data["t"], attitude_data["roll"], label="roll")
    plt.plot(attitude_data["t"], attitude_data["pitch"], label="pitch")
    plt.plot(attitude_data["t"], attitude_data["yaw"], label="yaw")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Angle (degrees)")
    plt.legend(loc=0)
    plt.show()


def plot_quadrotor_position_data():
    position_data = {
        "t": [],
        "roll": [], "pitch": [], "yaw": [],
        "x": [], "y": [], "z": []
    }

    position_output = open(QUADROTOR_POSITION_CONTROLLER_OUTPUT, "r")
    position_reader = csv.reader(position_output, delimiter=",")
    next(position_reader)

    for row in position_reader:
        position_data["t"].append(float(row[0]))
        position_data["roll"].append(float(row[1]))
        position_data["pitch"].append(float(row[2]))
        position_data["yaw"].append(float(row[3]))
        position_data["x"].append(float(row[4]))
        position_data["y"].append(float(row[5]))
        position_data["z"].append(float(row[6]))

    plt.subplot(2, 1, 1)
    plt.plot(position_data["t"], position_data["roll"], label="roll")
    plt.plot(position_data["t"], position_data["pitch"], label="pitch")
    plt.plot(position_data["t"], position_data["yaw"], label="yaw")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Angle (degrees)")
    plt.legend(loc=0)

    plt.subplot(2, 1, 2)
    plt.plot(position_data["t"], position_data["x"], label="x")
    plt.plot(position_data["t"], position_data["y"], label="y")
    plt.plot(position_data["t"], position_data["z"], label="z")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Position (meters)")
    plt.legend(loc=0)

    plt.show()


def plot_quadrotor_velocity_data():
    data = {
        "t": [],
        "roll": [], "pitch": [], "yaw": [],
        "x": [], "y": [], "z": [],
        "vx": [], "vy": [], "vz": []
    }

    output = open(QUADROTOR_POSITION_CONTROLLER_OUTPUT, "r")
    reader = csv.reader(output, delimiter=",")
    next(reader)

    for row in reader:
        data["t"].append(float(row[0]))
        data["roll"].append(float(row[1]))
        data["pitch"].append(float(row[2]))
        data["yaw"].append(float(row[3]))
        data["x"].append(float(row[4]))
        data["y"].append(float(row[5]))
        data["z"].append(float(row[6]))
        data["vx"].append(float(row[7]))
        data["vy"].append(float(row[8]))
        data["vz"].append(float(row[9]))

    # plot attitude data
    plt.subplot(3, 1, 1)
    plt.plot(data["t"], data["roll"], label="roll")
    plt.plot(data["t"], data["pitch"], label="pitch")
    plt.plot(data["t"], data["yaw"], label="yaw")

    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend(loc=0)

    # plot position data
    plt.subplot(3, 1, 2)
    plt.plot(data["t"], data["x"], label="x")
    plt.plot(data["t"], data["y"], label="y")
    plt.plot(data["t"], data["z"], label="z")

    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.legend(loc=0)

    # plot velocity data
    plt.subplot(3, 1, 3)
    plt.plot(data["t"], data["vx"], label="vx")
    plt.plot(data["t"], data["vy"], label="vy")
    plt.plot(data["t"], data["vz"], label="vz")

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (ms^-1)")
    plt.legend(loc=0)

    plt.show()


if __name__ == "__main__":
    plot_quadrotor_attitude_data()
    plot_quadrotor_position_data()
    plot_quadrotor_velocity_data()
