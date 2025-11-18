import torch
from rsl_rl.modules import ActorCritic

import os, time
import numpy as np
# from math import atan2, sqrt

import dynamixel_python as dp


import matplotlib.pyplot as plt







# Initialise motor

def initialise():

    DM = dp.DynamixelManager(baud_rate=115200, usb_port='/dev/ttyUSB0')
    motor = DM.add_dynamixel("motor_test", 3, "xl430-w250")

    DM.init()
    if not DM.ping_all():
        raise BaseException("motor isn't configured correctly")

    # set operating mode to position control
    motor.set_operating_mode(3)

    motor.set_led(True)
    print(f"Motor n°{motor.id} ... Ready!")
    time.sleep(0.3)

    # enable motor control
    motor.set_torque_enable(True)
    motor.set_profile_velocity(100)
    motor.set_goal_position(2048)
    time.sleep(1)

    

    return motor





def shutdown(motor):
    motor.set_goal_position(2048)
    motor.set_torque_enable(False)
    motor.set_led(False)





def test_pos(motor):
    # motor.set_goal_position(1934)
    motor.set_goal_position(1750)
    time.sleep(1)
    pos = []
    timer = []

    # motor.set_goal_position(2162)
    motor.set_goal_position(2350)
    t0 = time.time()
    for k in range(150):
        pos.append(motor.get_present_position())
        timer.append(time.time() - t0)

    return pos, timer



def test_vel(motor):
    motor.set_goal_position(1750)
    time.sleep(1)
    vel = []
    timer = []

    motor.set_goal_position(2350)
    t0 = time.time()
    for k in range(150):
        v = motor.get_present_velocity()
        if v > 1000:
            v = 0
        vel.append(v)
        timer.append(time.time() - t0)

    return vel, timer



def test_load(motor):
    motor.set_goal_position(1750)
    time.sleep(1)
    load = []
    timer = []

    motor.set_goal_position(2350)
    t0 = time.time()
    for k in range(150):
        l = motor.get_present_load()
        if l > 5000:
            l = 0
        load.append(l)
        timer.append(time.time() - t0)

    return load, timer



def test_shutdown(motor):
    print(motor.get_shutdown())




def plot_courbe(X, Y, filename):
    plt.figure()
    plt.plot(X, Y)
    plt.title("test")
    plt.savefig(filename)
    plt.close()




def test_pos_vel_load(motor):

    fig_pos, ax_pos = plt.subplots()
    fig_vel, ax_vel = plt.subplots()
    fig_load, ax_load = plt.subplots()

    ax_pos.set_title("position")
    ax_vel.set_title("velocity")
    ax_load.set_title("load")


    for k in range(6):
        input(" ? ")

        pos, timer = test_pos(motor)
        ax_pos.plot(timer, pos)

        vel, timer = test_vel(motor)
        ax_vel.plot(timer, vel)

        load, timer = test_load(motor)
        ax_load.plot(timer, load)

    fig_pos.savefig("python_plots/plot_pos.png")
    fig_vel.savefig("python_plots/plot_vel.png")
    fig_load.savefig("python_plots/plot_load.png")



def main():
    motor = initialise()
    motor
    
    # test_pos_vel_load(motor)

    for k in range(1000):
        test_shutdown(motor)


    shutdown(motor)

if __name__ == "__main__":
    main()
    pass