"""driver controller."""

from controller import Robot, Motor, Keyboard, PositionSensor, Supervisor, Gyro, GPS
import time
import numpy as np
import cv2 as cv
from datetime import datetime
import sys

# Implementation of Gyrodometry in gyrodometry.py
from gyrodometry import Gyrodometry

np.random.seed(42)

# Speed of the robot
SPEED = 6

# Functions to move the robot
def move_forward(SPEED):
    """
    Moves forward
    """
    
    global left_speed, right_speed

    left_speed = SPEED
    right_speed = SPEED
    

def move_backward(SPEED):
    """
    Moves backward
    """
    
    global left_speed, right_speed

    left_speed = -SPEED
    right_speed = -SPEED
    

def turn_right(SPEED):
    """
    Moves right
    """
    
    global left_speed, right_speed

    left_speed = SPEED * 0.175
    right_speed = -SPEED * 0.175
    

def turn_left(SPEED):
    """
    Moves left
    """
    
    global left_speed, right_speed

    left_speed = -SPEED * 0.175
    right_speed = SPEED * 0.175
    

def stop_robot():
    """
    Stops the robot
    """
    
    global left_speed, right_speed

    left_speed = 0.
    right_speed = 0.

    
def free_movement():   

    """
    Function that moves the robot using arrow keys
    """
    global SPEED


    # Instantiates the robot object and the timestep value
    robot = Robot()
    timestep = 32
             
    # Instantiates the keyboard object
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Setup of the motors so that they can be controlled by the keyboard
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Enables the encoders
    left_position_sensor = robot.getDevice('left wheel sensor')
    left_position_sensor.enable(timestep)
    right_position_sensor = robot.getDevice('right wheel sensor')
    right_position_sensor.enable(timestep)

    # Enables the gyro
    gyro = robot.getDevice('gyro')
    gyro.enable(timestep)

    # Enables the gps
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    # Creates a position tracker based on gyrodometry
    position_tracker = Gyrodometry(2.5, 0.5, 0., gyro, only_odometry=False)

    # Current speed of each wheel (it starts stopped)
    global left_speed, right_speed
    left_speed = 0
    right_speed = 0

    # Enables the tof sensor that is used to generate the map
    distance_sensor = robot.getDevice('tof')
    distance_sensor.enable(timestep)

    # Cycle counter
    cycle_count = 0

    # Creates a csv file with the log of the execution
    with open("data.csv", 'w') as file:
        file.write("x_odo,z_odo,theta_odo,x_gyro,z_gyro,theta_gyro,x_gps,y_gps,z_gps,odo_dist,gyro_dist,delta_theta_odo,delta_theta_gyro" + '\n')

    # Loop of the driver
    while robot.step(timestep) != -1:

        cycle_count += 1

        # Gets the a key of the kayboard
        key = keyboard.getKey()

        # It moves the robot according to the kayboard input
        if (key == Keyboard.UP):
            move_forward(SPEED)
        elif (key == Keyboard.DOWN):
            move_backward(SPEED)
        elif (key == Keyboard.RIGHT):
            turn_right(SPEED)
        elif (key == Keyboard.LEFT):
            turn_left(SPEED)
        else:
            stop_robot()
            
        # Updated velocities    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        # Gets odometry data from the encoders
        left_encoder_count = left_position_sensor.getValue()
        right_encoder_count = right_position_sensor.getValue()

        # We start in the cycle 50 so that the system is stabilized
        if cycle_count > 50:

            # Updates position using gyrodometry
            position_tracker.update_position(right_encoder_count, left_encoder_count)

            # Gets absolute (GPS) coordinates, gyrodometry coordinates and odometry coordinates
            gps_values = gps.getValues()
            gyrodometry_values = position_tracker.getGyrodometryValues()
            odometry_values = position_tracker.getOdometryValues()

            # Computes the error between gps value and both odometry and gyrodometry
            odo_dist = np.sqrt((gps_values[0] - odometry_values[0])**2 + (gps_values[2] - odometry_values[1])**2)
            gyro_dist = np.sqrt((gps_values[0] - gyrodometry_values[0])**2 + (gps_values[2] - gyrodometry_values[1])**2)

            # Gets both theta increments
            delta_theta_odo = position_tracker.delta_theta_odo
            delta_theta_gyro = position_tracker.delta_theta_gyro

            # Updates the log
            with open("data.csv", 'a') as file:
                file.write(f"{odometry_values[0]},{odometry_values[1]},{odometry_values[2]}," +
                        f"{gyrodometry_values[0]},{gyrodometry_values[1]},{gyrodometry_values[2]}," +
                        f"{gps_values[0]},{gps_values[1]},{gps_values[2]},{odo_dist},{gyro_dist}," + 
                        f"{delta_theta_odo},{delta_theta_gyro}" + '\n')

        

if __name__ == '__main__':
    free_movement()
