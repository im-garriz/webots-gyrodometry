import numpy as np
import time


# Class that computes the odometry of the robot
class Odometry:

    # Constructor of the class
    def __init__(self, x_init, z_init, theta_init):

        # Radius of the wheels
        self._WHEEL_RADIUS = 0.0205
        
        # Encoder increments per tour -> webots gives the position in radians, not encoder pulses,
        # so a whole tour is 2pi radians
        self._ENCODER_INCREMENTS_PER_TOUR = 2.0 * np.pi
        
        # Wheelbase: distance between wheels
        self._WHEELBASE = 0.05825
       
        # Starting position of the robot
        self._x = x_init
        self._z = z_init
        self._theta = theta_init

        # Variables where previous value of both encoders is stored
        self._old_right_count = 0
        self._old_left_count = 0
        
        # Encoder unit: conversion ratio between encoder position and linear distance
        self._encoder_unit = (2 * np.pi * self._WHEEL_RADIUS) / self._ENCODER_INCREMENTS_PER_TOUR

        self.delta_theta = 0

    # Method that updates the odometry position of the robot
    def update_odometry(self, encoder_right, encoder_left):

        # Increment on each encoder from the previous step
        delta_right = encoder_right - self._old_right_count
        delta_left = encoder_left - self._old_left_count

        # Stores the current values of both encoder for the next update
        self._old_right_count = encoder_right
        self._old_left_count = encoder_left

        # Estimation of the distance traveled by the wheels
        right_dist = delta_right * self._encoder_unit
        left_dist = delta_left * self._encoder_unit

        # The center distance is the mean between the distance of both wheels
        self.center_dist = (right_dist + left_dist) / 2
        
        # Variation of the angle from the previous update
        self.delta_theta_odo = (left_dist - right_dist) / self._WHEELBASE

    def update_robot_pos(self, delta_theta):

        # Updates the current position
        self._x = self._x + self.center_dist * np.cos(self._theta + delta_theta/2)
        self._z = self._z + self.center_dist * np.sin(self._theta + delta_theta/2)
        self._theta = self._theta + delta_theta
        
        # Handles the value of theta so that it is between 0 and 2pi
        if np.abs(self._theta) > 2 * np.pi:
            if self._theta > 0:
                self._theta = self._theta - 2 * np.pi
            else:
                self._theta = self._theta + 2 * np.pi


# Class that computes gyrodometry. As gyrometry is based on odometry, it inherits from it
class Gyrodometry(Odometry):

    def __init__(self, x_init, z_init, theta_init, gyro, only_odometry=False):

        super(Gyrodometry, self).__init__(x_init, z_init, theta_init)

        # Delta theta threshold: value that indicates whether to use
        # odomtetry or gyrodometry delta theta
        self.delta_theta_thresh = 0.015

        # Gyroscope of the robot
        self.gyro = gyro

        # Time increments for compute delta theta gyro
        self.t_old = 0
        self.t_current = 0

        # Numer of times the gyro is measured for one value (averaged)
        self.N_MEASURES = 20

        # Whether to do only odometry or gyrodometry
        self.only_odometry = only_odometry

        # Variables that store odometry positions (to keep track of both
        # odometry and gyrodometry)
        self.odo_x = x_init
        self.odo_z = z_init
        self.odo_theta = theta_init

    # Updates the position of the robot given its encoder values
    def update_position(self, encoder_right, encoder_left):
        
        # Computes odometry
        self.update_odometry(encoder_right, encoder_left)
        self.update_robot_pos_odometry()

        # Reads the gyro
        self.read_gyroscope_data()

        # Computes delta theta gyro-odo
        delta_gyro_odo = np.abs(self.delta_theta_odo - self.delta_theta_gyro)

        # If its higher than the threhold, uses delta_theta_gyro
        if delta_gyro_odo > self.delta_theta_thresh:    
            self.update_robot_pos(self.delta_theta_gyro)
        # It uses delta_theta_odo otherwise
        else:
            self.update_robot_pos(self.delta_theta_odo)

    # Read the gyro and computes delta theta gyro
    def read_gyroscope_data(self):

        # Computes delta t
        self.t_old = self.t_current
        self.t_current = time.time()
        delta_t = self.t_current - self.t_old

        if delta_t < 1000 and not self.only_odometry:

            measures = []

            # Makes N_MEASURES measures
            for _ in range(self.N_MEASURES):
                gyro_data = self.gyro.getValues()
                measures.append(gyro_data[-1])

            # computes its mean
            gyro_mesure = -np.mean(measures)

            # Computes angular vel
            omega = gyro_mesure * 0.00013249 + 0.0073192
            # And uses it to compute delta theta gyro
            self.delta_theta_gyro = omega * delta_t

            # Prints the difference between both delta thetas
            dif = np.abs(self.delta_theta_odo - self.delta_theta_gyro)
            print(f"Delta theta odo: {self.delta_theta_odo:.3f}, delta theta gyro: {self.delta_theta_gyro:.3f}, dif = {dif:.4f}")

        else:
            self.delta_theta_gyro = self.delta_theta_odo

        
    def getGyrodometryValues(self):

        return [self._x, self._z, self._theta]

    def getOdometryValues(self):

        return [self.odo_x, self.odo_z, self.odo_theta]


    def update_robot_pos_odometry(self):

        # Updates the current position
        self.odo_x = self.odo_x + self.center_dist * np.cos(self.odo_theta + self.delta_theta_odo/2)
        self.odo_z = self.odo_z + self.center_dist * np.sin(self.odo_theta + self.delta_theta_odo/2)
        self.odo_theta = self.odo_theta + self.delta_theta_odo
        
        # Handles the value of theta so that it is between 0 and 2pi
        if np.abs(self._theta) > 2 * np.pi:
            if self._theta > 0:
                self._theta = self._theta - 2 * np.pi
            else:
                self._theta = self._theta + 2 * np.pi
