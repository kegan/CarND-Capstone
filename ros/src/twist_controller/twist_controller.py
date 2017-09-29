import numpy as np
import rospy
import tf
import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    """ Twist controller to predict throttle, brake, and steer."""
    def __init__(self, *args, **kwargs):
        self.decel_limit = kwargs.get('decel_limit')
        self.accel_limit = kwargs.get('accel_limit')
        self.max_steer_angle = kwargs.get('max_steer_angle')

        # Vehicle mass, fuel mass and wheel radius are required to calculate braking torque
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.wheel_radius = kwargs.get('wheel_radius')
        self.fuel_mass = kwargs.get('fuel_capacity') * GAS_DENSITY # Assuming a full tank of gas

        # Controllers
        self.yaw_controller = YawController(
            wheel_base=kwargs.get('wheel_base'),
            steer_ratio=kwargs.get('steer_ratio'),
            min_speed=kwargs.get('min_speed'),
            max_lat_accel=kwargs.get('max_lat_accel'),
            max_steer_angle=kwargs.get('max_steer_angle')
        )
        self.throttle_filter = LowPassFilter(0.96, 1.0)
        self.brake_filter = LowPassFilter(0.96, 1.0)
        self.steer_filter = LowPassFilter(0.96, 1.0)

        self.acceleration = 0
        self.timestamp = None

    def control(self, *args, **kwargs):
        # For steering, we need the vehicle velocity (angular and linear)
        # For velocity, we need the current velocity and target velocity to calculate the error
        current_velocity = kwargs.get('current_velocity')
        linear_velocity = kwargs.get('linear_velocity')
        angular_velocity = kwargs.get('angular_velocity')
        target_velocity = kwargs.get('target_velocity')

        throttle, brake, steer = 0., 0., 0.

        # If this is the first time control() is called
        if self.timestamp is None:
            self.timestamp = rospy.get_time()
            return throttle, brake, steer

        current_timestamp = rospy.get_time()
        delta_time = current_timestamp - self.timestamp
        timestamp = current_timestamp

        # Velocity is in meters per second
        current_velocity = current_velocity
        velocity_error = target_velocity - current_velocity


        # Make sure we have a valid delta_time. We don't want to divide by zero.
        # Since we're publishing at 50Hz, the expected delta_time should be around 0.02
        if delta_time > 0.01:

            if velocity_error > 10:
                if self.acceleration < 0: self.acceleration = 0
                self.acceleration = 1.
            elif velocity_error > 2.5:
                self.acceleration += 0.224
                self.acceleration = min(self.acceleration, self.accel_limit)
            elif velocity_error >= 0:
                self.acceleration = 0.
            elif velocity_error > -10:
                self.acceleration -= 0.224
                self.acceleration = max(self.acceleration,self.decel_limit)
            else:
                if self.acceleration > 0: self.acceleration = 0
                self.acceleration = -1

            acceleration = self.acceleration

            # Throttle when acceleration is positive
            # Brake when acceleration is negative
            if acceleration >= 0:
                throttle = acceleration
                brake = 0.
            else:
                # Brake calculations (Brake value is in Torque (N * m))
                # (i.e. how much torque would we need to reduce our acceleration by x?)

                # Braking Torque = Force * Distance from point of rotation
                # * Distance from point of rotation = Wheel radius
                # * Force = Mass * target deceleration
                # * Mass = Vehicle mass + Fuel mass (assuming full tank)
                deceleration = abs(acceleration)
                brake = (self.vehicle_mass + self.fuel_mass) * deceleration * self.wheel_radius

                throttle = 0.

        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Apply low pass filters to the throttle and brake values to eliminate jitter
        throttle = min(max(self.throttle_filter.filt(throttle), 0), self.accel_limit)

        # Only apply smoothing if we are actually braking        
        if brake != 0.0:
            brake = self.brake_filter.filt(brake)
        else:
            self.brake_filter.reset()

        steer = self.steer_filter.filt(steer)

        # rospy.logout('Throttle=%f, Brake=%f, Steer=%f, Acceleration=%f', throttle, brake, steer, self.acceleration)

        return throttle, brake, steer