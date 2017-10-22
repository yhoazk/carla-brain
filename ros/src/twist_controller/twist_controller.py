import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
from math import fabs
from twiddle import PIDWithTwiddle

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

PRED_STEERING_FACTOR = 0.2
CORR_STEERING_FACTOR = 0.3


class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):

        self.brake_deadband = brake_deadband
        total_car_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_car_factor = total_car_mass * wheel_radius

        self.prev_time = rospy.get_time()

        # twiddle algorithm is disabled so iterations and tolerance are here to show
        # what values to use when you want to activate twiddle.
        # kp, ki and kd are values found from previous twiddle runs.
        self.steer_pid = PIDWithTwiddle(kp=0.607900, ki=0.000172, kd=1.640951,
                             mn=-max_steer_angle, mx=max_steer_angle,
                             optimize_params=False, iterations=10, tolerance=0.05)

        self.max_steer_angle = max_steer_angle
        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            min_speed=2.0,
                                            max_lat_accel=max_lat_accel,
                                            max_steer_angle=max_steer_angle)

        self.accel_pid = PIDWithTwiddle(kp=1.806471, ki=0.000635, kd=0.715603,
                                        mn=decel_limit, mx=accel_limit,
                                        optimize_params=False, iterations=10, tolerance=0.05)

    def control(self,
                dbw_enabled,
                cte,
                linear_velocity,
                angular_velocity,
                current_velocity):

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        if dbw_enabled:
            current_time = rospy.get_time()
            sample_time = current_time - self.prev_time
            self.prev_time = current_time

            predictive_steer = self.yaw_controller.get_steering(linear_velocity=linear_velocity,
                                                                angular_velocity=angular_velocity,
                                                                current_velocity=current_velocity)

            corrective_steer = self.steer_pid.step(error=cte, sample_time=sample_time)

            steer = CORR_STEERING_FACTOR * corrective_steer + PRED_STEERING_FACTOR * predictive_steer

            rospy.logdebug('steer = %f, cte = %f, corrective_steer = %f, predictive_steer = %f',
                          steer, cte, corrective_steer, predictive_steer)

            vel_delta = linear_velocity - current_velocity
            accel = self.accel_pid.step(error=vel_delta, sample_time=sample_time)

            rospy.logdebug('desired vel = %f, current vel = %f, accel = %f',
                          linear_velocity, current_velocity, accel)

            if accel < 0.0:
                if -accel < self.brake_deadband:
                    accel = 0.0
                
                throttle, brake = 0, -accel * self.brake_car_factor
            else:
                throttle, brake = accel, 0

        else:
            self.steer_pid.reset()
            self.accel_pid.reset()
            self.prev_time = rospy.get_time()

        return throttle, brake, steer
