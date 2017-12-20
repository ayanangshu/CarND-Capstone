import rospy
from pid import PID
from yaw_controller import YawController
<<<<<<< HEAD
from lowpass import LowPassFilter
=======
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DT = 1./50.


class Controller(object):
<<<<<<< HEAD
    def __init__(self, steer_ratio, decel_limit, accel_limit, max_steer_angle, wheel_base, max_let_accel,
                 max_throttle_percent, max_braking_percent):
=======
    def __init__(self, steer_ratio, decel_limit, accel_limit, max_steer_angle, wheel_base, max_let_accel):
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22

        self.steer_ratio = steer_ratio
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.max_steer_angle = max_steer_angle
        self.max_let_accel = max_let_accel
        self.wheel_base = wheel_base
<<<<<<< HEAD
        self.max_braking_percent = max_braking_percent

        # We use proportional factors between Carla and the simulator.
        # The simulator only was used for calibration. Max values for Carla were extracted through
        # "dbw_test.py" and provided rosbag
        calib_throttle = max_throttle_percent / 0.4
        calib_brake = max_braking_percent / 100
        self.throttle_pid = PID(0.1 * calib_throttle, 0.001 * calib_throttle, 0, 0, max_throttle_percent)
        self.brake_pid = PID(60. * calib_brake, 1. * calib_brake, 0, 0, max_braking_percent)

        tau = 0.1
        self.throttle_filter = LowPassFilter(tau, DT)
        self.brake_filter = LowPassFilter(tau, DT)
        self.steer_filter = LowPassFilter(tau, DT)
=======

        self.throttle_pid = PID(.15, .00, 0, self.decel_limit, self.accel_limit)
        self.brake_pid = PID(15, 1, 0, 0, 100)
        self.steer_pid = PID(2.0, 0.0005, 2.0, -self.max_steer_angle, self.max_steer_angle)
        self.last_velocity_error = 0
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22
        self.last_time = 0
        self.DT = DT
        self.brakeLatch = False

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio,
<<<<<<< HEAD
                                            0.5, self.max_let_accel, self.max_steer_angle)
=======
                                            0, self.max_let_accel, self.max_steer_angle)
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22

    def control(self, target_linear_velocity, target_angular_velocity,
                current_linear_velocity, dbw_status, log_handle):
        '''Defines target throttle, brake and steering values'''

<<<<<<< HEAD
        # Update DT
        new_time = rospy.get_rostime()
        if self.last_time:  # The first time, we are not able to calculate DT
            self.DT = (new_time - self.last_time).to_sec()
        self.last_time = new_time
            
        if dbw_status:            
            velocity_error = target_linear_velocity - current_linear_velocity

            # if we're going too slow, release the brakes (if they are applied)
            # This essentially adds hysterisis: the brakes are only enabled if our velocity error is negative, and only
            # released if the velocity error is positive 2 or greater.
            if velocity_error > 1:
                self.brakeLatch = False
            if not self.brakeLatch:
                throttle = self.throttle_pid.step(velocity_error, self.DT)
                brake = 0
                self.brake_pid.reset()
                self.brake_filter.reset()
                # if we go too fast and we cannot decrease throttle, we need to start braking
                if (velocity_error < 0 and throttle is 0) or (velocity_error < -1):
                    self.brakeLatch = True
=======
        if dbw_status:
            # Update DT
            new_time = rospy.get_rostime()
            if self.last_time:  # The first time, we are not able to calculate DT
                self.DT = (new_time - self.last_time).to_sec()
            self.last_time = new_time

            velocity_error = target_linear_velocity - current_linear_velocity
            pid_throttle, feedforward_throttle, decel_target = 0, 0, 0

            # implement longitudinal controller

            # if we're going too slow, release the brakes (if they are applied)
            #T his essentially adds hysterisis: the brakes are only enabled if our velocity error is negative, and only
            # released if the velocity error is positive 2 or greater.
            if velocity_error > 2:
                self.brakeLatch = False
            if self.brakeLatch is False:
                pid_throttle = self.throttle_pid.step(velocity_error, self.DT, log_handle)
                feedforward_throttle = target_linear_velocity*.01888 #based on desired speed, predict how much throttle we need at steady state
                throttle = pid_throttle + feedforward_throttle
                accel_limit = 1 #mps2
                maxThrottle = 0.1962*accel_limit+0.083 # max throttle allowed for a given acceleration limit
                throttle = max(0,min(throttle, maxThrottle))  # saturate throttle if it exceeds acceleration limits
                brake = 0
                if current_linear_velocity >= .1 and velocity_error < 0 and throttle is 0:
                    self.brakeLatch = True
            # we need to brake when throttle PID is saturated at 0 and velocity error is negative. In other words, when
            # we can't slow down any more (throttle is already zero), and we want to slow down more (speed error is
            # negative), then we need to use the brakes
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22
            else:
                # We are currently braking
                throttle = 0
<<<<<<< HEAD
                self.throttle_pid.reset()
                self.throttle_filter.reset()
                brake = self.brake_pid.step(-velocity_error, self.DT)
            # If we're about to come to a stop, clamp the brake command to some value to hold the vehicle in place
            if current_linear_velocity < .1 and target_linear_velocity == 0:
                throttle = 0
                brake = self.max_braking_percent
                
            # implement steering controller
            steer_target = self.yaw_controller.get_steering(target_linear_velocity,
                                                     target_angular_velocity,
                                                     current_linear_velocity)

            # filter commands
            throttle = self.throttle_filter.filt(throttle)
            brake = self.brake_filter.filt(brake)
            steering = self.steer_filter.filt(steer_target)
=======
                brake = self.brake_pid.step(-velocity_error, self.DT, log_handle)
            # If we're about to come to a stop, clamp the brake command to some value to hold the vehicle in place
            if current_linear_velocity < .1 and target_linear_velocity == 0:
                throttle = 0
                brake = 25

            # implement steering controller
            steer_error = self.yaw_controller.get_steering(target_linear_velocity,
                                                     target_angular_velocity,
                                                     current_linear_velocity)

            steering = self.steer_pid.step(steer_error, self.DT, log_handle)
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22


            self.last_velocity_error = velocity_error
            #args = velocity_error
            self.log_data(log_handle, pid_throttle, feedforward_throttle, velocity_error, self.DT, decel_target, int(self.brakeLatch))
        else:
            self.brakeLatch = False
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.throttle_filter.reset()
            self.brake_filter.reset()
            self.steer_filter.reset()
            throttle, brake, steering = 0, 0, 0
            pid_throttle, velocity_error = 0, 0

        # Log data
        throttle_P, throttle_I, throttle_D = self.throttle_pid.get_PID()
        brake_P, brake_I, brake_D = self.brake_pid.get_PID()
        steer_P, steer_I, steer_D = 0, 0, 0
        self.log_data(log_handle, throttle_P, throttle_I, throttle_D, brake_P, brake_I, brake_D,
                          velocity_error, self.DT, int(self.brakeLatch))

        return throttle, brake, steering
<<<<<<< HEAD
=======

    def is_change_acc(self, velocity_error):

        is_switch_brake = (self.last_velocity_error >= 0) and (velocity_error < 0)
        is_switch_acc = (self.last_velocity_error <= 0) and (velocity_error > 0)

        return is_switch_brake or is_switch_acc
>>>>>>> bad2e61290983b1b85958c96b83e35c80c0c8b22
    
    def log_data(self, log_handle, *args):
        log_handle.write(','.join(str(arg) for arg in args) + ',')
