import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


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

        #Yaw Controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        #PID
        kp = 2
        ki = 0.0004
        kd = 0.1
        mn = 0.  # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, max)

        #LowPassFilter because velocity coming on messages are noise, so, we need to filter high frequency noise
        tau = 0.5
        ts = 0.02 #sample time
        self.vel_lpf = LowPassFilter(tau,ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.last_vel = 0
        self.brake = 0

        self.last_time = rospy.get_time()


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):

        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel , current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            self.brake = 700  # 700 Nm to hold the car in place if we are stopped at traffic light.
            print("Condition1", current_vel, linear_vel)
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decelerator = max(vel_error,self.decel_limit)
            self.brake = abs(decelerator) * (self.vehicle_mass + (self.fuel_capacity * GAS_DENSITY))* self.wheel_radius  # Torque is in Nm units
            print("Condition2", current_vel, linear_vel)
        else:
            self.brake = 0
            print("Condition3", current_vel, linear_vel)


        return throttle, self.brake, steering
