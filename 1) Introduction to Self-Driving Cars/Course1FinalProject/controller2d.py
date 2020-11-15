#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)
        self.vars.create_var('int_val', 0.0)
        self.vars.create_var('last_error', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]:
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)

                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            # Using PID controller for throttle
            throttle_output = 0
            brake_output    = 0

            kp = 2.2
            ki = 0.36
            kd = 0.2

            st = t - self.vars.t_previous

            # error term
            delta_v = v_desired - v

            # I
            integral = self.vars.int_val + delta_v * st

            # D
            derivate = (delta_v - self.vars.last_error) / st

            throttle_output = kp * delta_v + ki * integral + kd * derivate

            if throttle_output > 1:
                throttle_output = 1
                brake_output = 0
            elif throttle_output < 0:
                throttle_output = 0
                brake_output = -throttle_output
                if brake_output > 1:
                    brake_output = 1

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Change the steer output with the lateral controller.
            steer_output = 0
            # 1. calculate the heading error
            desired_yaw = np.arctan2(waypoints[-1][1] - waypoints[0][1], waypoints[-1][0] - waypoints[0][0])
            phi_error = desired_yaw - yaw
            if phi_error > np.pi:
                phi_error -= 2*np.pi
            if phi_error < -np.pi:
                phi_error += 2*np.pi
            print('[info] heading error: ', phi_error)

            # 2. calculate the cross track error
            current_xy = np.array([x, y])
            # calculate the distance between ego and each point along the waypoints
            current_and_waypoints_error = np.sqrt(np.sum((current_xy - np.array(waypoints)[:,:2]) ** 2, axis=1))

            cte_index = np.argmin(current_and_waypoints_error)
            cte = np.min(current_and_waypoints_error)

            # cross_track_deadband = 0.01
            if cte < 0.01:
                cte = 0

            # 3. Deciding sign of cross track error based on relative heading between path and crosstrack
            yaw_cross_track = np.arctan2(y - waypoints[cte_index][1], x - waypoints[cte_index][0])
            yaw_desired2ct = desired_yaw - yaw_cross_track
            if yaw_desired2ct > np.pi:
                yaw_desired2ct -= 2*np.pi
            if yaw_desired2ct < -np.pi:
                yaw_desired2ct += 2*np.pi

            if yaw_desired2ct < 0:
                cte = -cte

            print('[info] cte: ', cte)

            # 4. calculate the steering angle
            # k and ks are hyperparameters we need to tune
            k = 1.25
            ks = 1   # softening constant
            steer_output = phi_error + np.arctan(k * cte / (ks + v))

            """
            #########################################
            ######### Pure Pursuit Method ###########
            #########################################
            # 1. Set look ahead distance
            ld = 5
            # 2. search for the look ahead point on the reference path
            idx = 0
            for i in range(len(waypoints)):
            	dist = np.linalg.norm(np.array([
                        waypoints[i][0] - x,
                    	waypoints[i][1] - (y - 1)]))   # assume center of vehicle to rear axle is 1 meter
            	if dist >= ld:
            		idx = i
            		break
            # 3. calculate alpha_hat
            alpha_hat = np.arctan2(waypoints[idx][1] - (y - 1), waypoints[idx][0] - x)
            # 4. calculate alpha
            alpha = alpha_hat - yaw
            # 5.calculate steering angle
            k_dd = 1   # k_dd is the hyperparameter that needs to tune
            # 0.001 is the softening constant ensure the denominator is always > 0
            steer_output = np.arctan(2 * 2.5 * np.sin(alpha) / (k_dd * v + 0.001))
            """
            if steer_output > np.pi:
            	steer_output -= 2 * np.pi
            if steer_output < -np.pi:
            	steer_output += 2 * np.pi

            steer_output = min(1.22, steer_output)
            steer_output = max(-1.22, steer_output)
            print('[info] steering angle: ', steer_output)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
