# Copyright 2019 Wesley Liao
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import odrive
import odrive.enums


import monotonic
import threading
import time


"""
def desired_dynamics(postion: float,
             velocity: float,
             acceleration: float,
             net_force: float,
             timestep: float):
    
    return position_next, velocity_next, acceleration_next
    
def mass(mass):
    m = mass
    def decorator(dyn_fn):
    ddx2 = ddx + (f/m)
    dx2 = dx + (ddx2*dt)
    x2 = x + (dx2*dt)

    return x2, dx2, ddx2, 0.0, dt
    return decorator


def spring(spring_constant):
    k = spring_constant
    
    def decorator(dyn_fn):
    x2, dx2, ddx2, f2, dt2 = dyn_fn(x, dx, ddx, f, dt)
    return x2, dx2, ddx2, f+(x2*k), dt2
    return decorator

"""

class PIDController:
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.reset()

    def reset(self):
        self.error_sum = 0.0
        self.last_error = None

    def control(self, error, dt):
        self.error_sum += error*dt

        error_delta = 0.0
        if self.last_error is not None:
                error_delta = (error - self.last_error)/dt
        
        self.last_error = error

        return (self.kp*error) + (self.ki*self.error_sum) + (self.kd*error_delta)

class FFBSlider:

    SLIDER_STATE_ERROR = 0
    SLIDER_STATE_UNCALIBRATED = 1
    SLIDER_STATE_CALIBRATING = 2
    SLIDER_STATE_READY = 3
    SLIDER_STATE_RUNNING = 6
    SLIDER_STATE_STOPPING = 7
    SLIDER_STATE_SHUTDOWN = 8

    CALIBRATION_STEP_NOT_STARTED = 0   
    CALIBRATION_STEP_ODRIVE_ENCODER_OFFSET = 1
    CALIBRATION_STEP_FORWARD_LIMIT = 2
    CALIBRATION_STEP_REVERSE_LIMIT = 3
    CALIBRATION_STEP_CENTER_POSITION = 4
    CALIBRATION_STEP_DONE = 5

    def statestr(self):
        if self.state == self.SLIDER_STATE_ERROR:
            return "SLIDER_STATE_ERROR       "
        elif self.state == self.SLIDER_STATE_UNCALIBRATED:
            return "SLIDER_STATE_UNCALIBRATED"
        elif self.state == self.SLIDER_STATE_CALIBRATING:
            return "SLIDER_STATE_CALIBRATING "
        elif self.state == self.SLIDER_STATE_READY:
            return "SLIDER_STATE_READY       "
        elif self.state == self.SLIDER_STATE_RUNNING:
            return "SLIDER_STATE_RUNNING     "
        elif self.state == self.SLIDER_STATE_STOPPING:
            return "SLIDER_STATE_STOPPING    "
        elif self.state == self.SLIDER_STATE_SHUTDOWN:
            return "SLIDER_STATE_SHUTDOWN    "


    def calibstr(self):
        if self.calibration_step == self.CALIBRATION_STEP_NOT_STARTED:
            return "CALIBRATION_STEP_NOT_STARTED          "
        elif self.calibration_step == self.CALIBRATION_STEP_ODRIVE_ENCODER_OFFSET:
            return "CALIBRATION_STEP_ODRIVE_ENCODER_OFFSET"
        elif self.calibration_step == self.CALIBRATION_STEP_FORWARD_LIMIT:
            return "CALIBRATION_STEP_FORWARD_LIMIT        "
        elif self.calibration_step == self.CALIBRATION_STEP_REVERSE_LIMIT:
            return "CALIBRATION_STEP_REVERSE_LIMIT        "
        elif self.calibration_step == self.CALIBRATION_STEP_CENTER_POSITION:
            return "CALIBRATION_STEP_CENTER_POSITION      "
        elif self.calibration_step == self.CALIBRATION_STEP_DONE:
            return "CALIBRATION_STEP_DONE                 "


    def __init__(self,
         odrive_axis,
         slider_joint_frame,
         player1_force_topic,
         player2_force_topic, 
         update_frequency_Hz = 10,
         slider_travel_per_count_m = 0.0001):
    
        self.odaxis = odrive_axis

        self.position = None
        self.last_pos = None

        self.forward_limit = 0.0
        self.reverse_limit = 0.0

        self.player1_force = 0.0
        self.player2_force = 0.0

        rospy.init_node('ffb_slider')

        self.p1_sub = rospy.Subscriber(
            player1_force_topic,
            WrenchStamped,
            self.force_sub_cb,
            callback_args = self.player1_force)

        self.p2_sub = rospy.Subscriber(
            player2_force_topic,
            WrenchStamped,
            self.force_sub_cb,
            callback_args = self.player2_force)

        self.js_pub = rospy.Publisher('ffb_slider/joint_state', JointState, queue_size=1)
        self.slider_joint_frame = slider_joint_frame
        self.msg_id = 0 

        self.travel_per_count_m = slider_travel_per_count_m

        self.state = self.SLIDER_STATE_UNCALIBRATED
        self.calibration_step = None
        
        self.t_last = monotonic.monotonic()
        self.update_interval_s = 1.0/update_frequency_Hz
        self.update()

    @property
    def center_position(self):
        if self.forward_limit is None or self.reverse_limit is None:
            return None
        return ((self.forward_limit - self.reverse_limit)/2)+self.reverse_limit

    def next_header(self):
        self.msg_id += 1
        return Header(
            seq = self.msg_id,
            stamp = rospy.get_rostime(),
            frame_id = self.slider_joint_frame)

    def force_sub_cb(self, msg, player):
        player = msg.wrench.force.x

    def calibrate_motor(self):
        if self.state == self.SLIDER_STATE_UNCALIBRATED:
            self.state = self.SLIDER_STATE_CALIBRATING
            self.calibration_step = self.CALIBRATION_STEP_NOT_STARTED

    def start(self):
        if self.state == self.SLIDER_STATE_READY:
            self.state =  self.SLIDER_STATE_RUNNING
        else:
            print("ERROR system not ready")

    def stop(self):
        if self.state == self.SLIDER_STATE_RUNNING:
            self.state = self.SLIDER_STATE_STOPPING
        else:
            print("ERROR system not running")
    
    def shutdown(self):
        self.state = self.SLIDER_STATE_SHUTDOWN

    def update(self):
    
        # Scheedule next update before anything else so execution time
        # does not slow update rate.
        if self.state != self.SLIDER_STATE_SHUTDOWN:
            looptimer = threading.Timer(self.update_interval_s, self.update)
            looptimer.start()
        else:
            self.odaxis.requested_state = odrive.enums.AXIS_STATE_IDLE
        
        t_now = monotonic.monotonic()
        dt_s = t_now-self.t_last
        self.t_last = t_now

        print('{:06.5f}'.format(dt_s), self.statestr(), self.calibstr(), self.odaxis.current_state,
        '{:06.2f}'.format(self.forward_limit), 
        '{:06.2f}'.format(self.reverse_limit),
        '{:06.2f}'.format(self.center_position), 
        '{:06.2f}'.format(self.odaxis.encoder.pos_estimate),
        '{:06.2f}'.format(self.odaxis.encoder.vel_estimate))

        self.js_pub.publish(JointState(
            header = self.next_header(),
            name = ("slider",),
            position = ((self.odaxis.encoder.pos_estimate-self.center_position)*self.travel_per_count_m,),
            velocity = (self.odaxis.encoder.vel_estimate*self.travel_per_count_m,),
            effort = (0.0,)))

        if self.state  == self.SLIDER_STATE_ERROR:
            print("ERROR")

        elif self.state == self.SLIDER_STATE_UNCALIBRATED:
            pass
        elif self.state == self.SLIDER_STATE_CALIBRATING:
        
            if self.calibration_step == self.CALIBRATION_STEP_NOT_STARTED:
                self.odaxis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                self.calibration_step = self.CALIBRATION_STEP_ODRIVE_ENCODER_OFFSET

            elif self.calibration_step == self.CALIBRATION_STEP_ODRIVE_ENCODER_OFFSET:
                if self.odaxis.current_state == odrive.enums.AXIS_STATE_IDLE: #wait for odrive to finish
            
                    if not self.odaxis.error:
                        self.odaxis.controller.config.control_mode = odrive.enums.CTRL_MODE_VELOCITY_CONTROL
                        self.odaxis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
                        self.odaxis.controller.vel_setpoint = 2400 #counts/s
                        fwtimeout = threading.Timer(5.0, self.motor_limit_timeout)
                        fwtimeout.start()
                        self.calibration_step = self.CALIBRATION_STEP_FORWARD_LIMIT
                    else:
                        self.state = self.SLIDER_STATE_ERROR
            
            elif self.calibration_step == self.CALIBRATION_STEP_CENTER_POSITION:
                if abs(self.odaxis.encoder.vel_estimate) < 50 and abs(self.odaxis.encoder.shadow_count - self.center_position) < 120:
                    self.odaxis.requested_state = odrive.enums.AXIS_STATE_IDLE

                    self.calibration_step = self.CALIBRATION_STEP_DONE
                    self.state = self.SLIDER_STATE_READY

        elif self.state == self.SLIDER_STATE_READY:
            pass
        elif self.state == self.SLIDER_STATE_RUNNING:
          pass 

        elif self.state == self.SLIDER_STATE_STOPPING:
            pass

    def motor_limit_timeout(self):

        if self.calibration_step == self.CALIBRATION_STEP_FORWARD_LIMIT:
            self.forward_limit = self.odaxis.encoder.pos_estimate

            self.odaxis.controller.vel_setpoint = -2400 #counts/s

            rvtimeout = threading.Timer(5.0, self.motor_limit_timeout)
            rvtimeout.start()

            self.calibration_step = self.CALIBRATION_STEP_REVERSE_LIMIT

        elif self.calibration_step == self.CALIBRATION_STEP_REVERSE_LIMIT:
            self.reverse_limit = self.odaxis.encoder.pos_estimate
            
            self.odaxis.controller.config.control_mode = odrive.enums.CTRL_MODE_POSITION_CONTROL
            self.odaxis.controller.pos_setpoint = self.center_position

            self.odaxis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

            self.calibration_step = self.CALIBRATION_STEP_CENTER_POSITION



if __name__ == "__main__":
    print("finding an odrive...")
    odrv0 = odrive.find_any()

    belt_pitch_m = 0.005
    pulley_teeth = 20
    encoder_cpr = 2400

    travel_per_count_m = (belt_pitch_m*pulley_teeth)/encoder_cpr
    
    ffbslider = FFBSlider(odrv0.axis1,
            "slider",
            "load_cell_1",
            "load_cell_2",
            update_frequency_Hz = 10,
        slider_travel_per_count_m = travel_per_count_m)

    time.sleep(2)
    ffbslider.calibrate_motor()


    while(not rospy.is_shutdown()):
        time.sleep(.1)
    ffbslider.shutdown()
    
