#!/usr/bin/env python

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

from slider import InteractionSlider

import odrive
import odrive.enums

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

class FFBSlider(InteractionSlider):

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
            slider_joint_name: str,
            slider_joint_frame_id: str,
            player1_force_topic: str,
            player2_force_topic: str,
            odrive_axis,
            encoder_cpr = 2400,
            belt_pitch_m = 0.005,
            pulley_teeth = 20,
            node_name = "ODriveBeltSlider",
            update_frequency_Hz = 10):

         super(FFBSlider, self).__init__(
             slider_joint_name,
             slider_joint_frame_id,
             player1_force_topic,
             player2_force_topic,
             node_name = node_name,
             update_frequency_Hz = update_frequency)
    
        self.odaxis = odrive_axis
        self.encoder_cpr = encoder_cpr
        self.belt_pitch_m = belt_pitch_m
        self.pulley_teeth = pulley_teeth

        self.forward_limit_counts = 0.0
        self.reverse_limit_counts = 0.0

        self.state = self.SLIDER_STATE_UNCALIBRATED
        self.calibration_step = None

    @property
    def position_m(self):
        return self.position_counts*self.travel_per_count_m

    @property
    def velocity_mps(self):
        return self.odaxis.encoder.vel_estimate

    @property
    def travel_per_count_m(self):
        return (self.belt_pitch_m*self.pulley_teeth)/self.encoder_cpr

    @property
    def position_counts(self):
        return self.odaxis.encoder.pos_estimate-self.center_position_counts

    @property
    def center_position_counts(self):
        return ((self.forward_limit - self.reverse_limit)/2)+self.reverse_limit


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

    def update(self, dt_s):
    
        print('{:06.5f}'.format(dt_s), self.statestr(), self.calibstr(), self.odaxis.current_state,
            '{:06.2f}'.format(self.forward_limit), 
            '{:06.2f}'.format(self.reverse_limit),
            '{:06.2f}'.format(self.center_position), 
            '{:06.2f}'.format(self.odaxis.encoder.pos_estimate),
            '{:06.2f}'.format(self.odaxis.encoder.vel_estimate))

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
    
    ffbslider = FFBSlider(    time.sleep(2)
    ffbslider.calibrate_motor()


    while(not rospy.is_shutdown()):
        time.sleep(.1)
    ffbslider.shutdown()
    
