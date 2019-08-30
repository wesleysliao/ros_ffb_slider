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

import time
import rospy


class SpringSimSlider(InteractionSlider):
    
    def __init__(self,
        slider_joint_name,
        slider_joint_frame_id,
        player1_force_topic,
        player2_force_topic,
        spring_constant_Npm = 1,
        node_name = "SpringSimSlider",
        update_frequency_Hz = 10):
        
        self.k_Npm = spring_constant_Npm

        super(SpringSimSlider, self).__init__(
            slider_joint_name,
            slider_joint_frame_id,
            player1_force_topic,
            player2_force_topic,
            node_name = node_name,
            update_frequency_Hz = update_frequency_Hz)
        
    @property
    def effort_N(self):
        return -(-self.player2_force_N)
        #return -(self.player1_force_N - self.player2_force_N)

    @InteractionSlider.position_m.setter
    def position_m(self, pos_value):
        self._position_m = pos_value

    @InteractionSlider.velocity_mps.setter
    def velocity_mps(self, vel_value):
        self._velocity_mps = vel_value

    def update(self, dt_s):
        pos = -self.effort_N/self.k_Npm
        self.velocity_mps = (pos - self.position_m)/dt_s
        self.position_m = pos

if __name__ == "__main__":
    springsimslider = SpringSimSlider(
        "baseboard_to_simslider",
        "simslider",
        "load_cell_1",
        "load_cell_2",
        spring_constant_Npm = 1000)

    rospy.on_shutdown(springsimslider.stop)

