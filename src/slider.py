
import monotonic
import threading
import time

import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class InteractionSlider:

    def __init__(self,
        slider_joint_name: str,
        slider_joint_frame_id: str,
        player1_force_topic: str,
        player2_force_topic: str,
        node_name = "InteractionSlider",
        update_frequency_Hz = 10):

        self._position_m = None
        self._velocity_mps = None
        self._effort_N = None
        
        self.player1_force_N = None
        self.player2_force_N = None

        self.joint_name = slider_joint_name
        self.joint_frame_id = slider_joint_frame_id 

        rospy.init_node(node_name, anonymous=True)

        self.p1_sub = rospy.
         self.p1_sub = rospy.Subscriber(
            player1_force_topic,
            WrenchStamped,
            self.force_sub_cb,
            callback_args = self.player1_force_N)

        self.p2_sub = rospy.Subscriber(
            player2_force_topic,
            WrenchStamped,
            self.force_sub_cb,
            callback_args = self.player2_force_N)

        self.js_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.msg_id = 0

        self.t_last = monotonic.monotonic()
        self.update_interval_s = 1.0/update_frequency_Hz
        self.publish_loop()

    @property
    def velocity_mps(self):
        return self._velocity_mps

    @property
    def position_m(self):
        return self._position_m

    @property
    def effort_N(self):
        return self._effort_N

    def next_header(self):
        self.msg_id += 1
        return Header(
            seq = self.msg_id,
            stamp = rospy.get_rostime(),
            frame_id = self.slider_joint_name)

    def force_sub_cb(self, msg, player):
        player = msg.wrench.force.x

    def publish_loop(self):
        looptimer = threading.Timer(self.update_interval_s, self.publish_loop)
        looptimer.start()
        
        t_now = monotonic.monotonic()
        dt_s = t_now-self.t_last
        self.t_last = t_now

        self.update(dt_s)

        self.js_pub.publish(JointState(
            header = self.next_header(),
            name = (self.joint_name,),
            position = self.position_m,
            velocity = self.velocity_mps, 
            effort = self.effort_N))

    def update(self, dt_s):
        self.position_m = 0.0
        self.velocity_mps = 0.0
        self.effort_N = 0.0


class SpringSimSlider(InteractionSlider):
    
    def __init__(self,
        slider_joint_name: str,
        slider_joint_frame_id: str,
        player1_force_topic: str,
        player2_force_topic: str,
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
            update_frequency_Hz = update_frequency)

    def update(self, dt_s):
        
        applied_force_N = self.player1_force_N + self.player2_force_N
        
        pos = applied_force_N/self.k_Npm
        
        self.effort_N = -applied_force_N
        self.velocity_mps = (pos - self.position_m)/dt_s
        self.position_m = pos
