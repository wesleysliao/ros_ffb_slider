
import monotonic
import threading
import time

import rospy
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class InteractionSlider(object):

    def __init__(self,
        slider_joint_name,
        slider_joint_frame_id,
        player1_force_topic,
        player2_force_topic,
        node_name = "InteractionSlider",
        update_frequency_Hz = 10):

        self._position_m = 0.0 
        self._velocity_mps = 0.0
        self._effort_N = 0.0
        
        self.player1_force_N = 0.0
        self.player2_force_N = 0.0

        self.slider_joint_name = slider_joint_name
        self.slider_joint_frame_id = slider_joint_frame_id 

        rospy.init_node(node_name, anonymous=True)

        self.p1_sub = rospy.Subscriber(
            player1_force_topic,
            WrenchStamped,
            self.p1_force_sub_cb)

        self.p2_sub = rospy.Subscriber(
            player2_force_topic,
            WrenchStamped,
            self.p2_force_sub_cb)

        self.js_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.msg_id = 0

        self.t_last = monotonic.monotonic()
        self._stop_flag = False
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
            frame_id = self.slider_joint_frame_id)

    def p1_force_sub_cb(self, msg):
        self.player1_force_N = msg.wrench.force.x

    def p2_force_sub_cb(self, msg):
        self.player2_force_N = msg.wrench.force.x

    def stop(self):
        self._stop_flag = True

    def publish_loop(self):
        if not self._stop_flag:
            looptimer = threading.Timer(self.update_interval_s, self.publish_loop)
            looptimer.start()
        else:
            return
        
        t_now = monotonic.monotonic()
        dt_s = t_now-self.t_last
        self.t_last = t_now

        self.update(dt_s)

        self.js_pub.publish(JointState(
            header = self.next_header(),
            name = (self.slider_joint_name,),
            position = (self.position_m,),
            velocity = (self.velocity_mps,), 
            effort = (self.effort_N,)))

    def update(self, dt_s):
        pass
