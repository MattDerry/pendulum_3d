#!/usr/bin/env python
"""
Matt Derry
Summer 2014

Simulate spherical inverted pendulum and publish results so that
ROS can be visualization.

"""

## define all imports:
import roslib
roslib.load_manifest('pendulum_3d')
import rospy
import tf
from sensor_msgs.msg import JointState as JS
import std_srvs.srv as SS
import geometry_msgs.msg as GM
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
import trep
from trep import discopt
from math import pi, fmod, atan2, sin
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from pendulum_3d.cfg import PendulumConfig


####################
# GLOBAL VARIABLES #
####################
DT = 1/50.
tf_freq = 100.
TIMESTEP = 20  # in ms
EXCEPTION_COUNT_MAX = 5
DISTURBANCE_MAGNITUDE = 2.0

# Pendulum params:
MAX_LINKS = 2

# Weight limits:
MAX_WEIGHT = 1000000000
MIN_WEIGHT = 0.0001
KIN_WEIGHT = 1.0

# Trust limits:
MAX_TRUST = 1.0
MIN_TRUST = 0.0
CURRENT_TRUST = 1.0

# Saturation:
MAX_VELOCITY = 0.25/(TIMESTEP/1000.0)

MAX_TORQUE = 250.0
SATURATION_TORQUE = 1000.0

MIN_SCALE = 0.1
MAX_SCALE = 2.0

STATE_INTERACTIVE = 0
STATE_CONTROLLED_INTERACTIVE = 1

QBAR_END = 0.0
QBAR_DIP = pi/8
QBAR_DIP1 = QBAR_DIP
QBAR_DIP2 = QBAR_DIP
TS_TRANSITION_1 = 100
TS_TRANSITION_2 = 200


###########################
# MISCELLANEOUS FUNCTIONS #
###########################


# map to 0 to 2pi
def normalize_angle_positive(angle):
    return fmod(fmod(angle, 2.0*pi) + 2.0*pi, 2.0*pi)


# map to -pi to +pi
def normalize_angle(angle):
    a = normalize_angle_positive(angle)
    if a > pi:
        a -= 2.0*pi
    return a


# utility function for getting minimum angular distance between two angles in
# radians.  Answer will be -pi <= result <= pi, adding result to 'before' will
# always be an angle equivalent to 'after'
def shortest_angular_distance(before, after):
    result = normalize_angle_positive(normalize_angle_positive(after) -
                                      normalize_angle_positive(before))
    if result > pi:
        result = -(2.0*pi-result)
    return normalize_angle(result)


# trep system generator:
class BalanceBoard(trep.System):
    def __init__(self, num_links, link_length, link_mass, damping):
        super(BalanceBoard, self).__init__()

        self.num_links = num_links
        self.link_length = link_length
        self.link_mass = link_mass

        rospy.loginfo("Build balance board")
        bboard = trep.Frame(self.world_frame, trep.TX, 0)
        self._add_link(bboard, 1.0)

        trep.potentials.Gravity(self, (0, 0.0, -9.8))
        trep.forces.Damping(self, 1.0)
        trep.forces.ConfigForce(self, 'link-1-base_x', 'torque1_x')
        trep.forces.ConfigForce(self, 'link-1-base_y', 'torque1_y')

        rospy.loginfo("Configuration Variables: %d (Kinematic: %d, Dynamic: %d), Inputs: %d, Constraints: %d", self.nQ, self.nQk, self.nQd, self.nu, self.nc)

    def _add_link(self, parent, link):
        if link == self.num_links+1:
            return
        base1 = trep.Frame(parent, trep.RX, 'link-%d-base_x' % link,
                           'link-%d-base_x' % link)
        base2 = trep.Frame(base1, trep.RY, 'link-%d-base_y' % link,
                           'link-%d-base_y' % link)
        end = trep.Frame(base2, trep.TZ, self.link_length,
                         mass=self.link_mass, name=('link-%d' % link))
        self._add_link(end, link+1)


def create_systems(max_links, link_length, link_mass, frequency, amplitude, damping):
    """
    Creates the balance board and loads or generates the trajectories.
    """
    rospy.loginfo("Creating %d link balance board" % 2)
    return BalanceBoard(2, float(link_length), float(link_mass), float(damping))


class PendulumSimulator:
    def __init__(self, sys):
        rospy.loginfo("Starting pendulum simulator node")
        self.links_bool = rospy.get_param('links', False)
        self.finished = False
        self.create_timer_marker()
        # first, let's just define the initial configuration of the system:
        self.sys = sys
        self.mvi = trep.MidpointVI(self.sys)
        # self.start_interactive()
        self.start_controlled_interactive()

        # fill out a message, and store it in the class so that I can update it
        # whenever necessary
        self.mappings = {
            'link1_rx_link': 'link-1-base_x',
            'link1_ry_link': 'link-1-base_y',
            'link2_rx_link': 'link-2-base_x',
            'link2_ry_link': 'link-2-base_y',
            }
        self.js = JS()
        self.names = [x for x in self.mappings.keys()]
        self.js.name = self.names
        self.js.header.frame_id = 'base'
        self.update_values()
        self.count_exceptions = 0

        self.user_ux = 0.0
        self.user_uy = 0.0

        self.reset_button_prev = 0
        self.reset_button = 0

        self.trust = rospy.get_param('~trust', CURRENT_TRUST)
        self.max_torque = rospy.get_param('~max_torque', MAX_TORQUE)
        self.state = rospy.get_param('~state', STATE_CONTROLLED_INTERACTIVE)

        self.user_markers = []
        self.user_markers.append(self.create_marker(1))
        self.user_markers.append(self.create_marker(2))

        self.create_task_marker()

        # define tf broadcaster and listener
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # define a publisher for the joint states
        self.joint_pub = rospy.Publisher("joint_states", JS)
        # define a timer for publishing the frames and tf's
        self.server = DynamicReconfigureServer(PendulumConfig, self.reconfigure)

        rospy.Timer(rospy.Duration(1.0/tf_freq), self.send_joint_states)

        # offer service to reset simulation
        self.reset_srv_provider = rospy.Service("simulator_reset", SS.Empty, self.reset_provider)
        # define a timer for integrating the state of the VI
        rospy.Subscriber("/board_joy", Joy, self.joy_cb)
        #rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.pub = rospy.Publisher("/user_torque_markers", Marker, queue_size=10)
        self.timer_pub = rospy.Publisher("/task_timer", Marker, queue_size=1)
        self.task_pub = rospy.Publisher("/task_direction", Marker, queue_size=1)
        self.task_pub.publish(self.task_marker)
        rospy.loginfo("Starting integration...")
        rospy.Timer(rospy.Duration(DT), self.update_interactive)

    def create_marker(self, mid):
        user_marker = Marker()
        user_marker.header.frame_id = 'base'
        user_marker.header.stamp = rospy.Time.now()
        user_marker.id = mid
        user_marker.type = 0
        user_marker.action = 0
        if mid == 1:
            user_marker.pose.position.x = 0
            user_marker.pose.position.y = 2.0
            user_marker.pose.position.z = 0
            user_marker.pose.orientation = GM.Quaternion(*(tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx')))
        elif mid == 2:
            user_marker.pose.position.x = 2.0
            user_marker.pose.position.y = 0.0
            user_marker.pose.position.z = 0
            user_marker.pose.orientation = GM.Quaternion(*(tf.transformations.quaternion_from_euler(-pi/2, 0, 0, 'rzyx')))
        user_marker.color.r = 1.0
        user_marker.color.g = 1.0
        user_marker.color.b = 0.0
        user_marker.color.a = 1.0
        user_marker.scale.z = 0.01

        return user_marker

    def create_task_marker(self):
        self.task_marker = Marker()
        self.task_marker.header.frame_id = 'base'
        self.task_marker.header.stamp = rospy.Time.now()
        #self.task_marker.lifetime = 4.0
        self.task_marker.id = 1
        self.task_marker.type = 0
        self.task_marker.action = 0
        self.task_marker.pose.position.x = 0.0
        self.task_marker.pose.position.y = 0.0
        self.task_marker.pose.position.z = 0.0
        rospy.loginfo("task heading: %f", atan2(QBAR_DIP1, QBAR_DIP2))
        self.task_marker.pose.orientation = GM.Quaternion(*(tf.transformations.quaternion_from_euler(atan2(QBAR_DIP1, QBAR_DIP2), 0, 0, 'rzyx')))
        self.task_marker.color.r = 1.0
        self.task_marker.color.g = 1.0
        self.task_marker.color.b = 0.0
        self.task_marker.color.a = 1.0
        self.task_marker.scale.z = 1.0
        self.task_marker.scale.x = 1.0
        self.task_marker.scale.y = 1.0

    ###########################
    # Timer Marker
    ###########################
    def create_timer_marker(self):
        self.timer_marker = Marker()
        self.timer_marker.header.frame_id = 'base'
        self.timer_marker.header.stamp = rospy.Time.now()
        self.timer_marker.id = 1
        self.timer_marker.type = 9
        self.timer_marker.action = 0
        self.timer_marker.pose.position.x = 0
        self.timer_marker.pose.position.y = 2.0
        self.timer_marker.pose.position.z = 0
        self.timer_marker.text = "0"
        self.timer_marker.color.r = 1.0
        self.timer_marker.color.g = 1.0
        self.timer_marker.color.b = 0.0
        self.timer_marker.color.a = 1.0
        self.timer_marker.scale.z = 0.5

    def joy_cb(self, data):
        self.joy = data
        self.user_ux = self.max_torque * -data.axes[0]
        self.user_uy = self.max_torque * data.axes[1]

        self.reset_button_prev = self.reset_button
        self.reset_button = data.buttons[0]

        if self.reset_button_prev and not self.reset_button:
            self.reset()

    def reconfigure(self, config, level):
        # Fill in local variables with values received from
        # dynamic reconfigure clients (typically the GUI).
        self.trust = config["trust"]
        self.max_torque = config["max_torque"]
        self.state = config["state"]
        return config

    def reset_provider(self, req):
        self.reset()
        return SS.EmptyResponse()

    def reset(self):
        if self.state == STATE_CONTROLLED_INTERACTIVE:
            self.start_controlled_interactive()
        else:
            self.start_interactive()

    def start_controlled_interactive(self):
        self.state = STATE_CONTROLLED_INTERACTIVE
        self.simulation_failed = False

        # calculate linearization, and feedback controller:
        tvec = np.arange(0, 500.0*TIMESTEP/1000.0, TIMESTEP/1000.0)
        self.dsys = trep.discopt.DSystem(self.mvi, tvec)
        link1_rx_index = self.dsys.system.get_config('link-1-base_x').index
        link1_ry_index = self.dsys.system.get_config('link-1-base_y').index
        link2_rx_index = self.dsys.system.get_config('link-2-base_x').index
        link2_ry_index = self.dsys.system.get_config('link-2-base_y').index

        # Variables to hold executed trajectories for later cost calc
        self.state_trajectory = np.zeros((len(tvec), 8))
        self.input_trajectory = np.zeros((len(tvec)-1, 2))

        qd_up = np.zeros((len(tvec), self.mvi.system.nQ))
        qd_up[:] = [0]*self.mvi.system.nQ

        qd_dip = np.zeros((len(tvec), self.mvi.system.nQ))
        qd_dip[:] = [0]*self.mvi.system.nQ

        for i, t in enumerate(tvec):
            if i < 400 and i > 200:
                qd_dip[i, link1_rx_index] = QBAR_DIP  # Set desired configuration trajectory
                qd_dip[i, link1_ry_index] = QBAR_DIP
                qd_dip[i, link2_rx_index] = -1*QBAR_DIP
                qd_dip[i, link2_ry_index] = -1*QBAR_DIP

        qd_state_dip = np.zeros((1, self.mvi.system.nQ))
        qd_state_dip[0, link1_rx_index] = QBAR_DIP
        qd_state_dip[0, link1_ry_index] = QBAR_DIP
        qd_state_dip[0, link2_rx_index] = -1*QBAR_DIP
        qd_state_dip[0, link2_ry_index] = -1*QBAR_DIP
        self.xBar_end = self.dsys.build_state()
        self.xBar_dip = self.dsys.build_state(qd_state_dip)
        (Xd_up, Ud_up) = self.dsys.build_trajectory(qd_up)  # Set desired state and input trajectory
        (Xd_dip, Ud_dip) = self.dsys.build_trajectory(qd_dip)

        rospy.loginfo("Xd: %d", len(Xd_up))
        dyn_weight = 50
        kin_weight = 1
        mom_weight = 0.01
        vel_weight = kin_weight*TIMESTEP/1000.0
        Q = np.diag(np.hstack(([dyn_weight]*self.sys.nQd,
                               [kin_weight]*self.sys.nQk,
                               [mom_weight]*self.sys.nQd,
                               [vel_weight]*self.sys.nQk)))
        R = mom_weight*np.identity(2)
        rospy.loginfo("Q shape: %s, R shape %s", Q.shape, R.shape)
        self.Qk = lambda k: Q
        self.Rk = lambda k: R
        (Kstab_up, A_up, B_up) = self.dsys.calc_feedback_controller(Xd_up, Ud_up, self.Qk, self.Rk, return_linearization=True)
        (Kstab_dip, A_dip, B_dip) = self.dsys.calc_feedback_controller(Xd_dip, Ud_dip, self.Qk, self.Rk, return_linearization=True)
        self.Kstab_up = Kstab_up
        self.Kstab_dip = Kstab_dip
        self.k = 0

        START_DIP = pi/6.7
        SD1 = 0.01
        SD2 = 0.01
        # set initial system state
        q0 = (SD2, SD1, -1*SD2, -1*SD1)
        q1 = (SD1, SD2, -1*SD2, -1*SD1)
        self.mvi.initialize_from_configs(0.0, q0, DT, q1)

        self.disp_q = self.mvi.q2
        self.disp_qd = np.hstack((self.mvi.q2[0:self.sys.nQd], [0]*self.sys.nQk))

    def update_markers(self, mid):
        if mid == 1:
            self.user_markers[0].scale.x = MIN_SCALE + (MAX_SCALE-MIN_SCALE) * self.user_uy/self.max_torque
            self.user_markers[0].scale.y = MIN_SCALE + (MAX_SCALE-MIN_SCALE) * 0.2 * self.user_uy/self.max_torque
            if abs(self.user_uy/self.max_torque) > 0:
                self.user_markers[0].color.r = 1.0
                self.user_markers[0].color.g = 1.0
                self.user_markers[0].color.b = 0.0
            else:
                self.user_markers[0].color.r = 0.0
                self.user_markers[0].color.g = 0.0
                self.user_markers[0].color.b = 0.0
        elif mid == 2:
            self.user_markers[1].scale.x = MIN_SCALE + (MAX_SCALE-MIN_SCALE) * self.user_ux/self.max_torque
            self.user_markers[1].scale.y = MIN_SCALE + (MAX_SCALE-MIN_SCALE) * 0.2 * self.user_ux/self.max_torque
            if abs(self.user_ux/self.max_torque) > 0:
                self.user_markers[1].color.r = 0.0
                self.user_markers[1].color.g = 0.0
                self.user_markers[1].color.b = 1.0
            else:
                self.user_markers[1].color.r = 0.0
                self.user_markers[1].color.g = 0.0
                self.user_markers[1].color.b = 0.0

    def update_interactive(self, event):
        if self.simulation_failed:
            return

        self.update_markers(1)
        self.user_markers[0].header.stamp = rospy.Time.now()
        self.pub.publish(self.user_markers[0])
        self.update_markers(2)
        self.user_markers[1].header.stamp = rospy.Time.now()
        self.pub.publish(self.user_markers[1])

        u = self.mvi.u1
        u[0] = self.user_ux
        u[1] = self.user_uy

        ## if we are in interactive+control mode, let's run that controller:
        if self.state == STATE_CONTROLLED_INTERACTIVE:
            # get state stuff
            qtmp = self.mvi.q2
            ptmp = self.mvi.p2
            # wrap angle of pendulum:
            for q in self.sys.dyn_configs:
                q0 = qtmp[self.sys.get_config(q).index]
                qtmp[self.sys.get_config(q).index] = normalize_angle(q0)
            X = np.hstack((qtmp, ptmp))
            # calculate feedback law
            if self.k < 400 and self.k > 200:
                xTilde = X - self.xBar_dip
                u_cont = -np.dot(self.Kstab_dip[0], xTilde)
                # rospy.loginfo("X[0]: %f (%f), X[1]: %f (%f)", X[0], (1.0 * 9.81 * 2.0 * sin(X[0])), X[1], (1.0 * 9.81 * 2.0 * sin(X[1])))
                # u_cont[0] = u_cont[0] + (1.0 * 9.81 * 2.0 * sin(X[0]))
                # u_cont[1] = u_cont[1] + (1.0 * 9.81 * 2.0 * sin(X[1]))
                if self.k == 201:
                    rospy.loginfo("DIP!")
                    rospy.loginfo(xTilde)
            else:
                xTilde = X - self.xBar_end
                u_cont = -np.dot(self.Kstab_up[0], xTilde)

            if self.k == 400:
                rospy.loginfo("POP!")
                rospy.loginfo(xTilde)

            if u_cont[0] > SATURATION_TORQUE:
                u_cont[0] = SATURATION_TORQUE
            if u_cont[1] > SATURATION_TORQUE:
                u_cont[1] = SATURATION_TORQUE

            # blend user and feedback control input
            u[0] = self.trust*self.user_ux + (1-self.trust)*u_cont[0]
            u[1] = self.trust*self.user_uy + (1-self.trust)*u_cont[1]
            # rospy.loginfo("[before] u: (%f, %f), user: (%f, %f), controller: (%f, %f)", u[0], u[1], self.user_ux, self.user_uy, u_cont[0], u_cont[1])

        try:
            self.k += 1
            t2 = self.k*TIMESTEP/1000.0
            self.mvi.step(t2, u1=u)
        except trep.ConvergenceError:
            self.simulation_failed = True
            rospy.logwarn("No solution to DEL equations!")
            return

        self.disp_q = self.mvi.q2
        self.disp_qd = np.hstack((self.mvi.q2[0:self.sys.nQd], [0]*self.sys.nQk))
        self.update_values()

    def update_values(self):
        """
        Just fill in all of position array stuff for the JS message
        """
        pos = [self.sys.get_config(self.mappings[n]).q for n in self.names]
        self.js.position = pos

    def send_joint_states(self, event):
        tnow = rospy.Time.now()
        # first send the transform:
        quat = tuple(tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx'))
        point = tuple((0, 0, 0))
        self.br.sendTransform(point, quat,
                              tnow,
                              'base', 'world')
        self.js.header.stamp = tnow
        self.joint_pub.publish(self.js)

        # send transforms
        quat = tuple(tf.transformations.quaternion_from_euler(self.sys.get_config('link-1-base_x').q, 0, 0, 'sxyz'))
        point = tuple((0, 0, 0))
        self.br.sendTransform(point, quat, tnow, 'link1_rx_link', 'base')
        quat = tuple(tf.transformations.quaternion_from_euler(0, self.sys.get_config('link-1-base_y').q, 0, 'sxyz'))
        point = tuple((0, 0, 0))
        self.br.sendTransform(point, quat, tnow, 'link1_ry_link', 'link1_rx_link')
        quat = tuple(tf.transformations.quaternion_from_euler(self.sys.get_config('link-2-base_x').q, 0, 0, 'sxyz'))
        point = tuple((0, 0, 0.5))
        self.br.sendTransform(point, quat, tnow, 'link2_rx_link', 'link1')
        quat = tuple(tf.transformations.quaternion_from_euler(0, self.sys.get_config('link-2-base_y').q, 0, 'sxyz'))
        point = tuple((0, 0, 0))
        self.br.sendTransform(point, quat, tnow, 'link2_ry_link', 'link2_rx_link')


def main():
    """
    Run the main loop, by instatiating a PendulumSimulator, and then
    calling ros.spin
    """
    rospy.init_node('pendulum_simulator')  # , log_level=rospy.INFO)

    # check what the value of the links param is
    links_bool = rospy.get_param('links', False)
    if not rospy.has_param('links'):
        rospy.set_param('links', links_bool)

    try:
        system = create_systems(2,
                                link_length='1.0',
                                link_mass='2.0',
                                frequency='0.5',
                                amplitude='0.5',
                                damping='0.1')
        sim = PendulumSimulator(system)

    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__ == '__main__':
    main()
