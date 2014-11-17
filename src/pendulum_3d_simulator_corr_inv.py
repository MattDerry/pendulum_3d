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
from pendulum_3d.msg import *
import std_srvs.srv as SS
import geometry_msgs.msg as GM
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
import trep
from trep import discopt
from math import pi, fmod, atan2, sin, exp
import numpy as np
import scipy.signal as sp
import matplotlib as mpl
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from pendulum_3d.cfg import PendulumConfig
import csv
import sys
import time


#####################
# Experiment Settings
#####################
USER_NUMBER = 1
MAX_TRIALS = 5
STARTING_TRUST = 0.3
MAX_TORQUE = 100.0

INPUT_DEVICE = "balance_board"  # "joystick"
ALPHA = 0.05

EXPONENT = -0.0001

MAX_COST = 10000
MIN_COST = 500

####################
# GLOBAL VARIABLES #
####################
DT = 1/50.
tf_freq = 30.
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
CURRENT_TRUST = 0.0

# Saturation:
MAX_VELOCITY = 0.25/(TIMESTEP/1000.0)

SATURATION_TORQUE = 1000.0

MIN_SCALE = 0.1
MAX_SCALE = 2.0

STATE_INTERACTIVE = 0
STATE_CONTROLLED_INTERACTIVE = 1

QBAR_END = pi
QBAR_DIP = pi/8
QBAR_DIP1 = QBAR_DIP
QBAR_DIP2 = QBAR_DIP
TS_TRANSITION_1 = 200
TS_TRANSITION_2 = 400

CURRENT_TRIAL_NUMBER = 0


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


class User:
    def __init__(self, user_number, max_number_of_trials, starting_trust, trial_alpha, input_device):
        self.user_id = user_number
        self.max_num_trials = max_number_of_trials
        self.current_trial_num = 0
        self.starting_trust = starting_trust
        self.input_device = input_device
        self.current_trust = starting_trust
        self.alpha = trial_alpha
        param_file_name = '/home/mderry/cps_data/user_' + str(user_number) + '/user_' + str(user_number) + '_params.csv'
        log_file_name = '/home/mderry/cps_data/user_' + str(user_number) + '/user_' + str(user_number) + '_trust_log.csv'
        with open(param_file_name, 'wb') as csvfile:
            paramwriter = csv.writer(csvfile, delimiter=',', quotechar="'", quoting=csv.QUOTE_MINIMAL)
            paramwriter.writerow(['User Number', 'Input Device', 'Max_Num_Trials', 'Starting Trust', 'Alpha'])
            paramwriter.writerow([str(user_number), input_device, str(max_number_of_trials), str(starting_trust), str(trial_alpha)])

        self.trust_log_file = open(log_file_name, 'wb')
        self.log_writer = csv.writer(self.trust_log_file, delimiter=',', quotechar="'", quoting=csv.QUOTE_NONE)
        self.log_writer.writerow(['Trial Number', 'Current Trust', 'Raw Task Trust', 'New Trust', 'Raw Task Cost'])
        self.write_to_log(0, starting_trust, 0, starting_trust, 0)  # no single task trust yet, so just set to zero for now

    def update_trust(self, new_trust, new_cost):
        self.current_trial_num = self.current_trial_num + 1
        adjusted_trust = (1-self.alpha) * self.current_trust + self.alpha * new_trust
        self.write_to_log(self.current_trial_num, self.current_trust, new_trust, adjusted_trust, new_cost)
        self.current_trust = adjusted_trust
        if self.current_trial_num == self.max_num_trials:
            self.trust_log_file.close()

    def get_current_trust(self):
        return self.current_trust

    def set_alpha(self, new_alpha):
        self.alpha = new_alpha

    def write_to_log(self, t_num, current_trust, single_task_trust, adjusted_trust, single_task_cost):
        if self.trust_log_file.closed:
            log_file_name = '/home/mderry/cps_data/user_' + str(USER_NUMBER) + '/user_' + str(USER_NUMBER) + '_trust_log.csv'
            self.trust_log_file = open(log_file_name, 'ab')
            self.log_writer = csv.writer(self.trust_log_file, delimiter=',', quotechar="'", quoting=csv.QUOTE_NONE)
            self.log_writer.writerow([str(t_num), str(current_trust), str(single_task_trust), str(adjusted_trust), str(single_task_cost)])
            self.trust_log_file.close()
        else:
            self.log_writer.writerow([str(t_num), str(current_trust), str(single_task_trust), str(adjusted_trust), str(single_task_cost)])


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
    def __init__(self, syst, trust, trial_num):
        rospy.loginfo("Creating pendulum simulator")
        self.links_bool = rospy.get_param('links', False)
        self.create_timer_marker()
        # first, let's just define the initial configuration of the system:
        self.sys = syst
        self.mvi = trep.MidpointVI(self.sys)
        # self.start_interactive()
        self.start_controlled_interactive()

        self.trial_num = trial_num

        self.finished = False

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

        self.trust = trust
        self.max_torque = MAX_TORQUE  # rospy.get_param('~max_torque', MAX_TORQUE)
        self.state = STATE_CONTROLLED_INTERACTIVE  # rospy.get_param('~state', STATE_CONTROLLED_INTERACTIVE)

        self.create_task_marker()
        self.create_target_marker()
        self.create_score_msg()

        # define tf broadcaster and listener
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # define a publisher for the joint states
        self.joint_pub = rospy.Publisher("joint_states", JS, queue_size=2)
        # define a timer for publishing the frames and tf's

        self.tf_timer = rospy.Timer(rospy.Duration(1.0/tf_freq), self.send_joint_states)

        # define a timer for integrating the state of the VI
        rospy.Subscriber("/board_joy", Joy, self.joy_cb)
        #rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.timer_pub = rospy.Publisher("/task_timer", Marker, queue_size=1)
        self.task_pub = rospy.Publisher("/task_direction", Marker, queue_size=2)
        self.target_pub = rospy.Publisher("/task_target", Marker, queue_size=2)
        self.score_pub = rospy.Publisher("/score", Score, queue_size=2)
        time.sleep(0.5)
        self.timer_marker.header.stamp = rospy.Time.now()
        self.timer_marker.text = "START"
        self.timer_pub.publish(self.timer_marker)
        self.task_pub.publish(self.task_marker)
        self.target_marker.header.stamp = rospy.Time.now()
        self.target_pub.publish(self.target_marker)
        self.timer = rospy.Timer(rospy.Duration(DT), self.update_interactive)
        rospy.loginfo("Starting integration...")

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
        # rospy.loginfo("task heading: %f", atan2(QBAR_DIP1, QBAR_DIP2))
        self.task_marker.pose.orientation = GM.Quaternion(*(tf.transformations.quaternion_from_euler(-1*atan2(QBAR_DIP1, QBAR_DIP2), 0, 0, 'rzyx')))
        self.task_marker.color.r = 1.0
        self.task_marker.color.g = 1.0
        self.task_marker.color.b = 0.0
        self.task_marker.color.a = 1.0
        self.task_marker.scale.z = 0.01
        self.task_marker.scale.x = 0.750
        self.task_marker.scale.y = 0.15

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

    def create_target_marker(self):
        self.target_marker = Marker()
        self.target_marker.header.frame_id = 'base'
        self.target_marker.header.stamp = rospy.Time.now()
        self.target_marker.id = 1
        self.target_marker.type = 2
        self.target_marker.action = 0
        self.target_marker.pose.position.x = 0.0
        self.target_marker.pose.position.y = 0.0
        self.target_marker.pose.position.z = 2.0
        self.target_marker.text = "0"
        self.target_marker.color.r = 0.0
        self.target_marker.color.g = 0.0
        self.target_marker.color.b = 1.0
        self.target_marker.color.a = 0.5
        self.target_marker.scale.x = 0.35
        self.target_marker.scale.y = 0.35
        self.target_marker.scale.z = 0.35

    def create_score_msg(self):
        self.score = Score()

    def joy_cb(self, data):
        self.joy = data
        self.user_ux = self.max_torque * data.axes[0]
        self.user_uy = self.max_torque * -data.axes[1]

        self.reset_button_prev = self.reset_button
        self.reset_button = data.buttons[0]

        if self.reset_button_prev and not self.reset_button:
            self.reset()

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

        qd_up = np.zeros((len(tvec), self.mvi.system.nQ))
        qd_up[:] = [0]*self.mvi.system.nQ
        qd_up[:,link1_rx_index] = QBAR_END
        #qd_up[:,link1_ry_index] = QBAR_END

        qd_dip = np.zeros((len(tvec), self.mvi.system.nQ))
        qd_dip[:] = [0]*self.mvi.system.nQ
        qd_dip[:,link1_rx_index] = QBAR_END
        #qd_dip[:,link1_ry_index] = QBAR_END

        self.qd_comb = np.zeros((len(tvec), self.mvi.system.nQ))
        self.qd_comb[:] = [0]*self.mvi.system.nQ
        self.qd_comb[:,link1_rx_index] = QBAR_END
        #self.qd_comb[:,link1_ry_index] = QBAR_END

        for i, t in enumerate(tvec):
            if i > 200 and i < 400:
                qd_dip[i, link1_rx_index] = normalize_angle(QBAR_END + QBAR_DIP)  # Set desired configuration trajectory
                qd_dip[i, link1_ry_index] = normalize_angle(QBAR_DIP)
                qd_dip[i, link2_rx_index] = -1*QBAR_DIP
                qd_dip[i, link2_ry_index] = -1*QBAR_DIP
                self.qd_comb[i, link1_rx_index] = normalize_angle(QBAR_END + QBAR_DIP)  # Set desired configuration trajectory
                self.qd_comb[i, link1_ry_index] = normalize_angle(QBAR_DIP)
                self.qd_comb[i, link2_rx_index] = -1*QBAR_DIP
                self.qd_comb[i, link2_ry_index] = -1*QBAR_DIP

        qd_state_dip = np.zeros((1, self.mvi.system.nQ))
        qd_state_dip[0, link1_rx_index] = normalize_angle(QBAR_END + QBAR_DIP)
        qd_state_dip[0, link1_ry_index] = normalize_angle(QBAR_DIP)
        qd_state_dip[0, link2_rx_index] = -1*QBAR_DIP
        qd_state_dip[0, link2_ry_index] = -1*QBAR_DIP
        self.xBar_end = self.dsys.build_state()
        self.xBar_dip = self.dsys.build_state(qd_state_dip)
        (Xd_up, Ud_up) = self.dsys.build_trajectory(qd_up)  # Set desired state and input trajectory
        (Xd_dip, Ud_dip) = self.dsys.build_trajectory(qd_dip)
        (Xd_comb, Ud_comb) = self.dsys.build_trajectory(self.qd_comb)
        print self.qd_comb[0,:]
        print self.qd_comb[300,:]
        self.Xd = Xd_comb
        self.Ud = Ud_comb

        # rospy.loginfo("X shape")
        # rospy.loginfo(Xd_up.shape)
        # rospy.loginfo("U shape")
        # rospy.loginfo(Ud_up.shape)

        self.state_trajectory = np.zeros(Xd_up.shape)
        self.user_input_trajectory = np.zeros(Ud_up.shape)
        self.controller_input_trajectory = np.zeros(Ud_up.shape)
        self.combined_input_trajectory = np.zeros(Ud_up.shape)

        # rospy.loginfo("Xd: %d", len(Xd_up))
        dyn_weight = 50
        kin_weight = 1
        mom_weight = 0.01
        vel_weight = kin_weight*TIMESTEP/1000.0
        self.Q = np.diag(np.hstack(([dyn_weight]*self.sys.nQd,
                                    [kin_weight]*self.sys.nQk,
                                    [mom_weight]*self.sys.nQd,
                                    [vel_weight]*self.sys.nQk)))
        self.R = mom_weight*np.identity(2)
        # rospy.loginfo("Q shape: %s, R shape %s", self.Q.shape, self.R.shape)
        self.Qk = lambda k: self.Q
        self.Rk = lambda k: self.R
        (Kstab_up, A_up, B_up) = self.dsys.calc_feedback_controller(Xd_up, Ud_up, self.Qk, self.Rk, return_linearization=True)
        (Kstab_dip, A_dip, B_dip) = self.dsys.calc_feedback_controller(Xd_dip, Ud_dip, self.Qk, self.Rk, return_linearization=True)
        self.Kstab_up = Kstab_up
        self.Kstab_dip = Kstab_dip
        self.k = 1
        self.state_trajectory[0] = self.xBar_end

        # set initial system state
        q0 = (QBAR_END, 0.1, 0.0, 0.0)
        q1 = (QBAR_END, 0.1, 0.0, 0.0)
        self.mvi.initialize_from_configs(0.0, q0, DT, q1)

        self.disp_q = self.mvi.q2
        self.disp_qd = np.hstack((self.mvi.q2[0:self.sys.nQd], [0]*self.sys.nQk))

    def calc_correlations(self):
        self.state_trajectory
        self.combined_input_trajectory
        self.user_input_trajectory
        self.controller_input_trajectory

        self.cmd_x_corr = np.correlate(self.user_input_trajectory[:,0], self.controller_input_trajectory[:,0], 'full')
        self.cmd_y_corr = np.correlate(self.user_input_trajectory[:,1], self.controller_input_trajectory[:,1], 'full')
        max_x_idx = np.argmax(self.cmd_x_corr)
        max_y_idx = np.argmax(self.cmd_y_corr)

        max_idx = 0
        max_corr = 0
        for i in range(len(self.cmd_x_corr)):
            if self.cmd_x_corr[i] + self.cmd_y_corr[i] > max_corr:
                max_idx = i
                max_corr = self.cmd_x_corr[i] + self.cmd_y_corr[i]

        self.score.cmd_corr = max_corr
        self.score.cmd_offset = max_idx - (len(self.user_input_trajectory[:,0]))
        self.score.cmd_x_corr = self.cmd_x_corr[max_x_idx]
        self.score.cmd_x_corr_offset = max_x_idx - (len(self.user_input_trajectory[:,0]))
        self.score.cmd_y_corr = self.cmd_y_corr[max_y_idx]
        self.score.cmd_y_corr_offset = max_y_idx - (len(self.user_input_trajectory[:,0]))
        rospy.loginfo("x cmd correlation: %f, index offset: %d", self.cmd_x_corr[max_x_idx], max_x_idx - (len(self.user_input_trajectory[:,0])))
        rospy.loginfo("y cmd correlation: %f, index offset: %d", self.cmd_y_corr[max_y_idx], max_y_idx - (len(self.user_input_trajectory[:,0])))
        rospy.loginfo("cmd correlation: %f, index offset: %d", max_corr, max_idx - (len(self.user_input_trajectory[:,0])))

        self.state_x1_corr = np.correlate(self.state_trajectory[:,0], self.Xd[:,0], 'full')
        self.state_y1_corr = np.correlate(self.state_trajectory[:,1], self.Xd[:,1], 'full')
        self.state_x2_corr = np.correlate(self.state_trajectory[:,2], self.Xd[:,2], 'full')
        self.state_y2_corr = np.correlate(self.state_trajectory[:,3], self.Xd[:,3], 'full')

        max_idx = 0
        max_corr = 0
        for i in range(len(self.state_x1_corr)):
            if self.state_x1_corr[i] + self.state_y1_corr[i] + self.state_x2_corr[i] + self.state_x2_corr[i]> max_corr:
                max_idx = i
                max_corr = self.state_x1_corr[i] + self.state_y1_corr[i] + self.state_x2_corr[i] + self.state_x2_corr[i]

        self.score.state_corr = max_corr
        self.score.state_offset = max_idx - (len(self.state_trajectory[:,0]))
        rospy.loginfo("state correlation: %f, index offset: %d", self.score.state_corr, self.score.state_offset)


    def update_interactive(self, event):
        if self.simulation_failed or self.k == 500:
            self.timer.shutdown()
            self.tf_timer.shutdown()

            self.tcost = 0

            if self.k == 500:
                self.timer_marker.header.stamp = rospy.Time.now()
                self.timer_marker.text = "STOP"
                self.timer_pub.publish(self.timer_marker)

            # Pad the trajectories so the costs are high, but not so high in the case of stabilization failure
            if self.k < 499:
                last_state = self.state_trajectory[self.k-1]
                last_combined_command = self.combined_input_trajectory[self.k-1]
                last_user_command = self.user_input_trajectory[self.k-1]
                last_controller_command = self.controller_input_trajectory[self.k-1]
                for i in range(self.k, 500):
                    self.state_trajectory[i] = last_state
                    self.combined_input_trajectory[i-1] = last_combined_command
                    self.user_input_trajectory[i-1] = last_user_command
                    self.controller_input_trajectory[i-1] = last_controller_command

            filestr = '/home/mderry/cps_data/user_' + str(USER_NUMBER) + '/user_' + str(USER_NUMBER) + '_trial_' + str(self.trial_num) + '_combined_trajectory.mat'
            self.dsys.save_state_trajectory(filestr, self.state_trajectory, self.combined_input_trajectory)
            filestr = '/home/mderry/cps_data/user_' + str(USER_NUMBER) + '/user_' + str(USER_NUMBER) + '_trial_' + str(self.trial_num) + '_user_trajectory.mat'
            self.dsys.save_state_trajectory(filestr, self.state_trajectory, self.user_input_trajectory)
            filestr = '/home/mderry/cps_data/user_' + str(USER_NUMBER) + '/user_' + str(USER_NUMBER) + '_trial_' + str(self.trial_num) + '_controller_trajectory.mat'
            self.dsys.save_state_trajectory(filestr, self.state_trajectory, self.controller_input_trajectory)

            dcost = discopt.DCost(self.Xd, self.Ud, self.Q, self.R)
            optimizer = discopt.DOptimizer(self.dsys, dcost)
            self.tcost = optimizer.calc_cost(self.state_trajectory, self.combined_input_trajectory)
            rospy.loginfo("calc_cost: %f", self.tcost)

            #if tcost > MAX_COST:
            #    tcost = MAX_COST
            if self.tcost < MIN_COST:
                self.tcost = MIN_COST

            self.score.cost = self.tcost
            # self.task_trust = 1.0 - ((optimizer.calc_cost(self.state_trajectory, self.combined_input_trajectory) - MIN_COST)/(MAX_COST-MIN_COST))
            self.task_trust = exp(EXPONENT * (self.tcost - MIN_COST))
            if self.task_trust > 1.0:
                self.task_trust = 1.0
            elif self.task_trust < 0.0:
                self.task_trust = 0.0

            self.calc_correlations()
            self.score_pub.publish(self.score)

            self.finished = True
            return

        u = self.mvi.u1
        u[0] = self.user_ux
        u[1] = self.user_uy

        if self.k % 50 == 0:
            rospy.loginfo("Clock: %d", self.k/50)
            self.timer_marker.header.stamp = rospy.Time.now()
            self.target_marker.header.stamp = rospy.Time.now()
            if self.k == 200:
                self.timer_marker.text = "DIP"
                self.target_marker.pose.position.x = sin(QBAR_DIP1) - 0.1
                self.target_marker.pose.position.y = -1*sin(QBAR_DIP2) + 0.1
                self.target_marker.pose.position.z = 1.95
            elif self.k == 400:
                self.timer_marker.text = "RETURN"
                self.target_marker.pose.position.x = 0.0
                self.target_marker.pose.position.y = 0.0
                self.target_marker.pose.position.z = 2.0
            elif self.k == 500:
                self.timer_marker.text = "STOP"
            elif self.k > 400:
                self.timer_marker.text = str(10-(self.k/50))
            elif self.k > 200:
                self.timer_marker.text = str(8-(self.k/50))
            elif self.k < 200:
                self.timer_marker.text = str(4-(self.k/50))
            self.timer_pub.publish(self.timer_marker)
            self.target_pub.publish(self.target_marker)

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
            if not self.in_basin_of_attraction(X):
                self.simulation_failed = True
                rospy.loginfo("Outside basin of attraction")

            self.state_trajectory[self.k] = X

            # calculate feedback law
            if self.k > 200 and self.k < 400:
                xTilde = X - self.xBar_dip
                u_cont = -np.dot(self.Kstab_dip[0], xTilde)

                #u_cont = u_cont + (0.75 * 9.81 * 2.0 * sin(QBAR_DIP))
                if self.k == 201:
                    rospy.loginfo("DIP!")
                    # rospy.loginfo(xTilde)
            else:
                xTilde = X - self.xBar_end
                u_cont = -np.dot(self.Kstab_up[0], xTilde)
                if self.k == 400:
                    rospy.loginfo("POP!")
                    # rospy.loginfo(xTilde)

            if u_cont[0] > SATURATION_TORQUE:
                u_cont[0] = SATURATION_TORQUE
            if u_cont[1] > SATURATION_TORQUE:
                u_cont[1] = SATURATION_TORQUE

            # blend user and feedback control input
            u[0] = self.trust*self.user_ux + (1-self.trust)*u_cont[0]
            u[1] = self.trust*self.user_uy + (1-self.trust)*u_cont[1]

            self.controller_input_trajectory[self.k-1] = u_cont
            self.user_input_trajectory[self.k-1, 0] = self.user_ux
            self.user_input_trajectory[self.k-1, 1] = self.user_uy
            self.combined_input_trajectory[self.k-1] = u

            # rospy.loginfo("[before] u: (%f, %f), user: (%f, %f), controller: (%f, %f)", u[0], u[1], self.user_ux, self.user_uy, u_cont[0], u_cont[1])

        try:
            self.k += 1
            t2 = self.k*TIMESTEP/1000.0
            u[0] = 0.0
            u[1] = 0.0
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

    def in_basin_of_attraction(self, state):
        BASIN_THRESHOLD = 2*pi/3
        if abs(state[0]) + abs(state[1]) > BASIN_THRESHOLD:
            # return False
            return True
        else:
            return True

    def output_log(self):
        self.file = 2


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

    count = 1

    try:
        system = create_systems(2, link_length='1.0', link_mass='2.0', frequency='0.5', amplitude='0.5', damping='0.1')
        user = User(USER_NUMBER, MAX_TRIALS, STARTING_TRUST, ALPHA, INPUT_DEVICE)
        r = rospy.Rate(100)
        while user.current_trial_num < user.max_num_trials:
            rospy.loginfo("Trial number: %d", count)
            sim = PendulumSimulator(system, user.current_trust, user.current_trial_num)
            while not sim.finished:
                r.sleep()
            rospy.loginfo("current trust: %f", user.current_trust)
            user.update_trust(sim.task_trust, sim.tcost)
            rospy.loginfo("new current trust: %f, task trust: %f", user.current_trust, sim.task_trust)
            del sim
            count = count + 1
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass

    rospy.loginfo('Session Complete')


if __name__ == '__main__':
    main()
