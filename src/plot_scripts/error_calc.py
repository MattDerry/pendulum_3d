import numpy as np
import sys
import csv
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
import math
from math import sin, cos, pi, exp, log
import trep.visual as visual
from matplotlib.pyplot import hold, plot
import pylab as mp

user_number = 1
max_trial_num = 30
file_name = '/home/mderry/cps_data/user_' + str(user_number) + '/user_' + str(user_number) + '_trust_log.csv'

DT = 1/50.
tf_freq = 100.
TIMESTEP = 20  # in ms

NORMALIZER = 5.51072018032

raw_trust_exp = np.zeros(0)
full_trust_exp = np.zeros(0)

raw_trust_ln = np.zeros(0)
full_trust_ln = np.zeros(0)

raw_trust_log = np.zeros(0)
full_trust_log = np.zeros(0)

X_e = np.zeros(0)
U_e = np.zeros(0)

MIN_COST = 248.77
alpha = 0.05
STARTING_TRUST = 0.1
current_trust_exp = STARTING_TRUST
current_trust_ln = STARTING_TRUST
raw_trust_exp = np.append(raw_trust_exp, 0)
full_trust_exp = np.append(full_trust_exp, STARTING_TRUST)
raw_trust_ln = np.append(raw_trust_ln, 0)
full_trust_ln = np.append(full_trust_ln, STARTING_TRUST)


# trep system generator:
class BalanceBoard(trep.System):
    def __init__(self, num_links, link_length, link_mass, damping):
        super(BalanceBoard, self).__init__()

        self.num_links = num_links
        self.link_length = link_length
        self.link_mass = link_mass

        print "Build balance board"
        bboard = trep.Frame(self.world_frame, trep.TX, 0)
        self._add_link(bboard, 1.0)

        trep.potentials.Gravity(self, (0, 0.0, -9.8))
        trep.forces.Damping(self, 1.0)
        trep.forces.ConfigForce(self, 'link-1-base_x', 'torque1_x')
        trep.forces.ConfigForce(self, 'link-1-base_y', 'torque1_y')

        print "Configuration Variables: %d (Kinematic: %d, Dynamic: %d), Inputs: %d, Constraints: %d", self.nQ, self.nQk, self.nQd, self.nu, self.nc

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
    print "Creating %d link balance board", 2
    return BalanceBoard(2, float(link_length), float(link_mass), float(damping))


system = create_systems(2, link_length='1.0', link_mass='2.0', frequency='0.5', amplitude='0.5', damping='0.1')
mvi = trep.MidpointVI(system)
tvec = np.arange(0, 500.0*TIMESTEP/1000.0, TIMESTEP/1000.0)
dsys = trep.discopt.DSystem(mvi, tvec)
link1_rx_index = dsys.system.get_config('link-1-base_x').index
link1_ry_index = dsys.system.get_config('link-1-base_y').index
link2_rx_index = dsys.system.get_config('link-2-base_x').index
link2_ry_index = dsys.system.get_config('link-2-base_y').index
qd_comb = np.zeros((len(tvec), mvi.system.nQ))
qd_comb[:] = [0]*mvi.system.nQ
(Xd_comb, Ud_comb) = dsys.build_trajectory(qd_comb)

dyn_weight = 50
kin_weight = 1
mom_weight = 0.01
vel_weight = kin_weight*TIMESTEP/1000.0
Q = np.diag(np.hstack(([dyn_weight]*system.nQd,
                      [kin_weight]*system.nQk,
                      [mom_weight]*system.nQd,
                      [vel_weight]*system.nQk)))
R = mom_weight*np.identity(2)

max_e_g = 0

EXPONENT = -0.0001
EXPONENT_NEW = -0.000231
LOG_EXP = -0.06
for k in range(3):
    for j in range(6):
        raw_trust_exp = np.zeros(0)
        full_trust_exp = np.zeros(0)
        raw_trust_log = np.zeros(0)
        full_trust_log = np.zeros(0)
        X_e = np.zeros(1)
        U_e = np.zeros(1)
        user_number = j+1
        for i in range(max_trial_num):
            if k == 0:
                user_name = 'Matt'
            elif k == 1:
                user_name = 'Alex'
            else:
                user_name = 'Brenna'
            title_str = 'User ' + str(k+1) + ': Session: ' + str(user_number)
            filestr = '/home/mderry/cps_data/' + user_name + '/user_' + str(user_number) + '/user_' + str(user_number) + '_trial_' + str(i) + '_combined_trajectory'
            matext = '.mat'
            csvext = '.csv'
            (Xd, Ud) = dsys.load_state_trajectory(filestr+matext)
            dcost = discopt.DCost(Xd_comb, Ud_comb, Q, R)
            optimizer = discopt.DOptimizer(dsys, dcost)
            tcost = optimizer.calc_cost(Xd, Ud)

            Xt = Xd_comb - Xd
            Xt = np.square(Xt)
            Ut = (Ud_comb - Ud)
            Ut = np.square(Ut)
            X_e_acc = np.sum(Xt)
            U_e_acc = np.sum(Ut)

            X_e = np.append(X_e, X_e_acc)
            U_e = np.append(U_e, U_e_acc)
            # print "calc_cost: %f" % tcost

            #if tcost > MAX_COST:
            #    tcost = MAX_COST
            if tcost < MIN_COST:
                tcost = MIN_COST
            # self.task_trust = 1.0 - ((optimizer.calc_cost(self.state_trajectory, self.combined_input_trajectory) - MIN_COST)/(MAX_COST-MIN_COST))
            task_trust_exp = exp(EXPONENT_NEW * (tcost - MIN_COST))
            task_trust_ln = exp(LOG_EXP * log((tcost - MIN_COST), 2))
            # task_trust_ln =
            if task_trust_exp > 1.0:
                task_trust_exp = 1.0
            elif task_trust_exp < 0.0:
                task_trust_exp = 0.0
            current_trust_exp = (1-alpha)*current_trust_exp + alpha*task_trust_exp
            raw_trust_exp = np.append(raw_trust_exp, task_trust_exp)
            full_trust_exp = np.append(full_trust_exp, current_trust_exp)

            if task_trust_ln > 1.0:
                task_trust_ln = 1.0
            elif task_trust_ln < 0.0:
                task_trust_ln = 0.0
            current_trust_ln = (1-alpha)*current_trust_ln + alpha*task_trust_ln
            raw_trust_ln = np.append(raw_trust_ln, task_trust_ln)
            full_trust_ln = np.append(full_trust_ln, current_trust_ln)

        file_name = '/home/mderry/cps_data/' + user_name + '/user_' + str(user_number) + '/user_' + str(user_number) + '_trust_log.csv'
        with open(file_name, 'rb') as csvfile:
            dialect = csv.Sniffer().sniff(csvfile.read(1024))
            csvfile.seek(0)
            reader = csv.reader(csvfile, dialect)
            count = 0
            for row in reader:
                if count > 0:
                    raw_trust_log = np.append(raw_trust_log, float(row[2]))
                    full_trust_log = np.append(full_trust_log, float(row[3]))
                count = count + 1

        max_xe = np.max(X_e)

        X_e = X_e / max_xe
        acc = 0
        for x in range(len(X_e)):
            acc = acc + X_e[x]
            X_e[x] = acc

        X_e = X_e / NORMALIZER
        outfilestr = '/home/mderry/cps_data/plot_data/user_' + str(k+1) + '_session' + str(j+1) + '.csv'
        error_log_file = open(outfilestr, 'b')
        log_writer = csv.writer(error_log_file, delimiter=',', quotechar="'", quoting=csv.QUOTE_NONE)
        for g in range(len(full_trust_log)):
            log_writer.writerow([str(X_e[g]), str(full_trust_log[g])])
        error_log_file.close()
        # mp.hold(True)
        # mp.plot(full_trust_log, color='purple', label='original exponential', linewidth=4)
        # mp.plot(X_e, color='red', label='accumulated error', linewidth=4)
        # # mp.plot(raw_trust_exp, color='orange', linewidth=1)
        # # mp.plot(full_trust_exp, color='orange', label='modified exponential', linewidth=4)
        # # mp.plot(raw_trust_ln, color='green', linewidth=1)
        # # mp.plot(full_trust_ln, color='green', label='log2 scaled exponential', linewidth=4)
        # mp.legend()
        # mp.title(title_str)
        # mp.xlabel('Trial Number')
        # mp.ylabel('Trust')
        # mp.hold(False)
        # mp.show()
