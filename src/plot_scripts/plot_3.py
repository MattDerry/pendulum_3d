import numpy as np
import sys
import csv
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt
import math
from math import sin, cos, pi
import trep.visual as visual
from matplotlib.pyplot import hold, plot
import pylab as mp

user_number_1 = 4
user_number_2 = 5
user_number_3 = 6
file_name_1 = '/home/mderry/cps_data/user_' + str(user_number_1) + '/user_' + str(user_number_1) + '_trust_log.csv'
file_name_2 = '/home/mderry/cps_data/user_' + str(user_number_2) + '/user_' + str(user_number_2) + '_trust_log.csv'
file_name_3 = '/home/mderry/cps_data/user_' + str(user_number_3) + '/user_' + str(user_number_3) + '_trust_log.csv'

raw_trust_1 = np.zeros(0)
full_trust_1 = np.zeros(0)
raw_cost_1 = np.zeros(0)
raw_trust_2 = np.zeros(0)
full_trust_2 = np.zeros(0)
raw_cost_2 = np.zeros(0)
raw_trust_3 = np.zeros(0)
full_trust_3 = np.zeros(0)
raw_cost_3 = np.zeros(0)

with open(file_name_1, 'rb') as csvfile:
    dialect = csv.Sniffer().sniff(csvfile.read(1024))
    csvfile.seek(0)
    reader = csv.reader(csvfile, dialect)
    count = 0
    for row in reader:
        if count > 0:
          raw_trust_1 = np.append(raw_trust_1, float(row[2]))
          full_trust_1 = np.append(full_trust_1, float(row[3]))
          raw_cost_1 = np.append(raw_cost_1, float(row[4]))
        count = count + 1

with open(file_name_2, 'rb') as csvfile:
    dialect = csv.Sniffer().sniff(csvfile.read(1024))
    csvfile.seek(0)
    reader = csv.reader(csvfile, dialect)
    count = 0
    for row in reader:
        if count > 0:
          raw_trust_2 = np.append(raw_trust_2, float(row[2]))
          full_trust_2 = np.append(full_trust_2, float(row[3]))
          raw_cost_2 = np.append(raw_cost_2, float(row[4]))
        count = count + 1

with open(file_name_3, 'rb') as csvfile:
    dialect = csv.Sniffer().sniff(csvfile.read(1024))
    csvfile.seek(0)
    reader = csv.reader(csvfile, dialect)
    count = 0
    for row in reader:
        if count > 0:
          raw_trust_3 = np.append(raw_trust_3, float(row[2]))
          full_trust_3 = np.append(full_trust_3, float(row[3]))
          raw_cost_3 = np.append(raw_cost_3, float(row[4]))
        count = count + 1

mp.hold(True)
mp.plot(full_trust_1, color='Blue', label='User Trust', linewidth=4)
mp.plot(full_trust_2, color='m', label='User Trust', linewidth=4)
mp.plot(full_trust_3, color='k', label='User Trust', linewidth=4)
mp.legend()
mp.hold(False)
mp.show()