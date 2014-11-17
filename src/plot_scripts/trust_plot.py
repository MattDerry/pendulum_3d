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

user_number = 4
file_name = '/home/mderry/cps_data/user_' + str(user_number) + '/user_' + str(user_number) + '_trust_log.csv'

raw_trust = np.zeros(0)
full_trust = np.zeros(0)

with open(file_name, 'rb') as csvfile:
    dialect = csv.Sniffer().sniff(csvfile.read(1024))
    csvfile.seek(0)
    reader = csv.reader(csvfile, dialect)
    count = 0
    for row in reader:
        if count > 0:
          raw_trust = np.append(raw_trust, float(row[2]))
          full_trust = np.append(full_trust, float(row[3]))
        count = count + 1

mp.hold(True)
mp.plot(raw_trust, color='Red', label='Trial Trust', linewidth=2)
mp.plot(full_trust, color='Blue', label='User Trust', linewidth=4)
mp.legend()
mp.hold(False)
mp.show()