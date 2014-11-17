#!/usr/bin/env python
import numpy as np
import pylab
import yaml

stream = file('time.yaml', 'r')
TIME_LOG = yaml.load(stream)
stream.close()

time2 = TIME_LOG[1]
time4 = TIME_LOG[2]
time6 = TIME_LOG[3]
time8 = TIME_LOG[4]
time10 = TIME_LOG[5]

stream = file('config.yaml', 'r')
CONFIG_LOG = yaml.load(stream)
stream.close()
for i in range(0, len(CONFIG_LOG)):
    for j in range(0, len(CONFIG_LOG[i])):
        for k in range(0, len(CONFIG_LOG[i][j])):
            if np.isnan(CONFIG_LOG[i][j][k]):
                print "Config NaN found and replaced"
                CONFIG_LOG[i][j][k] = 0

config2 = np.asanyarray(CONFIG_LOG[1])
config4 = np.asanyarray(CONFIG_LOG[2])
config6 = np.asanyarray(CONFIG_LOG[3])
config8 = np.asanyarray(CONFIG_LOG[4])
config10 = np.asanyarray(CONFIG_LOG[5])

stream = file('command.yaml', 'r')
COMMAND_LOG = yaml.load(stream)
stream.close()
for i in range(0, len(COMMAND_LOG)):
    for j in range(0, len(COMMAND_LOG[i])):
        for k in range(0, len(COMMAND_LOG[i][j])):
            if np.isnan(COMMAND_LOG[i][j][k]):
                print "Command NaN found and replaced"
                COMMAND_LOG[i][j][k] = 0

command2 = np.asanyarray(COMMAND_LOG[1])
command4 = np.asanyarray(COMMAND_LOG[2])
command6 = np.asanyarray(COMMAND_LOG[3])
command8 = np.asanyarray(COMMAND_LOG[4])
command10 = np.asanyarray(COMMAND_LOG[5])

#roygbiv
ax1 = pylab.subplot(211)
line2, = pylab.plot(time2, config2[:,0])
line4, = pylab.plot(time4, config4[:,0])
line6, = pylab.plot(time6, config6[:,0])
line8, = pylab.plot(time8, config8[:,0])
line10, = pylab.plot(time10, config10[:,0])
pylab.axvline(x=1.0, linewidth=3, linestyle="--", color="gray")
pylab.setp(line2, linewidth=3, color='red')
pylab.setp(line4, linewidth=3, color='orange')
pylab.setp(line6, linewidth=3, color='yellow')
pylab.setp(line8, linewidth=3, color='green')
pylab.setp(line10, linewidth=3, color='blue')
pylab.legend([line2, line4, line6, line8, line10], ["Trust 0.2", "Trust 0.4", "Trust 0.6", "Trust 0.8", "Trust 1.0"])
pylab.title("Feedback Controller Input Blended with User Input at Different Levels of Trust")
pylab.ylabel("Base Joint Angle about X-axis (radians)")
ax2 = pylab.subplot(212, sharex=ax1)
line2, = pylab.plot(time2, config2[:,1])
line4, = pylab.plot(time4, config4[:,1])
line6, = pylab.plot(time6, config6[:,1])
line8, = pylab.plot(time8, config8[:,1])
line10, = pylab.plot(time10, config10[:,1])
pylab.axvline(x=1.0, linewidth=3, linestyle="--", color="gray")
pylab.setp(line2, linewidth=3, color='red')
pylab.setp(line4, linewidth=3, color='orange')
pylab.setp(line6, linewidth=3, color='yellow')
pylab.setp(line8, linewidth=3, color='green')
pylab.setp(line10, linewidth=3, color='blue')
pylab.legend([line2, line4, line6, line8, line10], ["Trust 0.2", "Trust 0.4", "Trust 0.6", "Trust 0.8", "Trust 1.0"])
pylab.xlabel("Time (sec)")
pylab.ylabel("Base Joint Angle about Y-axis (radians)")
pylab.show()
