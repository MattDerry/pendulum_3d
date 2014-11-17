#!/usr/bin/env python
"""
Matt Derry
Summer 2014

Convert Wii Balance Board joy input from the Linux bluetooth driver and generic ROS joy message to a joy input matching the ps3 joy axes

"""

## define all imports:
import roslib
roslib.load_manifest('pendulum_3d')
import rospy
from rospy import Publisher
from sensor_msgs.msg import Joy


####################
# GLOBAL VARIABLES #
####################

RIGHT_TOP_IDX = 0
RIGHT_BOTTOM_IDX = 1
LEFT_TOP_IDX = 2
LEFT_BOTTOM_IDX = 3

MAX_PERCENT = 0.2
DEADZONE = 0.02

board_pub = rospy.Publisher("/board_joy", Joy, queue_size=1)
board_joy = Joy()

###########################
# MISCELLANEOUS FUNCTIONS #
###########################

def joy_cb(data):
    x_balance = (data.axes[RIGHT_TOP_IDX] + data.axes[RIGHT_BOTTOM_IDX]) / (data.axes[LEFT_TOP_IDX] + data.axes[LEFT_BOTTOM_IDX])
    if x_balance < 1-DEADZONE:
        x_balance = (x_balance - 1) / MAX_PERCENT
    elif x_balance > 1+DEADZONE:
        x_balance = (x_balance - 1) / MAX_PERCENT
    else:
        x_balance = 0

    board_joy.axes[0] = x_balance

    y_balance = (data.axes[LEFT_BOTTOM_IDX] + data.axes[RIGHT_BOTTOM_IDX]) / (data.axes[LEFT_TOP_IDX] + data.axes[RIGHT_TOP_IDX])
    if y_balance < 1-DEADZONE:
        y_balance = (y_balance - 1) / MAX_PERCENT
    elif y_balance > 1+DEADZONE:
        y_balance = (y_balance - 1) / MAX_PERCENT
    else:
        y_balance = 0

    board_joy.axes[1] = y_balance
    board_joy.buttons[14] = data.buttons[0]
    board_joy.header.stamp = rospy.Time.now()
    board_pub.publish(board_joy)

def main():
    """
    Run the main loop, by subscribing to joy, and then
    calling ros.spin
    """
    rospy.init_node('balance_board_driver')  # , log_level=rospy.INFO)

    board_joy.header.frame_id = ''
    board_joy.header.stamp = rospy.Time.now()
    board_joy.axes.append(0.0)
    board_joy.axes.append(0.0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)
    board_joy.buttons.append(0)

    # check what the value of the links param is
    try:
        sub = rospy.Subscriber("/joy", Joy, joy_cb)

    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__ == '__main__':
    main()
