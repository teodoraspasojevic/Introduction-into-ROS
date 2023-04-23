#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

# global detection counter for temperature difference exceeding 15°C at border crossings

global counter

# function that prints a message when a temperature difference exceeding 15°C is detected at border crossings
# and counts the number of occurrences of this situation.

def callback(data):
    global counter
    counter = counter + 1
    rospy.loginfo('%i. temperature difference between the maximum and minimum temperature that is greater than 15.', counter)


# function that initializes an action node and creates a subscriber.

def listener():
    rospy.init_node('action', anonymous=False)
    rospy.Subscriber('top_action', Bool, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        global counter
        counter = 0
        listener()
    except rospy.ROSInterruptException:
        pass