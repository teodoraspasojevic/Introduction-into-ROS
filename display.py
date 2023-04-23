#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32


# function that prints the received average temperature from the topic "top_display" to the terminal.

def callback(data):
    rospy.loginfo('Average daily temperature is %.1f°C.', data.data)


# function that initializes a display node and creates a subscriber.

def listener():
    rospy.init_node('display', anonymous=False)
    rospy.Subscriber('top_display', Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass