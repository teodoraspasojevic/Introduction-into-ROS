#!/usr/bin/env python3

import rospy
import pandas as pd
from std_msgs.msg import Float32, Float32MultiArray


# function that initializes a measurement node and publishes data to the processing node through a topic

def talker():

    # we initialize of the measurement node and create a Publisher that sends temperature data to processing

    rospy.init_node('measurements', anonymous=False)
    pub_measure = rospy.Publisher('top_measurements', Float32MultiArray, queue_size=10)
    r = rospy.Rate(10)

    # we use the pandas library to load data from a .csv file into a data frame, and convert df to a numpy array

    filename = 'weather_data_nyc_centralpark_2016.csv'
    df = pd.read_csv(filename, index_col=None)
    df = df.drop(['date', 'precipitation', 'snow fall', 'snow depth'], axis=1)
    data = df.to_numpy(dtype=Float32)

    # we pause communication briefly to synchronize the Publisher and Subscriber in the processing node

    rospy.sleep(1)

    # we send temperatures from one row of the table to the processing node via the "top_measurements" topic

    for i in range(len(data)):

        # if rospy is shut down, we stop the program

        if rospy.is_shutdown():
            break

        my_data = Float32MultiArray()
        my_data.data = data[i]
        pub_measure.publish(my_data)
        rospy.loginfo('[%.1f %.1f %.1f]°F', data[i][0], data[i][1], data[i][2])
        r.sleep()

    # we start a while loop so as not to interrupt communication between nodes
    # and we do not put the for loop that sends data to the topic inside the while loop
    # so as not to iterate through the table multiple times

    while not rospy.is_shutdown():
        r.sleep()


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass