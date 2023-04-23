#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Bool, Float32, Float32MultiArray
from dz1.srv import add_new_values, add_new_valuesResponse
from csv import writer

# Function called by the Subscriber when it receives new data via the "top_measurements" topic.

def callback(data):
    # We create a Publisher that will send data to the display node via the "top_display" topic.
    # We also create a Publisher that will send data to the action node via the "top_action" topic.

    pub_display = rospy.Publisher('top_display', Float32, queue_size = 10)
    pub_action = rospy.Publisher('top_action', Bool, queue_size = 10)

    # We print the temperatures we have loaded in °F to the terminal.

    rospy.loginfo('[%.1f %.1f %.1f]°F', data.data[0], data.data[1], data.data[2])

    # We convert the temperatures to °C.

    max_cel = (data.data[0]-32)/1.8
    min_cel = (data.data[1]-32)/1.8
    avg_cel = (data.data[2]-32)/1.8
    temperatures = np.array([max_cel, min_cel, avg_cel], dtype=Float32)

    # We print the temperatures we have calculated in °C to the terminal.

    rospy.loginfo('[%.1f %.1f %.1f]°C', temperatures[0], temperatures[1], temperatures[2])
    
    # We send the average temperature to the node for display, via the topic top_display.

    rospy.sleep(1)
    pub_display.publish(avg_cel)

    # We check if there is a difference in the boundary temperatures greater than 15°C,
    # and if there is, we send a signal to the node for action via the topic top_action.

    if (abs(max_cel - min_cel) > 15):
        flag = 1
        rospy.sleep(1)
        pub_action.publish(flag)

    # Temperatures converted to °C are written to a new .csv table.

    with open('celzius_weather_data_nyc_centralpark_2016.csv', mode='a', newline='') as file:
        write = writer(file)
        write.writerow([round(max_cel,2), round(min_cel,2), round(avg_cel,2)])


# Function called by the Service when a new data is received from the terminal.

def response_callback(req):
    # We create a Publisher that sends data to the display node via the "top_display" topic.
    # We create a Publisher that sends data to the action node via the "top_action" topic.

    pub_display = rospy.Publisher('top_display', Float32, queue_size = 10)
    pub_action = rospy.Publisher('top_action', Bool, queue_size = 10)
    
    # We print the temperatures we have loaded, in °C, to the terminal.

    rospy.loginfo('[%.1f %.1f %.1f]°C', req.new_max, req.new_min, req.new_avg)

    # We convert the temperatures to °F.

    max_far = req.new_max*1.8 + 32
    min_far = req.new_min*1.8 + 32
    avg_far = req.new_avg*1.8 + 32
    temperatures = np.array([max_far, min_far, avg_far], dtype=Float32)

    # We print the temperatures we have loaded in °F to the terminal.

    rospy.loginfo('[%.1f %.1f %.1f]°F', temperatures[0], temperatures[1], temperatures[2])

    # We send the average temperature to the display node via the "top_display" topic.

    pub_display.publish(req.new_avg)

    # We check if there is a difference in the boundary temperatures greater than 15°C, and if so
    # we send a signal to the action node via the "top_action" topic.

    if (abs(req.new_max - req.new_min) > 15):
        flag = 1
        pub_action.publish(flag)
    
    # We write the temperatures converted to °C into a new .csv table.
    
    with open('celzius_weather_data_nyc_centralpark_2016.csv', mode='a', newline='') as file:
        write = writer(file)
        write.writerow([round(req.new_max,2), round(req.new_min,2), round(req.new_avg,2)])
    return add_new_valuesResponse(True)  


# This function creates a new table, initializes the processing node, creates a Subscriber and a Service.

def listener():

    # We create a new .csv table in which we will store all the read temperatures, as well as those we send via the service.

    file = open('celzius_weather_data_nyc_centralpark_2016.csv', mode='w', newline='')
    write = writer(file)
    write.writerow(["Max temp", "Min temp", "Avg temp"])
    file.close()

    # We initialize the processing node, create a Subscriber, and a Service.

    rospy.init_node('processing', anonymous = False)
    rospy.Subscriber('top_measurements', Float32MultiArray, callback)
    srv = rospy.Service('add_new_values', add_new_values, response_callback)
    rospy.loginfo("Service is ready!")
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass