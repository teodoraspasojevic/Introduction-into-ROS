# Introduction-into-ROS

Project that introduces us into how we can use ROS. We  read data from .csv table, containing weather parameters, and then process, display and store them.

**Project description**

We create separate node for every action that is needed to be implemented: loading data in measurements node, processing data in processing node, displayment of data in display node and action node for signalization of specific occurrences in data. All nodes communicate through topics, and structure of topics and nodes used in the project is displayed in graph.png. We added an option of adding new data, and this functionality is realized using service add_new_values.

**Instructions for running the scripts using ROS**

1. Run terminal on your Linux PC, and run command roscore.
2. Use cd command to navigate to src directory of the project.
3. To run Python scripts use command rosrun, e.g. rosrune dz1 action.py. For every script open new terminal window. Run scripts in the following order:
action.py, display.py, processing.py, measurements.py.
4. To add new data (in this project new temperatures) using service, run command rosservice, e.g. rosservice call /add_new_values "{new_max: 150.0, new_min: 20.0, new_avg: 45.0}".
5. To show graph of the average temperatures run command rqt_plot, and chose to display data from topic top_display.
6. To display graph that shows the structure of the topics and nodes used in project, run rqt_graph.
