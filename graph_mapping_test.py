#!/usr/bin/env python
import networkx as nx
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry

x_pose = 0
y_pose = 0

def callback(odom_msg):
    x_pose = odom_msg.pose.pose.position.x
    y_pose = odom_msg.pose.pose.position.y

if __name__ == '__main__':
    msg = """
Node indexing starts at 0, beginning with the current position of the robot!
n: add new node at the current position
e: add new edge between two nodes
p: print current map
s: save the current map to a file
q: quit the program
"""
    graph_map = nx.Graph()
    num_nodes = 0
    num_saves = 1
    run = True
    rospy.init_node('graph_mapping', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    x_temp = x_pose
    y_temp = y_pose
    graph_map.add_node(num_nodes, x_pose=x_temp, y_pose=y_temp)
    print(msg)
    while(run):
        key = raw_input('Please input a command:\n')
        if key == 'n':
	    num_nodes+=1
            x_temp = x_pose
            y_temp = y_pose
            graph_map.add_node(num_nodes, x_pose=x_temp, y_pose=y_temp)
            print('Number of nodes: ' + str(graph_map.number_of_nodes()))
        elif key == 'e':
            node1 = raw_input('Please input the first node:\n')
            node2 = raw_input('Please input the second node:\n')
            #map_list = list(graph_map.nodes(data=True))
            #if (node1 in map_list) & (node2 in map_list):
            graph_map.add_edge(int(node1),int(node2))
            #else:
            #print('Please enter valid nodes.\n')
        elif key == 'p':
            print(graph_map.nodes(data=True))
            print(graph_map.edges(data=True))
        elif key == 's':
            nx.write_graphml(graph_map, "/home/agilex/catkin_ws/src/limo_ros/limo_bringup/maps/map"+str(num_saves)+".graphml", prettyprint=True)
        elif key == 'q':
            rospy.signal_shutdown('End of mapping.')
            run = False
        else: 
            print('Please enter a valid command.\n')


