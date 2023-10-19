#!/usr/bin/env python
import networkx as nx
import matplotlib.pyplot as plt
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import rosbag

x_pose = 0
y_pose = 0
bridge = CvBridge()
odom_bag = rosbag.Bag('/home/agilex/topological_mapping/bag_files/odometry.bag','w')

def odom_callback(odom_msg):
    global x_pose
    global y_pose
    x_pose = odom_msg.pose.pose.position.x
    y_pose = odom_msg.pose.pose.position.y
    if (odom_msg.twist.twist.linear.x != 0) or (odom_msg.twist.twist.linear.y != 0):
      odom_bag.write('odom_msg',odom_msg)

def image_callback(img_msg):
    global take_picture
    global num_nodes
    try:
        color_img = bridge.imgmsg_to_cv2(img_msg,desired_encoding='passthrough')
        if take_picture == True:
            cv2.imwrite("/home/agilex/topological_mapping/images/image_" + str(num_nodes) + ".png", color_img)
            take_picture = False
    except CvBridgeError as e:
        print(e)

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
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("camera/color/image_raw", Image, image_callback)
    x_temp = x_pose
    y_temp = y_pose
    graph_map.add_node(num_nodes, x_pose=x_temp, y_pose=y_temp, color_img="/home/agilex/topological_mapping/images/image_" + str(num_nodes) + ".png")
    take_picture = True
    print(msg)
    while(run):
        key = raw_input('Please input a command:\n')
        if key == 'n':
	    num_nodes+=1
            x_temp = x_pose
            y_temp = y_pose
            graph_map.add_node(num_nodes, x_pose=x_temp, y_pose=y_temp, color_img="/home/agilex/topological_mapping/images/image_" + str(num_nodes) + ".png")
            take_picture = True
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
            nx.write_graphml(graph_map, "/home/agilex/topological_mapping/maps/map"+str(num_saves)+".graphml", prettyprint=True)
            odom_bag.close()
        elif key == 'q':
            rospy.signal_shutdown('End of mapping.')
            run = False
        else:
            print('Please enter a valid command.\n')


