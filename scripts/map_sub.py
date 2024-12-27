import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import numpy as np

def map_callback(msg):
    global map_data
    map_data = msg
    #print(map_data)

def pose_callback(msg):
    global current_pose
    current_pose = msg.pose
    print(current_pose)

def goal_callback(msg):
    global goal_pose
    goal_pose = msg.pose
    #print(goal_pose)
    # Call your path planning function here

def path_planning(map_data, current_pose, goal_pose):
    # Implement A* or Dijkstra's algorithm
    pass

if __name__ == "__main__":
    rospy.init_node('path_planner')

    # Subscriptions
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    rospy.spin()
