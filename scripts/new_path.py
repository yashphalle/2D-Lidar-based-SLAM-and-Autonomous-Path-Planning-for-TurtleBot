import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import numpy as np
import heapq
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from waypoint import turtlebot_move

# Global variables
map_data = None
current_pose = None
goal_pose = None

# Callback function for map data
def map_callback(msg):
    global map_data
    map_data = msg

# Callback function for robot's current pose
def pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose
    
    #print("Current",current_pose.position)

# Callback function for goal pose
def goal_callback(msg):
    global goal_pose
    goal_pose = msg.pose
    # Call path planning after receiving a goal
    if current_pose and map_data:
        path = path_planning(map_data, current_pose, goal_pose)
        follow_path(path)

# A* Path planning function
def path_planning(map_data, current_pose, goal_pose):
    # Extracting map information
    print("path palnning called")
    resolution = map_data.info.resolution  # The size of each cell in meters
    width = map_data.info.width
    height = map_data.info.height
    origin_x = map_data.info.origin.position.x
    origin_y = map_data.info.origin.position.y
    #print("origin",origin_x,origin_y)
    map_data_array = np.array(map_data.data).reshape((height, width))
    #print(map_data_array.size)

    # A* parameters
    start = (int((current_pose.position.x - origin_x)/resolution ), 
             int((current_pose.position.y - origin_y)/resolution ))
    goal = (int((goal_pose.position.x - origin_x)/resolution ), 
            int((goal_pose.position.y - origin_y)/resolution ))
    
    print("start",str(start),"goal",str(goal))

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(start, goal):
        # Open list for A* (priority queue)
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        # Cost from start to a point and cost to go from that point to goal
        g_cost = {start: 0}
        f_cost = {start: heuristic(start, goal)}
        came_from = {}
        
        # Directions (8 possible moves)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]
        
        while open_list:
            current = heapq.heappop(open_list)[1]
            #print(current)
            
            # If we reached the goal, reconstruct the path
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Check all possible neighbors
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                if 0 <= neighbor[0] < height and 0 <= neighbor[1] < width:  # Check bounds
                    if map_data_array[neighbor[0], neighbor[1]] == 100:  # Check for obstacles
                        continue
                    
                    tentative_g_cost = g_cost[current] + 1
                    if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                        came_from[neighbor] = current
                        g_cost[neighbor] = tentative_g_cost
                        f_cost[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_cost[neighbor], neighbor))
        
        return []  # No path found
    
    # Call A* to get the path
    path = a_star(start, goal)
    #print(path)
    
    # Convert path from grid coordinates to real-world coordinates
    path_in_real_coords = []
    for (x, y) in path:
        real_x = x * resolution + origin_x
        real_y = y * resolution + origin_y
        path_in_real_coords.append((real_x, real_y))
    #print("realpath==========",path_in_real_coords)
    return path_in_real_coords

def move_to_goal(x, y, frame="map"):
    """
    Sends a goal to the move_base action server.
    Args:
        x: Target x-coordinate.
        y: Target y-coordinate.
        frame: Reference frame, default is "map".
    Returns:
        True if the goal was successfully reached, False otherwise.
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # No rotation

    rospy.loginfo(f"Sending goal: x={x}, y={y}")
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return False
    else:
        return client.get_result()

def follow_path(path):
    """
    Navigates through a list of waypoints.
    Args:
        waypoints: A list of dictionaries with 'x' and 'y' coordinates
    """

    turtlebot = turtlebot_move()  # Create an instance of the class
    # turtlebot.move_to_point(0.2, 0.2)
    # print("actually worked")

    # cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # rate = rospy.Rate(10)  # 10 Hz
    
    for i in range(1, len(path)):
        waypoint = path[i]
        reached = False
        rospy.loginfo(f"Navigating to waypoint using waypoint function {i + 1}/{len(path)}: {waypoint}")
        result = turtlebot.move_to_point(waypoint[0], waypoint[1])
        print(current_pose.position)
        


# Main function to initialize the node and subscriptions
if __name__ == "__main__":
    rospy.init_node('path_planner')

    # Subscriptions to topics
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    rospy.spin()
