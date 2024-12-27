import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
import numpy as np
import heapq

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

# Function to follow the computed path and send velocity commands
def follow_path(path):
    # Publisher for robot velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    for i in range(1, len(path)):
        waypoint = path[i]
        reached = False

        while not reached:
            cmd = Twist()
            
            # Current robot position
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            
            # Compute the difference in x and y
            dx = waypoint[0] - current_x
            dy = waypoint[1] - current_y
            
            # Compute the distance to the waypoint
            distance = np.sqrt(dx**2 + dy**2)
            
            # Compute the desired yaw (angle to waypoint)
            desired_yaw = np.arctan2(dx, dy)
            
            # Current robot yaw
            current_orientation = current_pose.orientation
            current_yaw = 2 * np.arctan2(current_orientation.z, current_orientation.w)
            
            # Normalize the yaw difference
            yaw_error = desired_yaw - current_yaw
            yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi  # Wrap between -pi and pi
            
            # If the robot needs to rotate
            if abs(yaw_error) > 0.1:  # Threshold to stop rotating
                cmd.angular.z = 0.5 * yaw_error  # Proportional control for angular velocity
            else:
                cmd.angular.z = 0.0  # Stop rotation when aligned
                
                # If aligned, move forward
                if distance > 0.1:  # Threshold to stop moving forward
                    cmd.linear.x = 0.2  # Move forward with constant speed
                else:
                    cmd.linear.x = 0.0  # Stop if close enough to waypoint
                    reached = True  # Mark the waypoint as reached
            
            # Publish the command
            cmd_vel_pub.publish(cmd)
            rate.sleep()


# Main function to initialize the node and subscriptions
if __name__ == "__main__":
    rospy.init_node('path_planner')

    # Subscriptions to topics
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    rospy.spin()
