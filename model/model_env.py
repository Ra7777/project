import math
import os
import sys
import random
import subprocess
import time
from os import path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

GOAL_REACHED_DIST = 0.2
COLLISION_DIST_FRONT = 0.3
COLLISION_DIST_SIDE = 0.45
COLLISION_DIST_BACK = 0.7
TIME_DELTA = 0.1
lower_bound = 4
upper_bound = 8
steer_bound = 4.71
removed = np.array([0,1,2,3,4,5,6,65,66,67,68,69,70,110,111,112,113,114,173,174,175,176,177,178,179,180,181,182,183,184,354,355,356,357,358,359])
collision_angles = np.array([0.58, 0.58, 0.58, 0.58, 0.59, 0.59, 0.59, 0.6, 0.6, 0.61, 0.61, 0.62, 0.62, 0.63, 0.63, 0.63, 0.63, 0.64, 0.64, 0.64, 0.65, 0.65, 0.65, 0.66, 0.66, 0.67, 0.67, 0.67, 0.68, 0.68, 0.68, 0.69, 0.69, 0.69, 0.7, 0.7, 0.7, 0.71, 0.71, 0.71, 0.72, 0.72, 0.72, 0.72, 0.73, 0.73, 0.73, 0.74, 0.74, 0.74, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.76, 0.76, 0.77, 0.77, 0.77, 0.77, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.79, 0.79, 0.79, 0.79, 0.79, 0.79, 0.79, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.78, 0.77, 0.77, 0.77, 0.77, 0.76, 0.76, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.74, 0.74, 0.74, 0.73, 0.73, 0.73, 0.72, 0.72, 0.72, 0.72, 0.72, 0.71, 0.71, 0.71, 0.7, 0.7, 0.69, 0.69, 0.69, 0.68, 0.68, 0.68, 0.68, 0.67, 0.67, 0.66, 0.66, 0.66, 0.65, 0.65, 0.64, 0.64, 0.64, 0.63, 0.63, 0.63, 0.63, 0.62, 0.62, 0.61, 0.61, 0.61, 0.61, 0.61, 0.59, 0.59, 0.59, 0.58, 0.58, 0.58, 0.55, 0.55, 0.55, 0.54, 0.54, 0.53, 0.53, 0.53, 0.52, 0.52, 0.52, 0.51, 0.51, 0.51, 0.51, 0.5, 0.5, 0.5, 0.5, 0.49, 0.49, 0.49, 0.49, 0.48, 0.48, 0.48, 0.47, 0.47, 0.47, 0.47, 0.46, 0.46, 0.46, 0.46, 0.46, 0.46, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.44, 0.44, 0.44, 0.44, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.41, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.42, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.43, 0.44, 0.44, 0.44, 0.44, 0.44, 0.45, 0.45, 0.45, 0.45, 0.45, 0.45, 0.46, 0.46, 0.46, 0.46, 0.46, 0.47, 0.47, 0.47, 0.47, 0.48, 0.48, 0.48, 0.48, 0.49, 0.49, 0.49, 0.5, 0.5, 0.5, 0.5, 0.5, 0.51, 0.51, 0.51, 0.51, 0.52, 0.52, 0.52, 0.53, 0.53, 0.54, 0.54, 0.54, 0.55, 0.55])


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True

    #fire hydrant
    if -7.3 > x > -10.5 and 10.5 > y > 7.3:
        goal_ok = False

    #plus-wall vertical
    if -2.0 > x > -7.0 and 8.0 > y > -1.0:
        goal_ok = False

    #plus-wall horizontal
    if -0.0 > x > -8.5 and 5.8 > y > 2.0:
        goal_ok = False

    #brick-wall triangle
    if -0.7 > x > -8.7 and -1.4 > y > -8.5:
        goal_ok = False

    #???
    #if -1.3 > x > -3.7 and -0.8 > y > -2.7:
    #    goal_ok = False

    #L-wall horizontal
    if 8.5 > x > 1.0 and -3.0 > y > -7.0:
        goal_ok = False

    #L-wall vertical
    if 9.0 > x > 4.0 and 2.0 > y > -7.0:
        goal_ok = False

    #closet
    if 11.5 > x > 7.0 and -5.5 > y > -9.8:
        goal_ok = False

    #brick-wall rectangle
    if 8.9 > x > 1.7 and 8.5 > y > 2.5:
        goal_ok = False

    #table
    if -6.0 > x > -10.6 and 1.5 > y > -3.5:
        goal_ok = False

    #outer area
    if x > 9.0 or x < -9.0 or y > 9.0 or y < -9.0:
        goal_ok = False
    #if 1.0 > x > -1.0 and 1.0 > y > -1.0:
    #	goal_ok = False

    return goal_ok


class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.odom_buf_x = 0
        self.odom_buf_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.x1 = 0
        self.y1 = 0

        self.ang = 0

        self.upper = 10.0
        self.lower = -10.0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.velodyne_data_col = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = SetEntityState.Request()
        self.set_self_state.state.name = 'bot'
        self.set_self_state.state.pose.position.x = 0.0
        self.set_self_state.state.pose.position.y = 0.0
        self.set_self_state.state.pose.position.z = 0.0
        self.set_self_state.state.pose.orientation.x = 0.0
        self.set_self_state.state.pose.orientation.y = 0.0
        self.set_self_state.state.pose.orientation.z = 0.0
        self.set_self_state.state.pose.orientation.w = 1.0
        #self.set_self_state.state.reference_frame = "world"

        #self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        #for m in range(self.environment_dim - 1):
        #    self.gaps.append(
        #        [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
        #    )
        #self.gaps[-1][-1] += 0.03

        #port = "11311"
        #subprocess.Popen(["roscore", "-p", port])

        #print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        self.rclpy = rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('gym')
        self.node.get_logger().info('Created node')

        self.last_distance = 0
        self.distance = 0
        #if launchfile.startswith("/"):
        #    fullpath = launchfile
        #else:
        #    fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        #if not path.exists(fullpath):
        #    raise IOError("File " + fullpath + " does not exist")

        #subprocess.Popen(["roslaunch", "-p", port, fullpath])
        #print("Gazebo launched!")

        # Set up the ROS publishers and subscribers
        #self.vel_pub = rospy.Publisher("/r1/cmd_vel", Twist, queue_size=1)
        #qos_profile = QoSProfile(depth=10)
        self.vel_pub = self.node.create_publisher(msg_type=Twist, topic='/tricycle_controller/cmd_vel', qos_profile=10)
        #self.set_state = rospy.Publisher(
        #    "gazebo/set_model_state", ModelState, queue_size=10
        #)
        #self.set_state = self.node.create_publisher(msg_type=ModelState, topic='gazebo/set_model_state', qos_profile=10)
        self.set_state = self.node.create_client(SetEntityState, '/gazebo/set_entity_state')
        #self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.unpause = self.node.create_client(Empty, '/unpause_physics')
        #self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.pause = self.node.create_client(Empty, '/pause_physics')
        self.reset_odometry = self.node.create_client(Empty, '/tricycle_controller/reset_odometry')
        #self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.reset_proxy = self.node.create_client(Empty, '/reset_world')
        #self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher = self.node.create_publisher(msg_type=MarkerArray, topic='goal_point', qos_profile=3)
        #self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher2 = self.node.create_publisher(msg_type=MarkerArray, topic='linear_velocity', qos_profile=1)
        #self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.publisher3 = self.node.create_publisher(msg_type=MarkerArray, topic='angular_velocity', qos_profile=1)

        #self.velodyne = rospy.Subscriber(
        #    "/scan", PointCloud2, self.velodyne_callback, queue_size=1
        #)
        #self.odom = rospy.Subscriber(
        #    "/r1/odom", Odometry, self.odom_callback, queue_size=1
        #)
        self.odom = self.node.create_subscription(
            msg_type=Odometry,
            topic='/tricycle_controller/odom',
            callback=self.odom_callback,
            qos_profile=10)
        self.velodyne = self.node.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.velodyne_callback,
            qos_profile=30)


    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, data):
        #data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        #print(data.ranges)
        laserlist = np.array(data.ranges)
        laserlist = np.delete(laserlist, removed)
        #data.ranges[data.ranges == float('inf')] = 12.0
        laserlist[laserlist == np.inf] = 12.0
        #self.velodyne_data = [min(data.ranges[i:i+8]) for i in range(0, len(data.ranges), 9)]
        #self.velodyne_data = [min(laserlist[i:i+8]) for i in range(0, 41, 7)] + [min(laserlist[42:50])] + [min(laserlist[50:58])] \
        #                     + [min(laserlist[i:i+9]) for i in range(58, 89, 8)] + [min(laserlist[90:97])] \
        #                     + [min(laserlist[i:i+8]) for i in range(97, 138, 7)] + [min(laserlist[139:147])] + [min(laserlist[147:155])] \
        #                     + [min(laserlist[i:i+9]) for i in range(155, 315, 8)] + [min(laserlist[315:])]
        self.velodyne_data = [min(laserlist[0:58])] \
                             + [min(laserlist[58:97])] \
                             + [min(laserlist[97:155])] \
                             + [min(laserlist[i:i+9]) for i in range(155, 315, 8)] + [min(laserlist[315:])]
        self.velodyne_data_col = [laserlist[i] for i in range(0, len(laserlist))]

    def odom_callback(self, od_data):
        self.last_odom = od_data

    # Perform an action and read a new state
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        #print("Action entered into step:")
        #print(action)
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        #rclpy.wait_for_service('/gazebo/unpause_physics')

        #try:
        #--------------------------------
        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.log('reset service not available, waiting again...')
        req = Empty.Request()
        future = self.unpause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        #--------------------------------
        #resp = self.unpause.call()
        #self.rclpy.spin_once(self.node, resp)
        #self.log('spin')
        #except (rclpy.ServiceException) as e:
        #    print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        #rospy.wait_for_service("/gazebo/pause_physics")
        #try:
        #    pass
        #    self.pause()
        #except (rospy.ServiceException) as e:
        #    print("/gazebo/pause_physics service call failed")

        #while not self.pause.wait_for_service(timeout_sec=1.0):
        #    self.node.get_logger().info('pause service not available, waiting again...')
        #try:
        #    pass

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.log('reset service not available, waiting again...')
        req = Empty.Request()
        future = self.pause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        #resp = self.pause.call()
        #self.rclpy.spin_once(self.node, resp)
        #self.log('spin')
        #except (rclpy.ServiceException) as e:
        #    print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(self.velodyne_data_col)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        #rclpy.spin_once(self.node)
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        self.last_distance = self.distance
        '''self.distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )'''
        self.distance = np.linalg.norm(
            [self.odom_x + math.cos(angle) * 0.56 - self.goal_x, self.odom_y + math.sin(angle) * 0.56 - self.goal_y]
        )


        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if self.distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [self.distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser, self.distance, self.last_distance)
        return state, reward, done, target

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        while not self.reset_proxy.wait_for_service(timeout_sec=1.0):
            self.log('reset service not available, waiting again...')
        #rospy.wait_for_service("/gazebo/reset_world")
        #try:
        req = Empty.Request()
        #resp = self.reset_proxy.call(req)
        future = self.reset_proxy.call_async(req) #.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        while not self.reset_odometry.wait_for_service(timeout_sec=1.0):
            self.log('odom reset service not available, waiting again...')
        req = Empty.Request()
        future = self.reset_odometry.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        #rclpy.spin_once(self.node)

        #except rospy.ServiceException as e:
        #    print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        self.ang = angle
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-9.0, 9.0)
            y = np.random.uniform(-9.0, 9.0)
            position_ok = check_pos(x, y)
        object_state.state.pose.position.x = x
        object_state.state.pose.position.y = y
        object_state.state.pose.position.z = 0.
        object_state.state.pose.orientation.x = quaternion.x
        object_state.state.pose.orientation.y = quaternion.y
        object_state.state.pose.orientation.z = quaternion.z
        object_state.state.pose.orientation.w = quaternion.w
        #self.set_state.publish(object_state)

        while not self.set_state.wait_for_service(timeout_sec=1.0):
            self.log('set_state service not available, waiting again...')
        #req = SetEntityState.Request(object_state)
        future = self.set_state.call_async(object_state)
        rclpy.spin_until_future_complete(self.node, future)

        self.odom_x = 0.0
        self.odom_buf_x = object_state.state.pose.position.x
        self.odom_y = 0.0
        self.odom_buf_y = object_state.state.pose.position.y

        # set a random goal in empty space in environment
        self.change_goal()
        '''while not self.pause.wait_for_service(timeout_sec=1.0):
            self.log('reset service not available, waiting again...')
        req = Empty.Request()
        future = self.pause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        time.sleep(1000000000)'''
        # randomly scatter boxes in the environment
        self.random_box()
        self.publish_markers([0.0, 0.0])

        while not self.unpause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('unpause service not available, waiting again...')
        future = self.unpause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)


        time.sleep(TIME_DELTA)

        while not self.pause.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('pause service not available, waiting again...')
        future = self.pause.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        #rclpy.spin_once(self.node)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        '''self.distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )'''

        self.distance = np.linalg.norm(
            [self.odom_x + math.cos(angle) * 0.56 - self.goal_x, self.odom_y + math.sin(angle) * 0.56 - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [self.distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            r1 = random.uniform(self.upper, self.lower)
            r2 = random.uniform(self.upper, self.lower)
            #self.goal_x = self.odom_x + r1
            #self.goal_y = self.odom_y + r2
            self.x1 = r1 + self.odom_buf_x
            self.y1 = r2 + self.odom_buf_y
            self.goal_x = r1*math.cos(self.ang) + r2*math.sin(self.ang)
            self.goal_y = r2*math.cos(self.ang) - r1*math.sin(self.ang)
            #goal_ok = check_pos(self.goal_x + self.odom_buf_x, self.goal_y + self.odom_buf_y)
            goal_ok = check_pos(self.x1, self.y1)
        '''print(f'ang: {self.ang}, sin: {math.sin(self.ang)}, cos: {math.cos(self.ang)}')
        print(f'odom_buf_x: {self.odom_buf_x}, odom_buf_y: {self.odom_buf_y}')
        print(f'r1: {r1}, r2: {r2}')
        print(f'x: {self.x1}, y: {self.y1}')
        print(f'goal_x: {self.goal_x}, goal_y: {self.goal_y}')'''

    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training
        # environment
        for i in range(4):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-12, 12)
                y = np.random.uniform(-12, 12)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_buf_x, y - self.odom_buf_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 3.0 or distance_to_goal < 3.0:
                    box_ok = False
            box_state = SetEntityState.Request()
            box_state.state.name = name
            box_state.state.pose.position.x = x
            box_state.state.pose.position.y = y
            box_state.state.pose.position.z = 0.0
            box_state.state.pose.orientation.x = 0.0
            box_state.state.pose.orientation.y = 0.0
            box_state.state.pose.orientation.z = 0.0
            box_state.state.pose.orientation.w = 1.0
            #self.set_state.publish(box_state)
            while not self.set_state.wait_for_service(timeout_sec=1.0):
                self.log('reset service not available, waiting again...')
            #req = SetEntityState.Request(box_state)
            future = self.set_state.call_async(box_state)
            rclpy.spin_until_future_complete(self.node, future)

    def publish_markers(self, action):
        #print(action)
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        #print(type(action[1]))
        #print(type(abs(action[1])))
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5.0
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0.0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        #min_laser_front = min(laser_data[155:])
        #min_laser_side = min(laser_data[:29]+laser_data[126:155])
        #min_laser_back = min(laser_data[29:126])
        if min(np.subtract(laser_data, collision_angles)) < 0:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser, distance, last_distance):
        if target:
            return 100.0
        elif collision:
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            #print(action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2)
            #print(abs(action[1]) / 2 - action[0] / 2 - r3(min_laser) / 2)
            #return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2
            #print("old: %f, new: %f" % (action[0] / 2 - abs(action[1]) / 2, (last_distance - distance) / 10))
            #print(action[0] / 2 - abs(action[1]) / 2 + (last_distance - distance))
            return action[0] / 2 - abs(action[1]) / 2 #- r3(min_laser) / 2)

    def log(self, msg):
        self.node.get_logger().info(msg)

