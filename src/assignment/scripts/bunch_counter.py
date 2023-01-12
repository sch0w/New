#!/usr/bin/env python
# Reference on bug2 algorithm: https://github.com/bipul93/ros-bug2/blob/master/scripts/bot.py

import rospy, sys
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import enum
import math
import numpy

from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from cv2 import namedWindow, cvtColor, imshow, inRange

from subprocess import Popen

# ------------------------------ Initialising and defining varables ------------------------------------

# Defining Bot States
# Robot performs the following states while heading to the desrired location. 
# The combination may vary depending on the map's size and obsticles
class BotState(enum.Enum):
    LOOK_TOWARDS = 0  # rotates bot towards goal
    GOAL_SEEK = 1  # follow line
    WALL_FOLLOW = 2  # Go around the wall / avoid obstacles
    ROTATE_TO_VINES = 3 # Rotate towards vines to caputure camera image
    TAKE_IMAGE = 4 # Capture images along the vines
    #SECOND_BEACON = 5 # New beacon loaction for further images

# Initialised values
yaw = 0
yaw_threshold = math.pi / 90 #Value at 90 gives 2 degrees tolerence. 4 degree tolerence will miss homing spots
goal_distance_threshold = 0.5
currentBotState = BotState.LOOK_TOWARDS

# base scan laser range values
maxRange = 3
minRange = 0

bot_pose = None
init_bot_pose = []
beacon_pose = None
bot_motion = None  # cmd_vel publisher
homing_signal = None  # subscriber
init_config_complete = False
wall_hit_point = None
beacon_found = False
taken_image = False
#second_beacon = False
twist = Twist()
distance_moved = 0

front_obs_distance = None
left_obs_distance = None

wall_following = False

# Rotation vairables
# Target for rotation
target = -90 # Target angle is 90 degree (To directly face vines)
#Smoother (kp) prevents overshooting by slowing down rotation the closer it is from the target
kp=0.5 # Slows angle of rotationthe closer it is from the desried angle 

# ---------------------------------------- Navigation --------------------------------------

# Looks towards homing beacon position
def look_towards(des_pos):
    global yaw, yaw_threshold, bot_motion, currentBotState, twist
    quaternion = (
        des_pos.orientation.x,
        des_pos.orientation.y,
        des_pos.orientation.z,
        des_pos.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]  # bot's yaw
    beacon_yaw = math.atan2(beacon_pose.position.y - des_pos.position.y, beacon_pose.position.x - des_pos.position.x)
    yaw_diff = normalize(beacon_yaw - yaw)

    # math.fabs = returns absolute value of a number as a float
    if math.fabs(yaw_diff) > yaw_threshold:
        print("Rads to get to beacon", math.fabs(yaw_diff))
        twist.angular.z = -0.2 # clockwise rotation if yaw_diff > 0 else counter-clockwise rotation

    if math.fabs(yaw_diff) <= yaw_threshold:
        twist.angular.z = 0
        currentBotState = BotState.GOAL_SEEK
    bot_motion.publish(twist)

# Seeks for homing_beacon and if obstruction is faced envoke BUG2's wall_follow() function
def goal_seek():
    print("goal seek initilised")
    global zone_F, zone_FL, zone_FR, currentBotState, bot_pose, wall_hit_point, front_obs_distance, left_obs_distance

    # Avoiding obsticles
    obstacle_in_front = numpy.any((zone_F < 1.5)) or numpy.any((zone_FR < 1.5)) or numpy.any((zone_FL < 1.5)) 
    # distance of 1.5 is the lowest it should get to move safely around the wall with a 2m offset
    # once we are in a line distance position. 1m is the lowest and with trials Thorvald's side corners got caught on the hedge when the becaon is positioned very close to it
    # Or find the minimum value in this zone. or maybe numpy.any would be faster
    print("obsticle in front?", obstacle_in_front)
    if obstacle_in_front:
        twist.linear.x = 0
        wall_hit_point = bot_pose.position
        currentBotState = BotState.WALL_FOLLOW
    else:
        twist.angular.z = 0
        twist.linear.x = 1
    bot_motion.publish(twist)

# Comment:
# Avoids obsticales with BUG2 algorithm, follows walls using the beacon as the target
# Depending on the rotation of the vines this maybe useful to spawn the robot perpendicular to the vines and have the target point behind them. 
# This way the robot will navigate fully around each vine row 

def wall_follow():
    print("wall follow initilised")
    global twist, bot_pose, bot_motion, currentBotState, distance_moved, wall_hit_point

    # The distance '<2' should be at least as large as if the robot was inscribed in a circle
    # In Thorvalds instance its rectnagluar in shape so the enscribed circle when rotating about its centre point + some tolerance
    obstacle_in_front = numpy.any((zone_F < 2)) or numpy.any((zone_FR < 2)) or numpy.any((zone_FL < 2))
    distance_moved = math.sqrt(pow(bot_pose.position.y - wall_hit_point.y, 2) + pow(bot_pose.position.x - wall_hit_point.x, 2))
    #print(line_distance(), distance_moved, (line_distance() < 0.2 and distance_moved > 0.5))


    if line_distance() < 0.2 and distance_moved > 0.5:
        print("line_hit")
        print(distance_moved)
        # found line point. rotate and move forward via the LOOK_TOWARDS state
        twist.angular.z = 0
        twist.linear.x = 0
        currentBotState = BotState.LOOK_TOWARDS
        return
    elif numpy.any((zone_F < 2)) or numpy.any((zone_FR < 2)):  # turn left
        print("turn right")
        twist.angular.z = 1
        twist.linear.x = 0
    elif numpy.any((zone_F < 2)) or numpy.any((zone_FL < 2)):  # turn right
        print("turn right")
        twist.angular.z = -1
        twist.linear.x = 0
    else:
        print("move forward")
        twist.angular.z = 0  # move forward
        twist.linear.x = 0.5
    bot_motion.publish(twist)
    

# Rotates to face the vines according tpo the world view (-90 degrees). This uses the front IKinect HD camera but couuld have also used left or right 
# hand camera here
def rotate_to_vines():
    print("rotating to vines")
    global bot_motion, twist, bot_pose, target, currentBotState, yaw_threshold, facing_vines
    #bot_motion = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
    target_rad = target*math.pi/180 #-90 degrees to face towards vines
    #print("bot pose", bot_pose)
    # Change bot pose from Quarternion to euler to get the yaw and difference to target (-90 degrees)
    quaternion = (
        bot_pose.orientation.x,
        bot_pose.orientation.y,
        bot_pose.orientation.z,
        bot_pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw_ = euler[2]  # bot's yaw
    print("yaw", yaw_)

    if math.fabs(target_rad-yaw_) > yaw_threshold:
        print("bot pose", bot_pose)
        print("target_rad", target_rad)
        twist.linear.x = 0.0
        twist.angular.z = kp * (target_rad-yaw_)
    else:
        twist.angular.z = 0
        currentBotState = BotState.TAKE_IMAGE
        return

    bot_motion.publish(twist)

        

# --------------------------------------------- helper functions --------------------------------------


# Angles are from 180 to -180 so nneed to nrolaise tio this rather than 320 etc
def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# distance between a point and a line - right angles to the line
# When following the line we need ot know the shortest distance to jump ioff from (BUG2 algorythem -> https://automaticaddison.com/the-bug2-algorithm-for-robot-motion-planning/ )
def line_distance():
    global init_bot_pose, beacon_pose, bot_pose
    point_1 = init_bot_pose  # in form of array
    point_2 = beacon_pose.position
    point_k = bot_pose.position
    numerator = math.fabs((point_2.y - point_1[1]) * point_k.x - (point_2.x - point_1[0]) * point_k.y + (point_2.x * point_1[1]) - (point_2.y * point_1[0]))
    denominator = math.sqrt(pow(point_2.y - point_1[1], 2) + pow(point_2.x - point_1[0], 2))
    return numerator / denominator


def callback(msg):
    global beacon_pose
    beacon_pose = msg.pose
    check_init_config()
    rospy.wait_for_message("homing_signal", PoseStamped)

#def callback_2(msg):
#    global beacon_pose
#    beacon_pose = msg.pose
#    check_init_config()
#    rospy.wait_for_message("homing_signal_2", PoseStamped)

# Bot pose position relative to homing beacon
def get_base_truth(bot_data):
    global bot_pose, beacon_found, goal_distance_threshold, currentBotState
    bot_pose = bot_data.pose.pose
    if not init_config_complete:
        check_init_config()

    if beacon_pose is not None:
        goal_distance = math.sqrt(pow(bot_pose.position.y - beacon_pose.position.y, 2) + pow(bot_pose.position.x - beacon_pose.position.x, 2))
        # print(goal_distance)
        if goal_distance <= goal_distance_threshold:
            currentBotState = BotState.ROTATE_TO_VINES
            beacon_found = True
            
# Takes subscriber infor from front scan and assigns field of view zones
def process_sensor_info(data):
    global maxRange, minRange, front_obs_distance, left_obs_distance, zone_R, zone_FR, zone_F, zone_FL, zone_L
    
    # Testing print outs
    #maxRange = data.range_max
    #print("maxRange", maxRange)
    #minRange = data.range_min
    #print("minRange", minRange)


    # The range split is currently at uneven angles. We could use >> 'zone = numpy.array_split(numpy.array(data.ranges), 5)' and split into 5 equal zones?
    # However, there are 720 data.ranges. This is defined in the sensors URDF folder for the Hokuya camera (bacchus_sensors.xacro file - line 29,30), which
    # holds the args that are pulled through via the launch file. These can be changed to: min_angle="-0.7854" and max_angle="2.3562" respectivly
    # The front camera on the Thorvald is offset by 45 degrees so doesn't detect within a range - I think this is because if you set the range from
    # -180 to 180, you have a gap at between the front and back camera (must be because the cameras are mounted back to back and obviously not ontop of one another
    # For me to use data ranges I will need to amend the front sensor URDF folder to adjust for the 45 degree offset so the front zones are actually the front laser scans
    # A benefit to splitting the laserscan into zones is you can get a narrow field of view if going down tight tunnels with larger forward zone and smaller side zones
    # However, a downside I have seen through testing is that the robot can get caught looping around the same 'avoid' route - never crashing but never taking a risk either! Come on Thorvald!!

    zone = numpy.array(data.ranges) 
    zone_R = zone[0:143] 
    zone_FR = zone[144:287]
    zone_F = zone[288:431]
    zone_FL = zone[432:575]
    zone_L = zone[576:719]

    if front_obs_distance is None and left_obs_distance is None:
        front_obs_distance = 2
        left_obs_distance = 2


# --------------------------------------------- main while loop for robot --------------------------------------

def check_init_config():
    global bot_pose, beacon_pose, init_config_complete, init_bot_pose
    if bot_pose is not None and beacon_pose is not None:
        init_config_complete = True
        init_bot_pose = [bot_pose.position.x, bot_pose.position.y]
        bot_bug2()

# While loop to keep checking if homing position had been reached, meaning Thorvald is facing the vines ready to capture an image
# Shifts between states depending upon issues faced - mainly managing obsticles via wall following
def bot_bug2():
    global bot_motion, currentBotState, bot_pose
    bot_motion = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(20)
    while not taken_image:
        if not init_config_complete:
            return
        if currentBotState is BotState.LOOK_TOWARDS:
            print("look towards")
            look_towards(bot_pose)
        elif currentBotState is BotState.GOAL_SEEK:
            print("Goal Seek")
            goal_seek()
        elif currentBotState is BotState.WALL_FOLLOW:
            print("Wall Follow")
            wall_follow()
        elif currentBotState is BotState.ROTATE_TO_VINES:
            print("Rotate to Vines")
            rotate_to_vines()
        elif currentBotState is BotState.TAKE_IMAGE:
            print("Taking Image")
            image_listener()
            return
        rate.sleep()
    #print("Image captured")



# --------------------------------------------- main program --------------------------------------

def init():
    global homing_signal
    rospy.init_node("branchcounter.py")

    homing_signal = rospy.Subscriber('/homing_signal', PoseStamped, callback)

    #homing_signal_2 = rospy.Subscriber('/homing_signal_2', PoseStamped, callback_2)

    rospy.Subscriber('/thorvald_001/front_scan', LaserScan, process_sensor_info)
    rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, get_base_truth)

    print('--------- initiate bunch counterer -----------')
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
       
    except rospy.ROSInterruptException:
        pass