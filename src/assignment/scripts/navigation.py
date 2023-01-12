
import math
from tf.transformations import euler_from_quaternion




class Navigation:
    
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

            
    
    pass