import rospy

# import custom_msg which is in the workspace
#from custom_msg.msg import general_parameters
from nav_msgs.msg import Odometry
import math
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import tf
import std_msgs
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
import numpy as np

from turtlesim.msg import Pose
from math import atan2, acos
from tf.transformations import euler_from_quaternion


class Zumo:
    def __init__(self, uri, sim, dt):
        """
        uri: float: simde kullanılıyorsa uri yerine 0,1,2 gibi sim modelinin 
        namespaceleri, realde kullanılıyorsa E7 gibi uri'in son iki hanesi

        sim: bool: simülasyona ait olup olmadığını belirleyen flag 
        """
        self.dt = dt
        self.uri = uri
        self.sim = sim
        self.current_position_x = float()
        self.current_position_y = float()
        self.current_yaw = float()
        self.assigned_fp = list()
        self.path_i_z = 1

        self.turtle_pos_x = float()
        self.turtle_pos_y = float()
        self.turtle_yaw = float()

        self.rate = rospy.Rate(10)

        self.difference = float()
        self.vel_ang = Twist()
        self.arrived = False
        self.flag = True

        if not self.sim:
            rospy.Subscriber(
                f"general_parameters/ika_{self.uri}", general_parameters, self.realPosCallback)
            self.pub = rospy.Publisher(
                f"goal_pose/ika_{self.uri}", Point, queue_size=10)

        else:
            rospy.Subscriber(
                f"/tb3_{self.uri}/odom", Odometry, self.callback_turtle)

            self.pub = rospy.Publisher(
                f"/tb3_{self.uri}/cmd_vel", Twist, queue_size=10)

    def realPosCallback(self, msg):
        self.current_position_x = msg.pose.x
        self.current_position_y = msg.pose.y

    def simPosCallback(self, msg):
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z
        self.current_roll = msg.pose.pose.orientation.x * 360
        self.current_pitch = msg.pose.pose.orientation.y * 360
        self.current_yaw = msg.pose.pose.orientation.z * 360

    def callback_turtle(self, msg):
        self.turtle_pos_x = msg.pose.pose.position.x
        self.turtle_pos_y = msg.pose.pose.position.y

        self.turtle_yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.turtle_yaw = self.turtle_yaw * 57.2957795

        if self.turtle_yaw < 0:
            self.turtle_yaw += 360

    def realPosCommand(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        self.pub.publish(point)

    def simPosCommand(self, x, y):
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, math.radians(0)
        )  # roll,yaw pitch için
        traj = MultiDOFJointTrajectory()  # kontrolcüye gönderilecek mesaj
        # mesaja istenen parametrelerin aktarılması (header)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = "frame"
        traj.joint_names.append("base_link")
        traj.header = header
        # istenen nokta için dönüşümler
        transforms = Transform(
            translation=Point(
                x, y, 0),
            rotation=Quaternion(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            ),
        )
        velocities = Twist()
        accelerations = Twist()
        point = MultiDOFJointTrajectoryPoint(
            [transforms], [velocities], [accelerations], rospy.Time(2)
        )
        traj.points.append(point)
        self.pub.publish(traj)

    def move(self, x, y):
        if self.sim:
            self.simPosCommand(x, y)
        else:
            self.realPosCommand(x, y)


    def constantSpeedCommamnd(self, desired_point, vel):
        """
        desired_point: liste:
        """
        dist_vec = np.array([self.current_position_x, self.current_position_y]) - np.array(desired_point)
        vel_cmd = vel * (dist_vec / np.linalg.norm(dist_vec))
        vel_cmd = vel_cmd.tolist()

        desired_x = self.current_position_x + self.dt * vel_cmd[0]
        desired_y = self.current_position_y + self.dt * vel_cmd[1]

        self.move(desired_x, desired_y)

    def steer_angle(self, goal_x, goal_y):

        angle_radian = atan2(goal_y - self.turtle_pos_y, goal_x - self.turtle_pos_x)
        angle = angle_radian * 57.2957795

        if angle < 0:
            angle += 360

        return angle
    
    def change_angle(self, goal_x, goal_y): 
        angle = self.steer_angle(goal_x, goal_y)

        self.difference = angle - self.turtle_yaw
        
        if self.difference > 180:
            self.difference = self.difference - 360

        if self.difference < -180:
            self.difference += 360
        
        if self.difference < 0:
            self.vel_ang.angular.z = -0.3
            self.pub.publish(self.vel_ang)
            self.rate.sleep()
        
        if self.difference > 2:
            self.vel_ang.angular.z = 0.3
            self.pub.publish(self.vel_ang)
            self.rate.sleep()

        self.difference = angle - self.turtle_yaw
        
        if abs(self.difference) <= 2:
            self.vel_ang.angular.z = 0
            self.pub.publish(self.vel_ang)
            
            self.arrived = True

    def move_turtle(self, goal_x, goal_y):
        if self.arrived == False:
            self.change_angle(goal_x, goal_y)

        if self.arrived == True:
            self.vel_ang.angular.z = 0
            self.vel_ang.linear.x = 0.2
            self.pub.publish(self.vel_ang)
            self.rate.sleep()

        if abs(self.turtle_pos_x - goal_x) <= 0.3 and abs(self.turtle_pos_y - goal_y) <= 0.3 and abs(self.difference) <= 2: # bir kereliğine duruyo sonra yeni path e geçiyo
            self.vel_ang.angular.z = 0
            self.vel_ang.linear.x = 0
            self.pub.publish(self.vel_ang)
            self.rate.sleep()
            self.arrived = False

    def stop_little_turtle(self): # astar daki tüm path i tamamladıktan sonra tamamen duruyo
        self.vel_ang.angular.z = 0
        self.vel_ang.linear.x = 0
        self.pub.publish(self.vel_ang)
        self.rate.sleep()
        
