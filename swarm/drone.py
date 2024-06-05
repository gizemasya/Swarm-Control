#!/usr/bin/env python

# from hibrit_c1 import hungarian
import matplotlib.pyplot as plt
from swarm import hungarian
import tf2_ros
from geometry_msgs.msg import TransformStamped
import rospy
import math
import time
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform, Quaternion

# from geometry_msgs.msg import Point32
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import tf
from nav_msgs.msg import Odometry
import numpy as np

# import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.commander import Commander
from datetime import datetime
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float32
#from custom_msg.msg import obstacle

from geometry_msgs.msg import Point
import datetime
"""from custom_msg.msg import obstacle
from custom_msg.msg import general_parameters #import custom_msg which is in the workspace"""


class Drone:
    def __init__(self, ns, link_uri=0, sim=False, zumo=False, pos_offset=[0,0]):
        self.link_uri = link_uri
        self.ns = ns
        self.sim = sim
        self.zumo = zumo
        self.rvo_time = 0
        self.rvo_flag = False
        self.pose_offset = pos_offset

        if self.zumo:
            self.d_h = 0

            # burada diğer sub, pub olayları için gerekenleri yaz

        if type(self.link_uri) == str and self.sim == True:
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                self.set_state = rospy.ServiceProxy(
                    '/gazebo/set_model_state', SetModelState)
            except rospy.ServiceException:
                print("Service call failed")

            self.state_msg = ModelState()
            # self.state_msg.model_name = 'firefly'
            self.state_msg.model_name = f'firefly{self.ns}'

            # drone namespace numarası

        self.connected = False

        if type(self.link_uri) == str:
            # cflib
            self._cf = Crazyflie(rw_cache="./cache")
            self._cf.connected.add_callback(self._connected)
            self._cf.disconnected.add_callback(self._disconnected)
            self._cf.connection_failed.add_callback(self._connection_failed)
            self._cf.connection_lost.add_callback(self._connection_lost)
            print("Connecting to %s" % link_uri)
            # Try to connect to the Crazyflie
            self._cf.open_link(link_uri)
            # Variable used to keep main loop occupied until disconnect
            self.is_connected = False
            self.command = Commander(self._cf)
            self.crazyswarm_pose = []

        if (type(self.link_uri) == str) and (self.zumo == True):
            uri = link_uri[-2:]
            rospy.Subscriber(
                f"general_parameters/ika_{uri}", general_parameters, self.callback_zumo)

        # ----------------------------------------------------------

        else:
            rospy.Subscriber(
                f"/firefly{ns}/odometry_sensor1/odometry", Odometry, self.callback
            )
            rospy.Subscriber(
                f"/firefly{ns}/odometry_sensor1/odometry", Odometry, self.callback_hız)

            # for i in range(4):
            #    rospy.Subscriber(f"/firefly{ns}/motor_speed/{i}", Float32, self.callback_angular)

        # dronenun mape göre mevcut konumu
        self.current_position_x = 0
        self.current_position_y = 0
        self.current_position_z = 0
        self.current_roll = 0
        self.current_yaw = 0
        self.current_pitch = 0
        self.current_battery = 0
        self.current_vx = 0
        self.current_vy = 0
        # varılması istenen noktalar (vel control ve pos control kullanıyor)
        self.desired_x = 0
        self.desired_y = 0
        self.desired_z = 0
        # droneların formasyon noktalarını bulunduran liste [f1=[x,y,z], f2=[x,y,z], .. ]
        self.formation = []
        # sürüdeki diğer droneların nesnelerini bulundurur
        self.detected_coworkers = []
        # vl'nin konumunu içeren liste [x, y, z] !!z olmak zorunda
        self.virtual_lead_pos = [0, 0, 0]
        # dronea atanmış formasyon noktasının konumunu içeren liste [x, y, z]
        self.assigned_fp = []
        # dronun initial heading'den ne kadar döndüğünü veren açı (flag kontrolü için)
        self.rotated_angle = 0
        # hangi formasyon haraketinin gerçekleştiğini ifade eden flagler
        self.formed_formation = False
        self.rotating = False
        # vl framenin, map frameine göre, saat yönünün tersinde derece cinsinden kaç
        # derece döndüğününü veren açı
        self.current_heading = 0
        # eş zamanlı atama algoritmasından gelen noktaları içeren liste
        self.paired_points = []
        # eş zamanlı atama noktasına publish edilecek noktaları içeren liste
        self.formation_points = []
        # default irtifa
        self.d_h = 1
        #
        self.dx = 0
        self.dy = 0

        # drone kalkışını initialize etme
        self.first_run_take_off = True  # drone kaldırırken, ilk noktasını korumak için flag
        self.first_run_land = True  # droneları indirme için kullanılan flag
        self.take_off_pos = []  # dronun ilk bağlantı kurduğu noktası.
        self.land_pos = []  # dronun inmek için koruyacağı nokta.
        self.hover_pos = []

        # --------------------- PROTOTIP ---------------------------
        self.liquid_formation = False
        # hedef noktası ile engel arasında bulunan engeller (check_obs_intersection)
        self.obs_list_i = []
        # -----------------------------------------------------

        # eski
        self.desired_heading = []
        self.angle = 0

        # Formasyon
        self.formation_name = ''
        self.formation_radius = 2.0
        self.first_run_form_land = True

        # ASTAR
        self.path = []  # [[x,y], [x,y], .....] a star hedef noktalar
        self.route = True  # yol çizme talimatı için flag
        self.path_i = 1  # self.path index
        self.path_i_d = 1

        # DÖNGÜ
        self.dt = 1 / 30

        # lol
        self.fire_point_x = []
        self.fire_point_y = []

        self.current_thrust = 0

        self.potansiyel_flag = False
        self.varış_noktası = []
        self.m1 = 1
        self.m2 = 1
        self.m3 = 1
        self.m4 = 1

        # 3. görev kontrol için
        self.first_run = False
        self.first_arrival_time = 0

        # 2. görev kontrol için
        self.first_run_2 = False
        self.first_arrival_time_2 = 0


    """
                    ---CALLBACK FONKSİYONLARI---
    """

    def callback_zumo(self, data):
        self.current_position_x = data.pose.x
        self.current_position_y = data.pose.y

    def callback_hız(self, msg):
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y

    def callback(self, msg):  # GAZEBO POSE
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z
        self.current_roll = msg.pose.pose.orientation.x * 360
        self.current_pitch = msg.pose.pose.orientation.y * 360
        self.current_yaw = msg.pose.pose.orientation.z * 360

        """print(self.current_roll, 'ROL')
        print(self.current_pitch, 'PİTCH')"""
        # rospy.loginfo('Firefly{} x: {}, y: {}, z: {}'.format(self.ns,
        # self.current_position_x, self.current_position_y, self.current_position_z))

    def callback_angular(self, msg):
        self.vel_1 = msg.data
        self.vel_2 = msg.data
        self.vel_3 = msg.data
        self.vel_4 = msg.data
        self.current_ang_velocitys = [
            self.vel_1, self.vel_2, self.vel_3, self.vel_4]

    """
                ---CFLİB FONKSİYONLARI---
    """

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)

        self.connected = True

        # OPTİMİZASYON İÇİN STABILIZER VE PM KAPATILABİLİR

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=500)
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('stabilizer.thrust', 'float')

        # ______________

        self._lg_stab1 = LogConfig(name="stateEstimate", period_in_ms=30)
        self._lg_stab1.add_variable("stateEstimate.x", "float")
        self._lg_stab1.add_variable("stateEstimate.y", "float")
        self._lg_stab1.add_variable("stateEstimate.z", "float")
        self._lg_stab1.add_variable("stateEstimate.vx", "float")
        self._lg_stab1.add_variable("stateEstimate.vy", "float")
        # ______________

        self._lg_stab2 = LogConfig(name="pm", period_in_ms=1000)
        self._lg_stab2.add_variable("pm.batteryLevel", "float")

        self._lg_stab3 = LogConfig(name="Motor", period_in_ms=1000)
        self._lg_stab3.add_variable('motor.m1', 'float')
        self._lg_stab3.add_variable('motor.m2', 'float')
        self._lg_stab3.add_variable('motor.m3', 'float')
        self._lg_stab3.add_variable('motor.m4', 'float')

        self._lg_stab5 = LogConfig(name="radio", period_in_ms=1000)  # 1hz
        self._lg_stab5.add_variable("radio.isConnected", "uint8_t")

        # --------------------------------------------------------

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_stab1)
            self._cf.log.add_config(self._lg_stab2)
            self._cf.log.add_config(self._lg_stab3)
            self._cf.log.add_config(self._lg_stab5)

            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab1.data_received_cb.add_callback(self._stab_log_data1)
            self._lg_stab2.data_received_cb.add_callback(self._stab_log_data2)
            self._lg_stab3.data_received_cb.add_callback(self._stab_log_data3)
            self._lg_stab5.data_received_cb.add_callback(self._stab_log_data5)

            # Start the logging
            self._lg_stab.start()
            self._lg_stab1.start()
            self._lg_stab2.start()
            self._lg_stab3.start()
            self._lg_stab5.start()

        except KeyError as e:
            print(
                "Could not start log configuration,"
                "{} not found in TOC".format(str(e))
            )
        except AttributeError:
            print("Could not add Stabilizer log config, bad configuration.")

    def _stab_log_data5(self, timestamp, data, logconf):
        self.is_connected = data['radio.isConnected']

    def _stab_log_data3(self, timestamp, data, logconf):  # pwm bakılıyor
        self.m1 = data['motor.m1']
        self.m2 = data['motor.m2']
        self.m3 = data['motor.m3']
        self.m4 = data['motor.m4']

    def _stab_log_data2(self, timestamp, data, logconf):
        self.current_battery = data['pm.batteryLevel']
        # print('batarya', self.current_battery)

    def _stab_log_data1(self, timestamp, data, logconf):
        #start = datetime.datetime.now()
        self.current_position_x = data['stateEstimate.x'] + self.pose_offset[0]
        self.current_position_y = data['stateEstimate.y'] + self.pose_offset[1]
        self.current_position_z = data['stateEstimate.z']
        self.current_vx = data['stateEstimate.vx']
        self.current_vy = data['stateEstimate.vy']

        #end = datetime.datetime.now()

        #print("pos freq", (end-start))
        #print(self.current_position_x)

        if self.sim:
            self.state_msg.pose.position.x = self.current_position_x
            self.state_msg.pose.position.y = self.current_position_y
            self.state_msg.pose.position.z = self.current_position_z + 0.08
            self.state_msg.pose.orientation.w = 1
            self.state_msg.pose.orientation.x = self.current_pitch / 360
            self.state_msg.pose.orientation.z = self.current_yaw / 360
            self.state_msg.pose.orientation.y = self.current_roll / 360
            resp = self.set_state(self.state_msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""

        self.current_roll = data['stabilizer.roll']
        self.current_pitch = data['stabilizer.pitch']
        self.current_yaw = data['stabilizer.yaw']
        self.current_thrust = data['stabilizer.thrust']

        """rospy.loginfo(
            f" POSITION x:  {self.current_position_x:3.3f},  y: {self.current_position_y:3.3f},  z: {self.current_position_z:3.3f}"
        )
        rospy.loginfo(
            f" ORIENTATION x: {self.current_roll:3.3f},  y: {self.current_pitch:3.3f},  z: {self.current_yaw:3.3f}"
        )"""

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))
        self.is_connected = False


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)
        self.is_connected = False

    """
                    --- DRON LİSTE KONTROLÜ ---
    """

    def update_dronelist(self, dronelist):
        """
        görev değişimi görevi için eklenmiş fonksiyon, mevcut droneları sürekli kontrol
        eder, bir drone listeden çıkarılduriığı anda kendi listesini duruma göre düzenler
        :param dronelist: droneların objelerini bulunduran liste
        """
        self.detected_coworkers = dronelist.copy()
        # rospy.loginfo(f'coworkers: {self.detected_coworkers}')
        i = 0

        while i < len(self.detected_coworkers):
            # rospy.loginfo(f'ns: {self.detected_coworkers[i].ns}')

            if self.detected_coworkers[i].ns == self.ns:
                self.detected_coworkers.remove(self.detected_coworkers[i])
                continue

            else:
                i += 1

    def update_threatlist(self, deadband):
        """
        for döngüsü ile listeden bir şeyler çıkarmaya
        çalışınca indexler sürekli kaydığından komik şeyler oluyor xd
        :return: z seviyesi farkı deadbandten yüksek olan
        öteki droneların objelerini içermeyen bir liste
        """

        updated_list = self.detected_coworkers.copy()
        i = 0

        while i < len(updated_list):

            if updated_list[i].ns == self.ns:
                updated_list.remove(updated_list[i])
                continue

            elif (
                    abs(updated_list[i].current_position_z -
                        self.current_position_z)
                    >= deadband
            ):
                updated_list.remove(updated_list[i])
                continue

            else:
                i += 1

        # rospy.loginfo(f'drone{self.ns}:  {updated_list}')
        return updated_list

    """
                    --- KONTROLCÜLER ---
    """

    def send_pose(self, x, y, z):
        self.command.send_position_setpoint(x - self.pose_offset[0], y - self.pose_offset[1], z, 0)

    def vel_control(self, vx, vy, z):
        self.command.send_hover_setpoint(vx, vy, 0, z)

    def set_global_vel(self, vx, vy, vz):
        self.command.send_velocity_world_setpoint(vx, vy, vz, 0)

    def velocity_control_land(self, desired_velocity):  # x hız, y hız, z konum
        """
        x y girdisi olarak hız vektörü z girdisi olarak konum alıyor.
        desired_velocity: list: [float:x, float:y, float:z]:
        dt: float:
        """
        try:

            self.desired_x = self.current_position_x + \
                desired_velocity[0] * self.dt
            self.desired_y = self.current_position_y + \
                desired_velocity[1] * self.dt
            self.desired_z = self.current_position_z + \
                desired_velocity[2] * self.dt

            # rospy.loginfo("x:{} y:{} z:{}".format(self.desired_x,self.desired_y,self.desired_z))
            if self.link_uri == 0:  # eğer cf ise
                self.position_control(
                    self.desired_x, self.desired_y, self.desired_z, 0)
            else:  # eğer reel ise
                self.send_pose(self.desired_x, self.desired_y, self.desired_z)

        except:
            pass

    def velocity_control(self, desired_velocity, desired_height):  # x hız, y hız, z konum
        """
        x y girdisi olarak hız vektörü z girdisi olarak konum alıyor.
        desired_velocity: list: [int:x, int:y]
        desired_height: int:
        dt: int:
        """
        try:
            self.desired_x = self.current_position_x + \
                desired_velocity[0] * self.dt
            self.desired_y = self.current_position_y + \
                desired_velocity[1] * self.dt

            # rospy.loginfo("x:{} y:{} z:{}".format(self.desired_x,self.desired_y,self.desired_z))
            if self.link_uri == 0:  # eğer cf ise
                self.position_control(
                    self.desired_x, self.desired_y, desired_height, 0)
            elif self.zumo:  # ZUMOYSAN GİR İSMAİL ANLASIN DİYE YAZILDI
                self.zumo_move(self.desired_x, self.desired_y, 0)
            else:  # eğer reel ise
                self.send_pose(self.desired_x, self.desired_y, desired_height)

        except:
            pass

    def velocity_control_z(self, x, y, v_z):
        """
        x, y girdisi olarak konum alıyor, z girdisi olarak hız vektörü
        desired_height: int:
        dt: int:
        """
        try:
            self.desired_z = self.current_position_z + v_z * self.dt
            # rospy.loginfo("x:{} y:{} z:{}".format(self.desired_x,self.desired_y,self.desired_z))

            if self.link_uri == 0:  # eğer cf değil ise
                self.position_control(
                    x, y, self.desired_z, 0)
            else:  # eğer reel ise
                self.send_pose(x, y, self.desired_z)
        except:
            pass

    def zumo_move(self, x, y, z):
        uri = self.link_uri[-2:]
        pub = rospy.Publisher(f"goal_pose/ika_{uri}", Point, queue_size=10)
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        pub.publish(point)

    def position_control(
            self, desired_x_to_go, desired_y_to_go, desired_z_to_go, desired_yaw
    ):
        firefly_command_publisher = rospy.Publisher(
            "/firefly{}/command/trajectory".format(self.ns),
            MultiDOFJointTrajectory,
            queue_size=10,
        )
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, math.radians(desired_yaw)
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
                desired_x_to_go, desired_y_to_go, desired_z_to_go),
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
        firefly_command_publisher.publish(traj)

    # -------------------------------------------------------------------------------------------------------------------------------------
    #                            ANTI LOKAL MINIMA
    # ------------------------------------------------------------------------------------------------------------------------------------

    def add_cohesion(self, drone_list, kc):
        pos = np.array([0, 0])
        for drone in drone_list:
            x = drone.current_position_x
            y = drone.current_position_y
            pos_vec = np.array([x, y])
            pos = pos + pos_vec
        pos = kc * (pos / len(drone_list))
        return pos

    # -------------------------------------------------------------------------------------------------------------------------------------
    #                            ATTRACTIVE FONKSIYONLAR
    # ------------------------------------------------------------------------------------------------------------------------------------

    def constant_speed(self, wp, speed):
        """
        wp : [x,y]
        speed: float
        """
        dp = np.array(wp)
        cp = np.array([self.current_position_x, self.current_position_y])

        v = dp - cp
        v = (v / np.linalg.norm(v)) * speed
        return v

    def draw_path(self, gx, gy, threatlist, grid_size=1, robot_radius=0.8, sx=[], sy=[], destination_list=[]):

        # engelleri ekleyecek fonksiyonı çağır, ox oy ata
        if sx == [] and sy == []:
            sx = self.current_position_x
            sy = self.current_position_y

        min_x = -2.5
        min_y = -2.5

        max_x = 2.5
        max_y = 2.5

        ox = []
        oy = []

        i = 0
        for x, y in zip(self.fire_point_x, self.fire_point_y):
            if i % 5 == 0:
                ox.append(x)
                oy.append(y)
            i += 1

        # alt yatay sınır
        for i in np.arange(min_x, max_x, grid_size * 10):
            ox.append(i)
            oy.append(min_y)
        # üst yatay sınır
        for i in np.arange(min_x, max_x, grid_size * 10):
            ox.append(i)
            oy.append(max_y)
            # alt yatay sınır
        for i in np.arange(min_y, max_y, grid_size * 10):
            ox.append(min_x)
            oy.append(i)
            # üst yatay sınır
        for i in np.arange(min_y, max_y, grid_size * 10):
            ox.append(max_x)
            oy.append(i)

        for threat in threatlist:

            # daire çiz
            x_center = threat.current_position_x
            y_center = threat.current_position_y
            # radius = threat.width
            radius = threat.radius
            for angle in np.arange(0, 360, 0.2):
                x = x_center + radius * np.cos(angle)
                y = y_center + radius * np.sin(angle)

                point = [x, y]

                ox.append(point[0])
                oy.append(point[1])

        a_star = a.AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)

        rx.reverse()
        ry.reverse()

        """if True:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.pause(0.0001)
            plt.show()"""

        for x, y in zip(rx, ry):
            point = [x, y]
            self.path.append(point)

        if rx == [gx]:
            self.varış_noktası = [gx, gy]
            self.potansiyel_flag = True

        return rx, ry

    def draw_path_rrt(self, gx, gy, threatlist, robot_radius=0.4):
        # engelleri ekleyecek fonksiyonı çağır, ox oy ata

        obstacle_list = []  # [x,y,size(radius)]
        """for threat in threatlist:
            obstacle_list.append((threat.current_position_x, threat.current_position_y, threat.width))"""
        obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                        (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
        rrt = r.RRT(
            start=[self.current_position_x, self.current_position_y],
            goal=[2, 2],
            rand_area=[-2, 15],
            obstacle_list=obstacle_list,
            play_area=[-5, 20, -5, 20],
            robot_radius=robot_radius
        )
        path = rrt.planning(animation=False)
        path = path.reverse()
        print(path)
        self.path = path

    def is_arrived_list(self, desired_point_list, deadband=0.40):
        """
        desired_point_list = [[x,y], [x,y], [x,y], .....]

        """

        drone_pos_vec = np.array(
            [
                self.current_position_x,
                self.current_position_y,
            ]
        )

        for dp in desired_point_list[self.path_i:]:
            desired_pos_vec = np.array(dp)

            dist = drone_pos_vec - desired_pos_vec
            dist_mag = np.linalg.norm(dist)

            print(dist_mag)

            if abs(dist_mag) > deadband:
                return False
            else:
                return True

    def is_arrived(self, desired_point, deadband=0.3):
        """
        desired_point = [x, y, z]
        """

        drone_pos_vec = np.array([self.current_position_x,
                                  self.current_position_y
                                  ]
                                 )

        desired_pos_vec = np.array(desired_point)

        dist = drone_pos_vec - desired_pos_vec
        dist_mag = np.linalg.norm(dist)

        print(dist_mag)

        if abs(dist_mag) > deadband:
            return False
        else:
            return True

    def add_attractive_potential(self, desired_point, ka):
        v_at_x = -ka * (self.current_position_x - desired_point[0])
        v_at_y = -ka * (self.current_position_y - desired_point[1])
        vel = [v_at_x, v_at_y]
        return vel

    def add_attractive_potential_z(self, desired_altitude, ka):
        set_point_z = -ka * (self.current_position_z - desired_altitude)
        return set_point_z

    def add_constant_speed_z(self, desired_altitude, ka):
        set_point_z = -ka * (self.current_position_z - desired_altitude)
        return set_point_z

    def add_attractive_potential_vl(self, desired_point, ka):
        v_at_x = -ka * (self.virtual_lead_pos[0] - desired_point[0])
        v_at_y = -ka * (self.virtual_lead_pos[1] - desired_point[1])
        vel = [v_at_x, v_at_y]
        return vel

    # ---------------------------------------------------------------------------------------------------------------------------------------
    #                                 REPULSIVE FONKSIYONLAR
    # --------------------------------------------------------------------------------------------------------------------------------------

    # threat argümanı: obstacle obje listesini veya drone obje listesini gir

    def add_repulsive_potential(
            self, threat_list, safe_dist, kr
    ):  # droneların birbirinden kaçınması için kullanılıyor. z ekseni için eklemeler yapılabilir
        v_repp = np.array([0, 0])
        for threat in threat_list:
            x_dist = -self.current_position_x + threat.current_position_x
            y_dist = -self.current_position_y + threat.current_position_y
            dist = math.sqrt((x_dist ** 2) + (y_dist ** 2))
            if dist <= safe_dist:
                vec_x = -kr * (1 - (dist / safe_dist)) * (x_dist / (dist ** 3))
                vec_y = -kr * (1 - (dist / safe_dist)) * (y_dist / (dist ** 3))
                vec = np.array([vec_x, vec_y])
                v_repp = v_repp + vec
            elif dist > safe_dist:
                pass
        v_repp.tolist()
        return v_repp

    def add_virtual_repulsive_potential(
            self, threat_list, kd, kr, kv, proportional
    ):  # engellerden kaçınma için alternatif
        v_repp = np.array([0, 0])
        for threat in threat_list:
            # desired_point liste, gitmek istediğin noktanın listesi x y z
            # fonksiyon sana istenilen nokta ile arandaki mesafeyi veriyor

            x_dist = -self.current_position_x + threat.current_position_x
            y_dist = -self.current_position_y + threat.current_position_y
            dist = math.sqrt((x_dist ** 2) + (y_dist ** 2))
            width = threat.width
            height = threat.height

            if threat.two_dim == True:
                if x_dist == 0:
                    r = threat.heigth / 2
                elif y_dist == 0:
                    r = width / 2
                elif abs(y_dist) < abs(x_dist):
                    a = (width / 2) * (abs(y_dist) / abs(x_dist))
                    r = math.sqrt(a ** 2 + (width / 2) ** 2)
                elif abs(y_dist) > abs(x_dist):
                    a = (height / 2) * (abs(x_dist) / abs(y_dist))
                    r = math.sqrt(a ** 2 + (height / 2) ** 2)
            else:
                r = threat.radius

            if proportional == True:
                safe_dist = r * kd
            else:
                safe_dist = kd

            dist_vec = [x_dist, y_dist]
            unit_vec = np.array(dist_vec) / dist
            dist_vec = dist_vec - (unit_vec * r)
            dist_vec.tolist()

            dist_mag = math.sqrt(dist_vec[0] ** 2 + dist_vec[1] ** 2)
            if dist_mag <= 0:
                v_repp_vir_x = -x_dist * 15
                v_repp_vir_y = -y_dist * 15

            elif dist_mag <= safe_dist:
                v_repp_x = (
                    -kr * (1 - (dist_mag / safe_dist)) *
                    (dist_vec[0] / (dist_mag ** 3))
                )
                v_repp_y = (
                    -kr * (1 - (dist_mag / safe_dist)) *
                    (dist_vec[1] / (dist_mag ** 3))
                )
                v_repp_vir_x = v_repp_x - (
                    kv
                    * (
                        dist_vec[0]
                        / math.sqrt((dist_vec[0] ** 2 + dist_vec[1] ** 2) ** 3)
                    )
                )
                v_repp_vir_y = v_repp_y - (
                    kv
                    * (
                        dist_vec[1]
                        / math.sqrt((dist_vec[0] ** 2 + dist_vec[1] ** 2) ** 3)
                    )
                )
                vec = np.array([v_repp_vir_x, v_repp_vir_y])
                v_repp = v_repp + vec
            elif dist_mag > safe_dist:
                pass
        v_repp.tolist()
        return v_repp

    """
                    --- TRANSFORM FONKSIYONLARI ---
    """

    def transform_from_euler(
            self, x, y, z, roll, pitch, yaw, header_frame_id, child_frame_id
    ):
        """
        Creates a new stamped transform from translation and euler-orientation

        :param x: x translation
        :param y: y translation
        :param z: z translation
        :param roll: orientation roll
        :param pitch: orientation pitch
        :param yaw: orientation yaw
        :param header_frame_id: transform from this frame
        :param child_frame_id: transform to this frame

        :returns: transform
        """
        t = TransformStamped()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = tf.transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        return t

    """
    FORMASYON FONKSIYONLARI
    """

    def land_single(self, vel):
        # haraketi uygula

        # iniş için koruyacağı noktayı belirle
        if self.first_run_land:
            self.land_pos = [self.assigned_fp[0], self.assigned_fp[1]]
            self.z = self.current_position_z
            self.first_run_land = False
        else:
            pass

            # indir
        try:
            # komutu gönder
            vel_sp = np.array(self.assigned_fp) - np.array(
                [self.current_position_x, self.current_position_y, self.current_position_z])
            vel_sp = vel * (vel_sp / np.linalg.norm(vel_sp))
            vel_sp = vel_sp.tolist()

            self.z += vel_sp[2]*self.dt

            if self.link_uri == 0:
                self.velocity_control(vel_sp[0:2], self.z)
            else:
                #self.vel_control(vel_sp[0], vel_sp[1], self.z)
                self.set_global_vel(vel_sp[0], vel_sp[1], vel_sp[2])

        except ZeroDivisionError:
            pass

        cond = self.current_position_z <= 0.1  # 0 veya 1 dönüyor

        if cond:

            if self.link_uri == 0:
                pass
            else:
                self.command.send_stop_setpoint()
            return True
        else:  # b == 0
            self.first_run_land = True
            return False

    def take_off_a(self, rate, dronelist):
        """
        rate: int: sürünün haraket hizi
        desired_point: list[x,y]: hedef nokta
        dronelist: droneların objesini bulunduran liste
        draw_traj: bool: True girilirse hedef noktaya gidilirken vl'ye yol çizilir
        dt: int: içinde çağirildiği loopun periyodu (duty loop preiyodu, self.dt)
        """
        # hız vektörü belirleme

        # virtual_lead'ten desired pointe doğru bir vektör çizer, ünit vektör olarak
        # yön vektörü

        # virtual leaderin yeni konumu hesaplanır
        self.virtual_lead_pos[2] += rate * self.dt

        self.form_formation(
            self.virtual_lead_pos, dronelist, 'triangle', r=self.formation_radius
        )

    def take_off(self, desired_height=1.5, ka=1.2):
        """
        dronea kalkiş vektörü dönüyor.
        """
        self.add_attractive_potential_z(desired_height, ka)
        self.d_h = desired_height

    def land(self, ka=1.2):
        """
        dronea kalkiş vektörü dönüyor.
        """
        self.add_attractive_potential_z(0, ka)
        self.d_h = 0

    """
    GÖREV KONTROL
    """

    def _is_arrived(self, deadband=0.1):

        drone_pos_vec = np.array(
            [
                self.current_position_x,
                self.current_position_y,
                self.current_position_z,
            ]
        )

        desired_pos_vec = np.array(self.assigned_fp)

        dist = drone_pos_vec - desired_pos_vec
        dist_mag = np.linalg.norm(dist)
        print(dist_mag)
        if abs(dist_mag) > deadband:
            return 0  # False
        else:
            return 1  # True

    """
    PROTOTIP
    """


class Obstacle:
    
    def __init__(self, id,obstacle_width):

      
        self.id = id
        self.threat = []
        self.counter = 0
        self.cb_flag = True
        
        self.obs_width = obstacle_width

        self.pose_x = float()
        self.pose_y = float()
        team_name = "obstacle_" + id

        rospy.Subscriber(f"/obstacle/{team_name}", obstacle, self.callback)
        time.sleep(1)

    def callback(self,data):
        string_1 = "\nTeam Name: " + str(data.obstacle_name) +"\nx: "+  str(data.pose.x)
        string_2 = "\ny: "+  str(data.pose.y) + "\nz: "+  str(data.pose.z)
        string = string_1 + string_2

        self.pose_x = data.pose.x
        self.pose_y = data.pose.y


        #rospy.loginfo(rospy.get_caller_id() + " I heard %s", string)

            

    def get(self):
     
        self.threat = [self.pose_x,self.pose_y,self.obs_width]

    
        return self.threat