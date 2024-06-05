import numpy as np
import tf2_ros
import rospy
from swarm import hungarian
from geometry_msgs.msg import TransformStamped
import tf
import math
from swarm.drone import Drone
from swarm.zumo import Zumo


KA_FF = 5.0
KA_CF = 30.0

DRONE_ARRIVAL_TOLERANCE = 0.3
VL_ARRIVAL_TOLERANCE = 0.1

class KarmaFormation():
    def __init__(self, plan):
        self.virtual_lead_pos = [0.0, 0.0, 0.0] #X, Y, Z, Heading
        self.fp_hava_list = list()
        self.fp_kara_list = list()
        self.plan = plan
    
    def initFormation(self):

        """
        kalkış için formation clasındaki vl kullanılır, bu fonksiyon karma harakete başlanacağı zaman
        gerekli ilk değerleri temin etmek.
        """

        print('Formasyon başlatılıyor...')

        #listeleri boşalt
        self.fp_kara_list = []
        self.fp_hava_list = []

        #orta nokta vektörü

        #z değeri de alınıyor çünkü muh. takeoff sonrası çalışacak
        average_xy_vector = np.array([0, 0]) 
        average_z = 0.0

        # x,y ekseninde İHA^ların konumlarının ortalaması alındı
        for robot in self.plan.robot_list:
            robot_pos_vec = np.array([robot.current_position_x, robot.current_position_y])
            average_xy_vector = average_xy_vector + robot_pos_vec
            if isinstance(robot, Drone):
                average_z = average_z + robot.current_position_z

      
        
        average_xy_vector = average_xy_vector / len(self.plan.robot_list)
        average_z = average_z / len(self.plan.drone_list)

        average_xy_vector = average_xy_vector.tolist()
        average_xy_vector.append(average_z)
        self.virtual_lead_pos = average_xy_vector
        average_vector = np.array(self.virtual_lead_pos)

        # formasyon noktalarını güncelle, ziple tekte döndür
        for drone,zumo in zip(self.plan.drone_list, self.plan.zumo_list ):
            drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
            formation_vector = drone_pos_vec - average_vector
            formation_vector = formation_vector.tolist()
            self.fp_hava_list.append(formation_vector)

            zumo_pos_vec = np.array([zumo.current_position_x, zumo.current_position_y, zumo.current_position_z])
            formation_vector = zumo_pos_vec - average_vector
            formation_vector = formation_vector.tolist()
            self.fp_kara_list.append(formation_vector)
        

        print("fp HAVA", self.fp_hava_list)
        print("fp kara", self.fp_kara_list)
        print("vl pos", self.virtual_lead_pos)

        self.findFormationPoints(self.virtual_lead_pos,
                                            self.fp_hava_list, self.plan.drone_list, assignment=True)
        self.findFormationPoints(self.virtual_lead_pos,
                                            self.fp_kara_list, self.plan.zumo_list, assignment=True)
   
    

    def assignmentForFormation(self, gfp_list, lfp_list, robot_list, assignment=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak plan.drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """
        print("Atama gerçekleştiriliyor...")


        #global_fp_list
        #local_fp_list

        cost_matrix = []
        new_gfp_list = []
        new_lfp_list = []

        for robot in robot_list:
            for fp in gfp_list:
                robot_pos = np.array([robot.current_position_x, robot.current_position_y, robot.current_position_z])
                f_pos = np.array(fp)
                dist_vec = robot_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(cost_matrix, (len(robot_list), len(gfp_list)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for robot in robot_list:
            index = robot_list.index(robot)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)

           
            new_gfp_list.append(gfp_list[index])
            new_lfp_list.append(lfp_list[index])

            if assignment:
                robot.assigned_fp = gfp_list[index]
        

        self.fp_list = new_lfp_list #form ilerlet fp list üzerinden ilerlediği için
        return new_gfp_list, new_lfp_list    
    
    def updateVirtualLead(self, x_vel, y_vel, z_vel):
        """
        sürü merkezi referansını aldığı hız girdileri doğrultusunda öteler

        float: x_vel       : m/s
        float: y_vel       : m/s
        float: z_vel       : m/s
        float: heading_vel :derece/s

        """

        self.virtual_lead_pos[0] += x_vel       * self.plan.dt 
        self.virtual_lead_pos[1] += y_vel       * self.plan.dt
        self.virtual_lead_pos[2] += z_vel       * self.plan.dt

    
    def sendPotentialCommand(self):
        """
        İHA'ları eşleştirilmiş olduklara noktalara öteletmek için tek döngülük komut gönderir. 
        Assigned fpler güncellenmezse hover atılır. Bu işlemi potansiyel alan metodu ile yapar.
        """

        # görevi uygula
        for robot in self.robot_list:
            try:

                if type(robot) == Zumo:
                    robot.constantSpeedCommamnd(robot.assigned_fp)

                else:
                    v = robot.add_attractive_potential(robot.assigned_fp[0:2], 10.0)
                    robot.velocity_control(v, drone.assigned_fp[2])

            except ZeroDivisionError:
                pass


    
    def sendCommand(self):
        """
        İHA'ları eşleştirilmiş olduklara noktalara öteletmek için tek döngülük komut gönderir. 
        Assigned fpler güncellenmezse hover atılır. 
        """

        # görevi uygula
        for robot in self.plan.robot_list:
            try:
                if type(robot) == Zumo:  # if firefly
                    robot.move(robot.assigned_fp[0], robot.assigned_fp[1])
                else:  # if crazyflie
                    if type(robot.link_uri) == str:
                        robot.send_pose(robot.assigned_fp[0], robot.assigned_fp[1], robot.assigned_fp[2])
                    else:
                        robot.position_control(robot.assigned_fp[0], robot.assigned_fp[1], robot.assigned_fp[2], 0)

            except ZeroDivisionError:
                pass


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
    
    def findFormationPoints(self, vl_pos, fp_list, robot_list, assignment=False):
        """
        girdi olarak orta noktayı ve orta noktaya göre olan formasyon noktalarını alıp
        çıktı olarak mutlak 0'a göre olan formasyon noktalarını döner

        - fplist = [[x, y, z], [x, y, z], .... ] formasyon noktalarını vl'ye göre içeren liste
        formasyon noktalarını plannerlistte bir değişkende kaydplantenen nself.virtual_lead_heading
        - virtual_lead_pos'e bir değer atanırsa o noktada formasonu oluşturur atanmazsa en son
        vl nerede kaldıysa orada oluşturur

        !!!! Yeni formasyon oluşturmak için kullanılacak olursa self.new_formation attributesi True
        yapılmalıplan
        """

        # transformları tutacak buffer
        buffer_core = tf2_ros.BufferCore(
            rospy.Duration(10.0)
        )  # 10 saniye cache süresi galiba
        
        # mapten orta noktaya transform
        ts1 = self.transform_from_euler(
            vl_pos[0],
            vl_pos[1],
            vl_pos[2],
            0,
            0,
            0,
            "map",
            "frame1",
        )
        buffer_core.set_transform(ts1, "default_authority")
      
        i = 2
        j = 0
        formation = []

        try:

            for fp in fp_list:
                # orta noktadan formasyon noktalarına olan transform
                ts = self.transform_from_euler(
                    fp[0],
                    fp[1],
                    fp[2],
                    0,
                    0,
                    0,
                    "frame1",
                    f"frame{i}",
                )
                # orta noktadan formasyon noktasına olan transformları buffera koy
                buffer_core.set_transform(ts, "default_authority")

                # bufferdan işlenmiş transformu çek
                fp_transform = buffer_core.lookup_transform_core(
                    "map", f"frame{i}", rospy.Time(0))

                fp_global = [
                    fp_transform.transform.translation.x,
                    fp_transform.transform.translation.y,
                    fp_transform.transform.translation.z,
                ]
                # buradan rotation.z çerekerek bütün İHA'ların baş açısını çekip gönderebiliriz

                # çekilen formasyon noktasını listeye kaydet.

                if assignment:
                    robot_list[j].assigned_fp = fp_global
                
                formation.append(fp_global)
                
                i += 1 
                j += 1  
        except IndexError:
            pass


       
        return formation



    def _robotArrival(self):
        """
        Her bir İHA'nın eşleşmiş oldukları formasyon noktalarına varıp varmadıklarını kontrol eder.
        Vardılarsa true değeri dönülür.
        """
        arrival = True

        for robot in self.plan.robot_list:
            pos_vec = np.array(
                 [robot.current_position_x, robot.current_position_y, robot.current_position_z])
            target_vec = np.array(robot.assigned_fp)

            if np.linalg.norm(target_vec - pos_vec) > DRONE_ARRIVAL_TOLERANCE:
                arrival = False

        return arrival

    def _virtualLeadArrival(self, target_pos_vec):
        if (np.linalg.norm(np.array(target_pos_vec) - np.array(self.virtual_lead_pos[0:2])) > VL_ARRIVAL_TOLERANCE):
            print(np.linalg.norm(np.array(target_pos_vec) - np.array(self.virtual_lead_pos[0:2])))
            return False
        else:
            return True