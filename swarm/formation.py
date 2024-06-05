import numpy as np
import tf2_ros
import rospy
from swarm import hungarian
from geometry_msgs.msg import TransformStamped
import tf
import math
from swarm import drone


KA_FF = 5.0
KA_CF = 30.0


class Formation():
    def __init__(self, mission):
        self.virtual_lead_pos = [0.0, 0.0, 0.0, 0.0]  # X, Y, Z, Heading
        self.fp_list = list()
        self.mission = mission  # mission objesi

    def initFormation(self):
        """
        İHA'lar kalkmadan önce İHA'ların yerdeki mevcut dizilimini formasyon olarak tanımlar.
        İHA'ların konumlarının ortalamalarını orta nokta olarak init eder.

        """

        print('Formasyon başlatılıyor...')

        self.fp_list = []

        # orta nokta vektörü
        average_vector = np.array([0, 0, 0])

        # konumların ortalaması alındı
        for drone in self.mission.drone_list:
            drone_pos_vec = np.array(
                [drone.current_position_x, drone.current_position_y, drone.current_position_z])
            average_vector = average_vector + drone_pos_vec
        average_vector = average_vector / len(self.mission.drone_list)

        self.virtual_lead_pos = average_vector.tolist()

        # heading eklendi
        self.virtual_lead_pos.append(0.0)

        # fp_list'i halleder

        # formasyon noktalarını güncelle
        for drone in self.mission.drone_list:
            drone_pos_vec = np.array(
                [drone.current_position_x, drone.current_position_y, drone.current_position_z])
            formation_vector = drone_pos_vec - average_vector
            formation_vector = formation_vector.tolist()
            self.fp_list.append(formation_vector)

        self.findFormationPoints(assignment=True)

        print('Formasyon başlatıldı')
        return True

    def updateFormation(self):
        """
        İHA'lar kalkmadan önce İHA'ların yerdeki mevcut dizilimini formasyon olarak tanımlar.
        İHA'ların konumlarının ortalamalarını orta nokta olarak init eder.

        """
        print("FP_LIST", self.fp_list)
        print('Formasyon güncelleniyor...')

        self.fp_list = []

        # orta nokta vektörü

        # formasyon noktalarını güncelle
        for drone in self.mission.drone_list:
            drone_pos_vec = np.array(
                [drone.current_position_x, drone.current_position_y, drone.current_position_z])
            print("drone_pos_vector: ", drone_pos_vec)
            formation_vector = drone_pos_vec - np.array(self.virtual_lead_pos[0:3])
            formation_vector = formation_vector.tolist()
            print("formasyon vector: ", formation_vector)
            self.fp_list.append(formation_vector)

        print("FP_LIST", self.fp_list)

        self.findFormationPoints(assignment=True)

        print("FP_LIST", self.fp_list)

        print('Formasyon başlatıldı')
        return True

    def assignment(self, formation_points, assignment=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak mission.drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """
        print("Atama gerçekleştiriliyor...")

        cost_matrix = []
        global_fp_list = []

        for drone in self.mission.drone_list:
            for fp in formation_points:
                drone_pos = np.array(
                    [drone.current_position_x, drone.current_position_y, drone.current_position_z])
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(
            cost_matrix, (len(self.mission.drone_list), len(formation_points)))
        # Get the element position.
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in self.mission.drone_list:
            index = self.mission.drone_list.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)

            global_fp_list.append(formation_points[index])

            if assignment:
                drone.assigned_fp = formation_points[index]

        return global_fp_list

    def assignmentForFormation(self, gfp_list, lfp_list, assignment=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak mission.drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """
        print("Atama gerçekleştiriliyor...")

        # global_fp_list
        # local_fp_list

        cost_matrix = []
        new_gfp_list = []
        new_lfp_list = []

        for drone in self.mission.drone_list:
            for fp in gfp_list:
                drone_pos = np.array(
                    [drone.current_position_x, drone.current_position_y, drone.current_position_z])
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(
            cost_matrix, (len(self.mission.drone_list), len(gfp_list)))
        # Get the element position.
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in self.mission.drone_list:
            index = self.mission.drone_list.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)

            new_gfp_list.append(gfp_list[index])
            new_lfp_list.append(lfp_list[index])

            if assignment:
                drone.assigned_fp = gfp_list[index]

        self.fp_list = new_lfp_list  # form ilerlet fp list üzerinden ilerlediği için
        return new_gfp_list, new_lfp_list

    def findFormationPoints(self, vl_pos=[], assignment=False):
        """
        girdi olarak orta noktayı ve orta noktaya göre olan formasyon noktalarını alıp
        çıktı olarak mutlak 0'a göre olan formasyon noktalarını döner

        - fplist = [[x, y, z], [x, y, z], .... ] formasyon noktalarını vl'ye göre içeren liste
        formasyon noktalarını plannerlistte bir değişkende kaydmissiontenen nself.virtual_lead_heading
        - virtual_lead_pos'e bir değer atanırsa o noktada formasonu oluşturur atanmazsa en son
        vl nerede kaldıysa orada oluşturur

        !!!! Yeni formasyon oluşturmak için kullanılacak olursa self.new_formation attributesi True
        yapılmalımission
        """

        if len(vl_pos) == 0:
            vl_pos = self.virtual_lead_pos
        else:
            pass

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
            vl_pos[3],
            "map",
            "frame1",
        )
        buffer_core.set_transform(ts1, "default_authority")

        i = 2
        j = 0
        formation = []

        try:
            for fp in self.fp_list:
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
                    self.mission.drone_list[j].assigned_fp = fp_global

                formation.append(fp_global)

                i += 1
                j += 1
        except IndexError:
            pass

        return formation

    def updateVirtualLead(self, x_vel, y_vel, z_vel, heading_vel):
        """
        sürü merkezi referansını aldığı hız girdileri doğrultusunda öteler

        float: x_vel       : m/s
        float: y_vel       : m/s
        float: z_vel       : m/s
        float: heading_vel :derece/s

        """

        self.virtual_lead_pos[0] += x_vel * self.mission.dt
        self.virtual_lead_pos[1] += y_vel * self.mission.dt
        self.virtual_lead_pos[2] += z_vel * self.mission.dt
        self.virtual_lead_pos[3] += heading_vel * self.mission.dt

    def sendPotentialCommand(self):
        """
        İHA'ları eşleştirilmiş olduklara noktalara öteletmek için tek döngülük komut gönderir. 
        Assigned fpler güncellenmezse hover atılır. Bu işlemi potansiyel alan metodu ile yapar.
        """

        # görevi uygula
        for drone in self.mission.drone_list:
            try:
                if type(drone.link_uri) == str:
                    # MAX VEL parametresi
                    print(f"drone sayı {drone.ns}")
                    v = drone.add_attractive_potential(
                        drone.assigned_fp[0:2], 1.5)
                    MAX_VEL = 1
                    if np.linalg.norm(np.array(v)) > MAX_VEL:
                        np_v = np.array(v)
                        np_v = MAX_VEL * (np_v / np.linalg.norm(np.array(v)))
                        v = np_v.tolist()

                    drone.vel_control(v[0], v[1], drone.assigned_fp[2])
                else:
                    v = drone.add_attractive_potential(
                        drone.assigned_fp[0:2], 30.0)
                    drone.velocity_control(v, drone.assigned_fp[2])

            except ZeroDivisionError:
                pass

    def sendCommand(self):
        """
        İHA'ları eşleştirilmiş olduklara noktalara öteletmek için tek döngülük komut gönderir. 
        Assigned fpler güncellenmezse hover atılır. 
        """

        # görevi uygula
        for drone in self.mission.drone_list:
            try:
                if drone.link_uri == 0:  # if firefly
                    drone.position_control(
                        drone.assigned_fp[0], drone.assigned_fp[1], drone.assigned_fp[2], 0)

                else:  # if crazyflie
                    drone.send_pose(
                        drone.assigned_fp[0], drone.assigned_fp[1], drone.assigned_fp[2])

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
