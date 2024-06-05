import numpy as np
import rospy
from swarm import formation
import time
from swarm import drone
from swarm import karma_formation as kformation
import datetime

DRONE_ARRIVAL_TOLERANCE = 0.05  # 0.2 idi
VL_ARRIVAL_TOLERANCE = 0.05


class Plan:
    def __init__(self, drone_list, zumo_list, dt):

        self.dt = dt
        self.drone_list = drone_list
        self.zumo_list = zumo_list
        self.robot_list = zumo_list + drone_list
        self.formation = formation.Formation(self)
        self.kformation = kformation.KarmaFormation(self)
        self.avoid = float()  # deneysel

        self.height = 1.0

        # görev indexi
        self.i = 0
        self.initial_heading = float()

        # first run flagler
        self.takeoff_first_run = True
        self.change_formation_first_run = True
        self.rotate_formation_first_run = True
        self.start_time = time.time()
        self.start_thrust = time.time()
        self.first_run_wait = True
        self.time_start = 0
        self.start_pwm = time.time()
        self.first_run_pwm = True
        self.first_run_thrust = True
        self.land_asynch_first_run = True
        self.first_run_wait = True
        self.first_form_karma = True
        self.hız_flag = True
        self.hız_süresi = float()
        self.mission_start = 0

        self.first_drone_arrival = False
        self.changeFormation_timer = 0

    def swarm_control(self):
        i = 0

        while i < len(self.drone_list):
            if self.drone_list[i].link_uri != 0:
                # bu olmazsa drone.connected dene
                if not self.drone_list[i].is_connected:
                    print(self.drone_list[i].link_uri, "failed to connect")
                    self.drone_list.remove(self.drone_list[i])
                    print("-----------------listeden kaldırıldı--------------------")
                    self.robot_list = self.zumo_list + self.drone_list

                    print("formasyon noktaları", self.formation.fp_list)
                    # self.formation.fp_list = self.formation.assignment(
                    #    self.formation.fp_list)
                    self.formation.updateFormation()

            if abs(self.drone_list[i].current_roll) > 60 or abs(self.drone_list[i].current_pitch) > 60:
                self.drone_list.remove(self.drone_list[i])
                print("-----------------listeden kaldırıldı--------------------")
                self.robot_list = self.zumo_list + self.drone_list              
                self.formation.updateFormation()
                
            else:
                break

            
            #print(f"roll: {self.drone_list[i].current_roll} pitch: {self.drone_list[i].current_pitch}")
            i += 1

    def _droneArrival_control(self):
        """
        Her bir İHA'nın eşleşmiş oldukları formasyon noktalarına varıp varmadıklarını kontrol eder.
        Vardılarsa true değeri dönülür.
        """

        arrival = True
        # dronlardan bir tanesi bile vardıysa true olur ve timer tutar

        for drone in self.drone_list:
            pos_vec = np.array(
                [drone.current_position_x, drone.current_position_y, drone.current_position_z])
            target_vec = np.array(drone.assigned_fp)

            if np.linalg.norm(target_vec - pos_vec) > DRONE_ARRIVAL_TOLERANCE:
                arrival = False

            else:  # varmışsa ilk time tut
                if self.first_drone_arrival == False:
                    self.first_drone_arrival = True
                    self.changeFormation_timer = time.time()
                    return False  # diğer fonksiyona geçmesin diye false döndük

        if (time.time() - self.changeFormation_timer > 5) and self.first_drone_arrival:
            arrival = True
            i = 0

            while i < len(self.drone_list):
                pos_vec = np.array(
                    [self.drone_list[i].current_position_x, self.drone_list[i].current_position_y, self.drone_list[i].current_position_z])
                target_vec = np.array(self.drone_list[i].assigned_fp)

                if np.linalg.norm(target_vec - pos_vec) > DRONE_ARRIVAL_TOLERANCE:

                    self.drone_list.remove(self.drone_list[i])
                    print("***************listeden kaldırıldı************************")
                    self.robot_list = self.zumo_list + self.drone_list
                    """self.formation.findFormationPoints(self)
                    self.formation.assignment(self, formation_points, assignment=True)"""
                    self.formation.updateFormation()

                else:
                    i += 1

        if arrival:
            self.first_drone_arrival = False

        return arrival

    def _droneArrival(self):
        """
        Her bir İHA'nın eşleşmiş oldukları formasyon noktalarına varıp varmadıklarını kontrol eder.
        Vardılarsa true değeri dönülür.
        """
        arrival = True

        for drone in self.drone_list:
            pos_vec = np.array(
                [drone.current_position_x, drone.current_position_y, drone.current_position_z])
            target_vec = np.array(drone.assigned_fp)

            if np.linalg.norm(target_vec - pos_vec) > DRONE_ARRIVAL_TOLERANCE:
                arrival = False

        return arrival

    def _virtualLeadArrival(self, target_pos_vec):
        if (np.linalg.norm(np.array(target_pos_vec) - np.array(self.formation.virtual_lead_pos[0:3])) > VL_ARRIVAL_TOLERANCE):
            return False
        else:
            return True

    # FORMASYON GÖREVLERİ

    def takeOff(self, takeoff_vel, desired_height):
        """
        desired_height: ihaların varsayılan irtifası
        float: takeoff_vel: ihaların kalkış hızı
        """
        if self.takeoff_first_run:
            # kalkışı init et
            self.height = desired_height
            # fp_list ve vl konumunu init et
            self.formation.initFormation()
            self.takeoff_first_run = False

        target_pos_vec = [self.formation.virtual_lead_pos[0],
                          self.formation.virtual_lead_pos[1],
                          desired_height]

        if not (self._virtualLeadArrival(target_pos_vec)):
            print("KALKIYOOOMMMMMMMMMMMMM")

            # orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - \
                np.array(self.formation.virtual_lead_pos[0:3])
            vel_sp = takeoff_vel * (vel_sp / np.linalg.norm(vel_sp))
            vel_sp = vel_sp.tolist()

            self.formation.updateVirtualLead(0.0, 0.0, vel_sp[2], 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False
        else:
            print("KALKTIIIIIIIIIIIIIIIII")
            self.takeoff_first_run = True
            return True

    def land(self, land_vel):
        """
        float: land_vel: iniş için gerekli hız
        """

        target_pos_vec = [self.formation.virtual_lead_pos[0],
                          self.formation.virtual_lead_pos[1],
                          0.0]

        if not (self._virtualLeadArrival(target_pos_vec)):

            # orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - \
                np.array(self.formation.virtual_lead_pos[0:3])
            vel_sp = land_vel * (vel_sp / np.linalg.norm(vel_sp))
            vel_sp = vel_sp.tolist()

            self.formation.updateVirtualLead(0.0, 0.0, vel_sp[2], 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            print("iniş")
            return False
        else:
            print("bitti ^_^ ")
            for drone in self.drone_list:
                drone.command.send_stop_setpoint()

            return True

    def changeFormation(self, fp_list):

        if self.change_formation_first_run:

            print("formasyon değiştiriliyor...")

            # yeni local fplere göre global fp listesini bul
            self.formation.fp_list = fp_list
            # bu fonksiyon fplist girdisi almasın, fonksiyonlara bu gücü yüklemeyelim, hatta elle girilmek zorunda olsun
            new_gfp_list = self.formation.findFormationPoints()

            # global fp listelerini kullanarak macar çalıştır (local ile mümkün değil bu) macar local
            # ve global listeleri sıralasın, sıralanmış global fpyi de assign etsin
            # local fp listesini bu sıralanmış listeye göre güncelle

            # bu içeride kafasına göre fplisti güncellemesin return değeri ile dışarıda güncelleyelim.
            self.formation.assignmentForFormation(
                new_gfp_list, fp_list, assignment=True)

            self.change_formation_first_run = False

        if not self._droneArrival_control():
            self.formation.sendPotentialCommand()
            return False
        else:
            self.change_formation_first_run = True
            return True

    def moveFormation(self, desired_x, desired_y, move_vel):
        """
        float: land_vel: iniş için gerekli hız
        """

        target_pos_vec = [desired_x, desired_y, self.height]

        if not (self._virtualLeadArrival(target_pos_vec)):

            # orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - \
                np.array(self.formation.virtual_lead_pos[0:3])
            vel_sp = move_vel * (vel_sp / np.linalg.norm(vel_sp))
            vel_sp = vel_sp.tolist()

            self.formation.updateVirtualLead(vel_sp[0], vel_sp[1], 0.0, 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False
        else:
            print("formasyon hedefe vardı")
            # motorları kapat komutu gönder burada
            return True

    def rotateFormation(self, desired_angle, angular_rate):
        """
        forksiyon şu anki haliyle tek yöne dönmeyi destekliyor
        """

        if self.rotate_formation_first_run:
            # kaç derece dönebildiğini bulmak için initial headingi kaydet
            self.initial_heading = self.formation.virtual_lead_pos[3]
            self.rotate_formation_first_run = False
            self.mission_start = datetime.datetime.now()

        if (self.formation.virtual_lead_pos[3] - self.initial_heading) > desired_angle:
            self.rotate_formation_first_run = True

            print("DÖNÜŞTE GEÇEN SÜRE: ",
                  datetime.datetime.now() - self.mission_start)

            return True
        else:
            self.formation.updateVirtualLead(0.0, 0.0, 0.0, angular_rate)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False

    def landAsynch(self, vel, landing_points):
        # yeni formasyon sitemine uygun landing
        # iniş için listeyi düzenle
        if self.land_asynch_first_run:
            self.hover_list = self.drone_list
            self.formation.hover_list = self.bubble_sort(self.drone_list)
            # bunun için algo yaz
            self.formation.fp_list = landing_points
            landing_points_global = self.formation.findFormationPoints(
                vl_pos=[self.formation.virtual_lead_pos[0], self.formation.virtual_lead_pos[1], 0, 0], assignment=False)  # formasyon noktalarını bulur
            # eşleşirme ve atama işlemini yapar.
            self.formation.assignment(landing_points_global, assignment=True)
            for drone in self.formation.hover_list:
                drone.hover_pos = [drone.current_position_x,
                                   drone.current_position_y, drone.current_position_z]

            self.land_asynch_first_run = False

        # haraketi uygula
        try:
            cond = self.hover_list[0].land_single(
                vel)  # sırası gelen dronu indir

            if cond:
                self.hover_list.remove(self.hover_list[0])
            # kendine denk geldiysen diğer iterasyona geç

            for drone in self.hover_list:

                if self.hover_list[0] == drone:
                    continue

                # sırası olmayanlara hover gönder
                if drone.link_uri == 0:
                    drone.position_control(
                        drone.hover_pos[0], drone.hover_pos[1], drone.hover_pos[2], 0)

                else:
                    drone.send_pose(
                        drone.hover_pos[0], drone.hover_pos[1], drone.hover_pos[2])

        except ZeroDivisionError:
            pass

            # haraketi kontrol et
            if len(self.hover_list) == 1:
                return True
            else:  # b == 0
                return False

    def preceed(self, plan_list):
        """
        döngü içine konulmasi lazim.
        plan_list : elemanlari fonksiyon olan liste: [görev1(), görev2()]
        """
        try:
            # print("drone sayısı", len(self.drone_list))
            a = plan_list[self.i]()
            last_time = time.time()
            last_thrust = time.time()
            last_pwm = time.time()

            if last_thrust - self.start_thrust > 5:
                self.first_run_thrust = False

            if last_pwm - self.start_pwm > 5:
                self.first_run_pwm = False

            if last_time - self.start_time > 1000:
                k = last_time - self.start_time
                print(k)
                self.i += 1

                self.start_time = time.time()
                print(self.i)

            else:
                # normal haraket
                if type(a) is bool:
                    if a:
                        self.i += 1
                    if self.i > len(plan_list) + 1:
                        rospy.on_shutdown()

            return True
        except IndexError:
            return False

    def waitAtFP(self, duration):
        """
        görevlerden sonra İHA'lar mevcut noktalarına yetişebilsinler diye eklendi. 
        self.time_start ve self.first_run_wait normai wait ile aynı değişkenler ama fonk
        kendini sıfırladığı için ekstadan değişken açmadım.
        """
        print("hold")
        if self.first_run_wait:
            for drone in self.drone_list:
                drone.hover_pos = [drone.current_position_x,
                                   drone.current_position_y, drone.current_position_z]

            self.first_run_wait = False
            self.time_start = time.time()

        if time.time() < self.time_start + duration:
            for drone in self.drone_list:
                if drone.link_uri == 0:
                    drone.position_control(
                        drone.assigned_fp[0], drone.assigned_fp[1], drone.assigned_fp[2], 0)
                else:
                    drone.send_pose(
                        drone.assigned_fp[0], drone.assigned_fp[1], drone.assigned_fp[2])
            return False
        if time.time() >= self.time_start + duration:
            self.first_run_wait = True
            return True

    def wait(self, duration):
        """
        """

        if self.first_run_wait:
            for drone in self.drone_list:
                drone.hover_pos = [drone.current_position_x,
                                   drone.current_position_y, drone.current_position_z]

            self.first_run_wait = False
            self.time_start = time.time()

        if time.time() < self.time_start + duration:
            for drone in self.drone_list:
                if drone.link_uri == 0:
                    drone.position_control(
                        drone.hover_pos[0], drone.hover_pos[1], drone.hover_pos[2], 0)
                else:
                    drone.send_pose(
                        drone.hover_pos[0], drone.hover_pos[1], drone.hover_pos[2])
            return False
        if time.time() >= self.time_start + duration:
            self.first_run_wait = True
            return True

        print("WAIT")

    def bubble_sort(self, our_list):
        # We go through the list as many times as there are elements
        for i in range(len(our_list)):
            # We want the last pair of adjacent elements to be (n-2, n-1)
            for j in range(len(our_list) - 1):
                if our_list[j].current_position_z > our_list[j + 1].current_position_z:
                    # Swap
                    our_list[j], our_list[j + 1] = our_list[j + 1], our_list[j]
        return our_list

    def karma_move(self, desired_x, desired_y, move_vel):
        if self.first_form_karma:
            self.kformation.initFormation()
            self.first_form_karma = False

        target_pos_vec = [desired_x, desired_y,
                          self.kformation.virtual_lead_pos[2]]

        if not (self.kformation._virtualLeadArrival(target_pos_vec[0:2])):

            # orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - \
                np.array(self.kformation.virtual_lead_pos[0:3])
            vel_sp = move_vel * (vel_sp / np.linalg.norm(vel_sp))
            vel_sp = vel_sp.tolist()

            self.kformation.updateVirtualLead(vel_sp[0], vel_sp[1], 0.0)
            self.kformation.findFormationPoints(self.kformation.virtual_lead_pos,
                                                self.kformation.fp_hava_list, self.drone_list, assignment=True)
            self.kformation.findFormationPoints(self.kformation.virtual_lead_pos,
                                                self.kformation.fp_kara_list, self.zumo_list, assignment=True)
            self.kformation.sendCommand()
            return False
        else:
            print("formasyon hedefe vardı")
            # motorları kapat komutu gönder burada
            return True

    # def moveNav(self, avoid):
    #     """
    #     avoid, avoid clasının bir objesi
    #     """
    #     self.avoid = avoid

    #     self.avoid.obstaclesMaps([-10.0, -10.0], [10.0, 10.0])
    #     self.avoid.dagilma_noktasi = [2,2,0,0]
    #     self.avoid.toplanma_noktasi = [7,7,0,0]
    #     self.avoid.Point()
    #     dagilma = self.avoid.assignment()

    def returnFalse(self):
        return False

    def deneme(self):

        if self.hız_flag == True:
            self.hız_süresi = time.time()
            self.hız_flag = False

        hız_start = time.time()

        if (hız_start - self.hız_süresi) < 4:

            for drone in self.drone_list:
                drone.vel_control(0.2, 0, 1)
                print("AGAGAGAGA")

            return False

        else:
            return True
