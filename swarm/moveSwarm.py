import numpy as np
import matplotlib.pyplot as plt
from multiprocessing.dummy import Process
from swarm import hungarian
from queue import Queue
import time
from swarm import drone
from swarm import formation
from swarm import zumo
# from zumo import Zumo
import rospy
from build import cppstar # c++ a* dan path ler hesaplanacak
from swarm import sim_camera_V2 as g # yangın koordinatlarının döndürüldüğü kod
import math


class Avoid():
    def __init__(self, kacıncı_gorev, grid_size, threatlist, robot_radius, plan, safe_distance, dh, bottom_left, top_right):

        self.ox = []
        self.oy = []

        self.dh = dh

        self.grid_size = grid_size
        self.threatlist = threatlist
        self.robot_radius = robot_radius
        self.safe_distance = safe_distance
        self.plan = plan
        self.bottom_left = bottom_left
        self.top_right = top_right
        self.yedek_path = []

        self.formation = formation.Formation(self)
        self.drone = drone.Drone(self)

        self.a = 0
        self.sayi = 0
        self.baslangıc = True
        self.start_time = time.time()
        self.start_time_karma = time.time()

        self.takeoff_first_run = True
        self.flag = False

        self.dagilma_noktasi = list()
        self.toplanma_noktasi = list()
        self.back_flag = True
        self.back_drone_time = 0
        self.back_drone_list = []

        self.astar_path_list = [i for i in range(len(self.plan.drone_list))]
        self.astar_path_list_z = [i for i in range(len(self.plan.zumo_list))]

        self.assigned_path_list_drone = [
            i for i in range(len(self.plan.drone_list))]
        self.assigned_path_list_zumo = [
            i for i in range(len(self.plan.zumo_list))]
        self.list = [i for i in range(len(self.plan.drone_list))]
        self.assigned_path_list_karma = []

        """self.fire_points_drone = [[1,-2,0], [1,2,0], [3,3,0]] # görüntü işlemeden gelen fonksiyonla içi dolacak
        self.fire_points_zumo = [[-4,1,0], [1,1,0]]"""

        if kacıncı_gorev == 3:
            self.fire_circum, self.fire_points_drone, self.fire_points_zumo = g.main(
                len(self.plan.drone_list), len(self.plan.zumo_list))

            for i in range(len(self.fire_circum)):
                self.fire_circum[i][0] = self.fire_circum[i][0] / 100
                self.fire_circum[i][1] = self.fire_circum[i][1] / 100
                self.fire_circum[i].append(0.001)

            for i in range(len(self.fire_points_drone)):
                self.fire_points_drone[i][0] = self.fire_points_drone[i][0] / 100
                self.fire_points_drone[i][1] = self.fire_points_drone[i][1] / 100

            for i in range(len(self.fire_points_zumo)):
                self.fire_points_zumo[i][0] = self.fire_points_zumo[i][0] / 100
                self.fire_points_zumo[i][1] = self.fire_points_zumo[i][1] / 100

            print("zumoooo", self.fire_points_zumo)
            
            #print("ÇEVREEEEEEEEEE", self.fire_circum)
            
            self.zumo_flag = True

            self.threat_drone = self.fire_points_drone.copy()

            print("threat droneeeeeee", self.threat_drone)
            print("droneeeeee", self.fire_points_drone)

            for i in range(len(self.threat_drone)):
                self.threat_drone[i] = self.threat_drone[i].copy()
                self.threat_drone[i].append(0.3)

            print("threat droneeeeeee", self.threat_drone)
            print("droneeeeee", self.fire_points_drone)

            self.threat_zumo = [[0.5, 1.5, 0.1]]
            

    def tektencifte(self, tek_liste):
        """
        tek liste içinde bulunan elemanları x ve y listesi içine dağıtır
        """

        x_listesi = []
        y_listesi = []

        for eleman in tek_liste:
            x_listesi.append(eleman[0])
            y_listesi.append(eleman[1])

        return x_listesi, y_listesi

    def ciftenteke(self, x_listesi, y_listesi):
        """
        x ve y listesi içindeki elemanları tek liste içine koyar.
        """

        tek_liste = []
        for x, y in zip(x_listesi, y_listesi):
            tek_liste.append([x, y])

        return tek_liste

    def circle(self, kacıncı_gorev):
        """
        engellerin daire şeklinde oluşmasını sağlayan fonksiyon
        main içine oluşturulacak bir threatliste ihtiyaç duyar.
        threatlist içine x merkez, y merkez ve yarıçap alır
        r = yarıçap
        theta = sin ve cos'un alacağı açı
        """
        if kacıncı_gorev == 2:
            threatlist = self.threatlist

        if kacıncı_gorev == 3:
            threatlist = self.fire_circum + self.threat_drone + self.threat_zumo

        for threat in threatlist:
            theta = np.linspace(0, 2 * np.pi, 1000)
            a = threat[2] * np.cos(theta) + threat[0]
            b = threat[2] * np.sin(theta) + threat[1]

            # buarada for döndürmek yerine ox +a yapılsa olurmuş gibi
            for i in range(0, 1000):
                self.ox.append(a[i])
                self.oy.append(b[i])

    def obstaclesMaps(self, bottom_left_corner, top_right_corner, kacıncı_gorev):
        """
        engellerin mapi, sağ üst köşe ve sol alt köşeyi baz alıyor
        top_right_corner = [x, y]
        bottom_left_corner = [x, y]
        threatlist = [x, y ,r]
        """

        self.ox = []
        self.oy = []

        # alt yatay sınır
        for i in np.arange(bottom_left_corner[0], top_right_corner[0], self.grid_size):
            self.ox.append(float(i))
            self.oy.append(bottom_left_corner[1])
        # üst yatay sınır
        for i in np.arange(bottom_left_corner[0], top_right_corner[0], self.grid_size):
            self.ox.append(float(i))
            self.oy.append(top_right_corner[1])
            # alt yatay sınır
        for i in np.arange(bottom_left_corner[1], top_right_corner[1], self.grid_size):
            self.ox.append(bottom_left_corner[0])
            self.oy.append(float(i))
            # üst yatay sınır
        for i in np.arange(bottom_left_corner[1], top_right_corner[1], self.grid_size):
            self.ox.append(top_right_corner[0])
            self.oy.append(float(i))

        self.ox.append(top_right_corner[0])
        self.oy.append(top_right_corner[1])

        self.circle(kacıncı_gorev)

        # grid size harita büyüklüğüne bakılarak setlenebilir

    def assignment(self):
        """
        start ve end pointler için atama
        """

        print("Atama gerçekleştiriliyor...")

        start_point = self.dagilma_gfp
        end_point = self.birlesme_gfp

        cost_matrix = []
        global_fp_list = []

        for drone in end_point:
            for fp in start_point:
                drone_pos = np.array(drone)
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(
            cost_matrix, (len(end_point), len(start_point)))
        # Get the element position.
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in end_point:
            index = end_point.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)
            global_fp_list.append(start_point[index])

        self.dagilma_gfp = global_fp_list

    def assignment_karma(self):
        """
        start ve end pointler için atama
        """

        print("Atama gerçekleştiriliyor...")

        start_point = self.start_points_drone
        end_point = self.fire_points_drone

        cost_matrix = []
        global_fp_list = []

        for drone in end_point:
            for fp in start_point:
                drone_pos = np.array(drone)
                f_pos = np.array(fp[0:2])
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(
            cost_matrix, (len(end_point), len(start_point)))
        # Get the element position.
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in end_point:
            index = end_point.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)
            global_fp_list.append(start_point[index])

        self.start_points_drone = global_fp_list

    def Point(self, fp_list):
        """
        İHA'ların engellerden kaçınmadan önce dağılmalarını engellerden kaçınma süreci
        bittikten sonra tekrar birleşerek formasyon almaları için
        komut gönderir.
        """
        self.formation.fp_list = fp_list

        self.birlesme_gfp = self.formation.findFormationPoints(
            vl_pos=self.toplanma_noktasi)
        self.dagilma_gfp = self.formation.findFormationPoints(
            vl_pos=self.dagilma_noktasi)

        print("dağılma", self.dagilma_gfp)
        print("birleşme", self.birlesme_gfp)

    def Point_karma(self, fp_list, dagilma_noktasi):

        print("vl pos", self.formation.virtual_lead_pos)
        self.formation.fp_list = fp_list
        print("fp noktaları", self.formation.fp_list)

        self.start_points_drone = self.formation.findFormationPoints(
            vl_pos=dagilma_noktasi)
        print("dağılma noktası", self.start_points_drone)

    def calculateStartPoint(self, desired_point):  # DESİRED POİNT X VE Y DE OLACAK
        """
        en yakın engel ve en yakın drone bulunur.
        desired point = [x, y]
        """

        self.formation.initFormation()  # ASSİGNED FP ATAMASIN

        distance_list = []
        fp_mag_list = []

        for threat, fp in zip(self.threatlist, self.formation.fp_list):
            threat_pos_vec = np.array(threat[0:2])
            distance = np.linalg.norm(
                self.formation.virtual_lead_pos[0:2] - threat_pos_vec)
            distance_list.append(distance)

            fp_mag = np.linalg.norm(np.array(fp[0:2]))
            fp_mag_list.append(fp_mag)

        closest_index = distance_list.index(min(distance_list))
        closest_threat = self.threatlist[closest_index]
        formation_radius = max(fp_mag_list)

        mesafe = formation_radius + self.safe_distance

        direction_vector = np.array(
            desired_point) - np.array(self.formation.virtual_lead_pos[0:2])
        vel_vector = direction_vector / np.linalg.norm(direction_vector)
        vel_vector = vel_vector * self.grid_size  # bunu kontrol et

        dagilma_noktasi = self.formation.virtual_lead_pos[0:2]

        print("vel_vector", vel_vector)
        print("vl", dagilma_noktasi)

        for i in range(int(np.linalg.norm(direction_vector) / self.grid_size)):
            distance = np.linalg.norm(
                np.array(dagilma_noktasi) - np.array(closest_threat[0:2]))
            print("distance", distance)
            dagilma_noktasi = dagilma_noktasi + vel_vector

            if distance <= mesafe:
                break
            else:
                continue

        dagilma_noktasi = dagilma_noktasi.tolist()

        dagilma_noktasi.append(0)
        dagilma_noktasi.append(0)

        self.dagilma_noktasi = dagilma_noktasi

    def debugCpp(self, path):
        plt.plot(self.ox, self.oy, ".k", color='black')
        rx, ry = self.tektencifte(path)
        plt.plot(rx, ry, ".k")
        plt.grid(True)
        plt.grid(color='black')
        plt.axis("equal")
        plt.show()

    def debugPrint(self, points, points_2, path_list, kacıncı_gorev):
        """
        bulunan yolu, haritanın sınırlarını ve engelleri plotlar
        aynı zamanda start ve goal noktalarını dagilma ve toplanma içine ikiye ayırır
        rx ve ry'yi de path içinde çift haline getirir

        """
        if kacıncı_gorev == 3:
            drone_start = self.start_points_drone
            fire_zumo = self.fire_points_zumo
            fire_drone = self.fire_points_drone
            threat_drone = self.threat_drone
            threat_zumo = self.threat_zumo
            path_list = self.path_list
            path_list_2 = self.path_list_zumo

        print(len(path_list))

        cx, cy = self.tektencifte(drone_start)
        sx, sy = self.tektencifte(fire_drone)
        gx, gy = self.tektencifte(fire_zumo)
        tx, ty = self.tektencifte(threat_drone)
        bx, by = self.tektencifte(threat_zumo)

        plt.plot(self.ox, self.oy, ".k", color='black') # map size, fire çevresi, engel görünecek droneların çevresi ve zumoların engeli

        plt.plot(sx, sy, ".k", color='blue')
        plt.plot(gx, gy, ".k", color='cyan')
        plt.plot(cx, cy, ".k", color='purple')
        plt.plot(tx, ty, ".k", color='red')
        plt.plot(bx, by, ".k", color='green')

        for path in path_list:
            rx, ry = self.tektencifte(path)
            plt.plot(rx, ry, ".k")

        if kacıncı_gorev == 3:
            for path in path_list_2:
                zx, zy = self.tektencifte(path)
                plt.plot(zx, zy, ".k")

        plt.grid(True)
        plt.grid(color='black')
        plt.axis("equal")
        plt.show()

    def startNavigation(self):
        """
        engellerden kaçınmadan önce dağılma noktasına kadar olan kalkış ve 
        formasyon alma işlemlerini gerçekleştirir.
        """

        try:
            def da(): return self.plan.takeOff(0.2, self.dh)
            def db(): return self.plan.changeFormation(self.fp_list)
            def dc(): return self.plan.moveFormation(
                self.dagilma_noktasi[0], self.dagilma_noktasi[1], 0.2)  # dagilma noktasi
            s_nav_list = [da, db, dc]
            is_arrived = True
            r = rospy.Rate(1/self.plan.dt)

            while is_arrived:
                o = s_nav_list[self.a]()
                last_time = time.time()

                if last_time - self.start_time > 100:
                    k = last_time - self.start_time
                    print("zaman aralığı", k)
                    self.a += 1
                    self.start_time = time.time()

                else:

                    if o:
                        self.a += 1
                        self.start_time = time.time()

                    if self.a > len(s_nav_list) + 1:
                        is_arrived = False

                r.sleep()

        except IndexError:

            print("dağılma noktasına ulaşıldı")

    def startNavigation_karma(self, takeoff_vel, desired_height):
        try:
            def da(): return self.plan.takeOff(takeoff_vel, desired_height)
            def db(): return self.plan.changeFormation(self.fp_list)
            s_nav_list = [da, db]

            is_arrived = True
            r = rospy.Rate(1/self.plan.dt)
            self.a = 0

            while is_arrived:

                o = s_nav_list[self.a]()
                last_time = time.time()

                if last_time - self.start_time > 100:
                    k = last_time - self.start_time
                    self.a += 1
                    self.start_time = time.time()

                else:

                    if o:
                        self.a += 1
                        self.start_time = time.time()
                        print("GÖREV DEĞİŞTİİİİİİİ")

                    if self.a > len(s_nav_list) + 1:
                        is_arrived = False

                r.sleep()

        except IndexError:

            print("start noktasına ulaşıldı")

    def pathPlan(self, toplanma, dagilma, q, kacıncı_gorev):
        """
        astar algoritmasının goal noktalar(rx, ry) hesapladığı ve listelediği fonksiyon
        """
        if kacıncı_gorev == 3:
            if self.zumo_flag: # eğer toplanma drone'a aitse o toplanma lar da engel olarak eklensin
                drone_threats = self.threat_drone.copy()
                print("drone threatttt", drone_threats) # z li olabilir aga 
                print("toplanma", [toplanma[0], toplanma[1]])
                drone_threats.remove([toplanma[0], toplanma[1], 0.3])
                self.threatlist = self.fire_circum + drone_threats
            else: # zumodan gelen toplanmaysa buraya gircek, zumoya sadece ateş engel olarak gidiyor
                self.threatlist = self.fire_circum + self.threat_zumo

            a_star = cppstar.astar(self.threatlist, self.bottom_left, self.top_right,
                                   dagilma, toplanma, self.grid_size, self.robot_radius)
            print("a_star boş gelme:", a_star)
            q.put(a_star)

        if kacıncı_gorev == 2:
            a_star = cppstar.astar(self.threatlist, self.bottom_left, self.top_right,
                                   dagilma, toplanma, self.grid_size, self.robot_radius)
            print("a_star boş gelme:", a_star)
            q.put(a_star)

    def pathDrawProcess(self):
        """
        astar algoritmasının processler içerisinde çalıştığı fonksiyon
        """

        print("aStar Başladı :) ")

        q = Queue()
        qz = Queue()

        self.path_list = []
        self.path_list_zumo = []

        # kuyruk oluşturdum
        queues = [Queue() for _ in range(len(self.plan.drone_list))]
        queues_z = [Queue() for _ in range(len(self.plan.zumo_list))]

        start_points_zumo = []
        for zumo in self.plan.zumo_list:
            start_points_zumo.append([zumo.turtle_pos_x, zumo.turtle_pos_y, 0])

        for toplanma, dagilma, q in zip(self.fire_points_drone, self.start_points_drone, queues):
            self.pathPlan(toplanma, dagilma, q, 3)

        self.zumo_flag = False

        for toplanma, dagilma, qz in zip(self.fire_points_zumo, start_points_zumo, queues_z):
            self.pathPlan(toplanma, dagilma, qz, 3)

        # yolları çek
        for q in queues:
            self.path_list.append(q.get())

        for qz in queues_z:
            self.path_list_zumo.append(qz.get())

        print("DRONEEEE", self.path_list)
        print("ZUMOOOOO", self.path_list_zumo)

        # print(self.path_list) içerisinde drone sayısı kadar path var
        return self.path_list, self.path_list_zumo

        # path_list doğru geliyor

    def pathDrawProcess_2(self):
        print("aStar Başladı :) ")

        q = Queue()
        self.path_list = []

        # kuyruk oluşturdum
        queues = [Queue() for _ in range(len(self.plan.drone_list))]

        # her kuyruk için yol planlama
        processes = []

        for toplanma, dagilma, q in zip(self.birlesme_gfp, self.dagilma_gfp, queues):

            a_star_process = Process(
                target=self.pathPlan, args=(toplanma, dagilma, q, 2))
            a_star_process.start()
            processes.append(a_star_process)

        for a_star_process in processes:
            a_star_process.join()

        # yolları çek
        for q in queues:
            self.path_list.append(q.get())

        # print(self.path_list) içerisinde drone sayısı kadar path var
        return self.path_list

    def calculatePath(self):

        # dağılma noktaso sx, sy ile hedef nokta arasında yol çiz
        astar = Process(target=self.pathDrawProcess_2)
        move = Process(target=self.startNavigation)

        astar.start()
        move.start()

        astar.join()
        move.join()

        return True

    def calculatePath_karma(self, takeoff_vel, desired_height):
        self.take_off = desired_height
        # bu fonk ateşe giden yolu çiziyor
        astar_karma = Process(target=self.pathDrawProcess)
        # bu fonk takeOff ve changeFormation lambda alacak ve drone lar kalkıp formasyon alacak.
        move_karma = Process(target=self.startNavigation_karma,
                             args=(takeoff_vel, desired_height))

        astar_karma.start()
        move_karma.start()

        astar_karma.join()
        move_karma.join()

        return True

    def pathAssigment(self):
        """
        astar algoritmasının bulduğu yolları İHA'larla eşler
        """
        # dronelara verilen yollar listeden çıkmalı path_mag hesaplarken aynı yolu atıyo
        # del self.path_list[index] yapılırsa bu sefer path_list index sayısı azalıyo astar_path_liste yanlış atıyo
        cost_matrix = []

        for drone in self.plan.drone_list:  # önceden drone eksilirse patlar gibi , o an da eksilirse patlar gibi, yol bulmadığında neden çalıştıralım ki
            for path in self.path_list:
                path_vec = np.array(
                    path[0]) - np.array([drone.current_position_x, drone.current_position_y])
                path_mag = np.linalg.norm(path_vec)
                cost_matrix.append(path_mag)

        cost_matrix = np.reshape(
            cost_matrix, (len(self.plan.drone_list), len(self.path_list)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)
        print(cost_matrix)
        print(ans_pos)

        for ans in ans_pos:
            self.astar_path_list[ans[0]] = self.path_list[ans[1]]

        if len(self.plan.zumo_list) != 0:
            cost_matrix_z = []

            for zumo in self.plan.zumo_list:
                for path in self.path_list_zumo:
                    path_vec = np.array(
                        path[0]) - np.array([zumo.turtle_pos_x, zumo.turtle_pos_y]) # AD DEĞİŞİRİLECEK
                    path_mag = np.linalg.norm(path_vec)
                    cost_matrix_z.append(path_mag)

            cost_matrix_z = np.reshape(
                cost_matrix_z, (len(self.plan.zumo_list), len(self.path_list_zumo)))
            ans_pos = hungarian.hungarian_algorithm(cost_matrix_z.copy())
            ans, ans_mat = hungarian.ans_calculation(cost_matrix_z,
                                                     ans_pos)

            for ans in ans_pos:
                self.astar_path_list_z[ans[0]] = self.path_list_zumo[ans[1]]

        self.assigned_path_list_karma = self.astar_path_list_z + self.astar_path_list
        return True

    def pathAssignment_2(self):

        cost_matrix = []

        for drone in self.plan.drone_list:
            for path in self.path_list:
                path_vec = np.array(
                    path[0]) - np.array([drone.current_position_x, drone.current_position_y])
                path_mag = np.linalg.norm(path_vec)
                cost_matrix.append(path_mag)

        cost_matrix = np.reshape(
            cost_matrix, (len(self.plan.drone_list), len(self.path_list)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)
        print(cost_matrix)
        print(ans_pos)

        for ans in ans_pos:
            self.astar_path_list[ans[0]] = self.path_list[ans[1]]

        print(len(self.astar_path_list), "BBBB")

        return True

    def moveAstar(self):
        """"
        astar tarafından çizilen yolun İHA'lar tarafından takip edilebilmesi için gereken
        vektör ve şart işlemleri burada yapılır
        """
        flag = True
        self.v_des = []

        if self.back_flag:

            back_vec = np.array([self.toplanma_noktasi[0], self.toplanma_noktasi[1]]) - np.array([self.dagilma_noktasi[0], self.dagilma_noktasi[1]])
            theta = (math.atan2(back_vec[1], back_vec[0]))*180/math.pi
            if theta < 0.0:
                theta = theta + 360
            theta_ref = 90 - theta  # y eksenine göre referans
            print("formasyon baş ", theta_ref)
            if theta_ref < 0.0:
                theta_ref = theta_ref + 360
            print("formasyon baş açı ", theta_ref)

            for path, drone in zip(self.astar_path_list, self.plan.drone_list):
                print(f"drone{drone.ns}:bbbbb", [path[0]])
                back_vec = np.array(
                    path[0]) - np.array([self.dagilma_noktasi[0], self.dagilma_noktasi[1]])
                theta = (math.atan2(back_vec[1], back_vec[0]))*(180/math.pi)
                print(f"drone{drone.ns}aaaaaa:", theta)
                if theta < 0.0:
                    theta = theta + 360
                print(theta, "Theta")
                theta_drone = 90 - theta  # y eksenine göre drone açısı
                if theta_drone < 0.0:
                    theta_drone = theta_drone + 360
                # print(f"drone{drone.ns}:", theta_drone)
                fark = theta_drone - theta_ref
                if fark < 0.0:
                    fark = fark + 360
                print(f"drone{drone.ns} fark: ", fark)
                if fark < 90.0 or fark > 270.0:
                    pass
                else:
                    self.back_drone_list.append(drone)
                    # print(f"drone{drone.ns} bekleyecek")

            print(self.back_drone_list, "liste")

            self.back_drone_time = time.time()
            self.back_flag = False
        
        Final_flag = True 
        print("drone len", len(self.plan.drone_list), "path len", len(self.astar_path_list))
        for drone, path in zip(self.plan.drone_list, self.astar_path_list):
            Flag = False
            
            if len(path) - drone.path_i > 3:
                Final_flag = False
                
            if drone.path_i < len(path):
                astar_array = np.array(
                    (path[drone.path_i])) - np.array([drone.current_position_x, drone.current_position_y])
                astar_vector = np.linalg.norm(astar_array)
                self.constant_vec = (astar_array / astar_vector) * 0.1
                self.constant_vec = self.constant_vec.tolist()
                time_dif = time.time()
                #print(f"iha{drone.ns} kalan nokta: {len(path) - drone.path_i} hata: {astar_vector}")
                if time_dif - self.back_drone_time < 2:
                    for back_drone in self.back_drone_list:
                        if back_drone.ns == drone.ns:
                            self.v_des.append((0, 0.01))
                            Flag = True
                            break
                        else:
                            pass
                    if Flag:
                        continue
                    
                self.v_des.append(self.constant_vec)
                if astar_vector < 0.04:
                    drone.path_i += 1
                print(f"drone{drone.ns}: ",len(path)- drone.path_i, "hata vektörü: ", astar_vector)
                flag = False
                
                if astar_vector > 0.04:  # ilk path_i'ye varmamışsa varmadığı andan itibaren zaman tutulmaya başlanıyo
                    if drone.first_run_2 == False:
                        print("time başladı")
                        drone.first_arrival_time_2 = time.time()
                        drone.first_run_2 = True

                else:
                    drone.first_arrival_time_2 = time.time()
                    drone.first_run_2 = False

                if time.time() - drone.first_arrival_time_2 > 8 and drone.first_run_2:
                    if astar_vector > 0.06:  # eğer bir drone bile varmış ve bundan 10 saniye geçmişse ve hala 5cm yaklaşamamışsa listeden çıkar ve flag true olsun
                        self.plan.drone_list.remove(drone)
                        self.astar_path_list.remove(path)
                        print("drone listeden kaldırıldı")
                        print(len(self.plan.drone_list), "drone_list")

            else :
                self.v_des.append((0,0.01))
        
            
        print(" ")
        if Final_flag:
            flag = True
            self.plan.formation.virtual_lead_pos = [self.toplanma_noktasi[0], self.toplanma_noktasi[1], self.dh,0]
        else: 
            self.RVO(self.v_des)
        return flag
    
    def moveKarma(self, move_vel, desired_height):
        """dronelar + kara araclarının birlikte hareketi için"""
        flag = True
        print("move karma ")
        for robot, path in zip(self.plan.robot_list, self.assigned_path_list_karma): # öncesinde drone eksilmesi sorun değil, o sırada olursa sıkıntı
            try:
                if type(robot) == zumo.Zumo and len(self.plan.zumo_list) != 0:
                    if robot.path_i_z < len(path):
                        zumo_array = np.array(
                            (path[robot.path_i_z]) - np.array([robot.turtle_pos_x, robot.turtle_pos_y]))
                        zumo_vector = np.linalg.norm(zumo_array)

                        if type(robot.uri) == str:
                            robot.move(path[robot.path_i_z][0], path[robot.path_i_z][1])

                        else:
                            robot.move_turtle(
                                path[robot.path_i_z][0], path[robot.path_i_z][1])

                        if zumo_vector <= 0.3:
                            robot.path_i_z += 1

                        flag = False

                    else:
                        robot.stop_little_turtle()

                if type(robot) == drone.Drone:
                    if robot.path_i_d < len(path):
                        print("vec hesap")
                        drone_array = np.array(
                            (path[robot.path_i_d]) - np.array([robot.current_position_x, robot.current_position_y]))
                        drone_vector = np.linalg.norm(drone_array)
                        drone_constant_vec = (
                            drone_array / drone_vector) * move_vel  # 0.02
                        drone_constant_vec = drone_constant_vec.tolist()

                        if type(robot.link_uri) == str:  # eğer gerçekse
                            robot.vel_control(drone_constant_vec[0], drone_constant_vec[1], desired_height)
                            #robot.send_pose(
                                #path[robot.path_i_d][0], path[robot.path_i_d][1], desired_height)
                            # robot.send_pose(drone_constant_vec[0], drone_constant_vec[1], desired_height)
                            # robot.velocity_control(drone_constant_vec, desired_height)

                        else:  # eğer simülasyonsa
                            print("takip")
                            robot.velocity_control(
                                drone_constant_vec, desired_height)

                        if drone_vector < 0.1: #burdaki değer 834 deki ile aynı olcak ( if time' lı )
                            robot.path_i_d += 1

                        if drone_vector > 0.1: # ilk path_i'ye varmamışsa varmadığı andan itibaren zaman tutulmaya başlanıyo
                            if robot.first_run == False:
                                print("time başladı")
                                robot.first_arrival_time = time.time()
                                robot.first_run = True
                        
                        else: 
                            robot.first_arrival_time = time.time()
                            robot.first_run = False

                        print("zaman", time.time() - robot.first_arrival_time)
                        print(f"drone{robot.ns}: ", drone_vector)
                        print(f"drone{robot.ns}: ", path[0])

                        if time.time() - robot.first_arrival_time > 60 and robot.first_run:
                                if drone_vector > 0.1: # eğer bir drone bile varmış ve bundan 10 saniye geçmişse ve hala 5cm yaklaşamamışsa listeden çıkar ve flag true olsun
                                    self.plan.drone_list.remove(robot)
                                    self.plan.robot_list.remove(robot)
                                    self.assigned_path_list_karma.remove(path)
                                    print("drone listeden kaldırıldı")


                        flag = False # eğer bir tanesinde hata alırsak time geçtikten sonra true lansın

                    else: # varmışsa buraya giriyor ve time tutmaya başlanıyo
                        print("dur")

                        """if self.first_run == False:
                            self.first_arrival_time = time.time()
                            self.first_run == True"""

                        if robot.link_uri == 0:
                            robot.position_control(
                                path[-1][0], path[-1][1], desired_height, 0)

                        else:
                            robot.send_pose(
                                path[-1][0], path[-1][1], desired_height)
                            
                    """if time.time() - self.first_arrival_time > 10 and self.first_run:
                        i = 0
                        while i < len(self.plan.drone_list):
                            if drone_vector > 0.05: # eğer bir drone bile varmış ve bundan 10 saniye geçmişse ve hala 5cm yaklaşamamışsa listeden çıkar ve flag true olsun
                                self.plan.drone_list.remove(self.plan.drone_list[i])
                                print("drone listeden kaldırıldı")
                                self.first_run = False
                            else:
                                i += 1"""                        

                    # else: #gerçekse ## vardı mı varmadı mı kontrolü yok!! eğer current dan çekebiliyosak else e gerek yok if in içine ifli yaz
                        # robot.send_pose(path[robot.path_i_k][0], path[robot.path_i_k][1], path[robot.path_i_k][2])
                        # robot.path_i_k += 1

            except ZeroDivisionError:
                pass

        return flag
