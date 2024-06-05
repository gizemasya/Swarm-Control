from swarm import drone as d
from swarm import plan as p
from swarm import moveSwarm as dav
from swarm import zumo as z
import numpy as np
import datetime
import rospy
import time
import cflib

cflib.crtp.init_drivers()

rospy.init_node("liteunant")
FREQUENCY = 30
r = rospy.Rate(FREQUENCY)

uri1 = "radio://0/80/2M/E7E7E7E7E7"
uri2 = "radio://0/80/2M/E7E7E7E7C1"

# SANİYE 100, SWARM CONTROL YOK, MOVE HIZI 9

zumo_list = []#[z.Zumo(2, True, 1/FREQUENCY)]
drone_list = [d.Drone(1, link_uri=0, sim=False, pos_offset=[0,0]), d.Drone(2, link_uri=0, sim=False, pos_offset=[0, 0]), d.Drone(3, link_uri=0, sim=False, pos_offset=[0,0]), d.Drone(4, link_uri=0, sim=False, pos_offset=[0, 0]), d.Drone(5, link_uri=0, sim=False, pos_offset=[0, 0]), d.Drone(6, link_uri=0, sim=False, pos_offset=[0, 0])] # SİM FALSE YAP VE ÇOKLU DRONE EKLE. URİ LARI SİL

plan = p.Plan(drone_list, zumo_list, 1/FREQUENCY)

avoid = dav.Avoid(3, 0.1, [], 0.02, plan, 1.0, 1, [-20.0,-20.0], [20.0, 20.0])
    
for drone in drone_list:
    drone.dt = 1 / FREQUENCY


k_fp_list = [[0.25, 0, 0], [-0.25,0, 0]]
k_fp_list = [[0,1, 0], [0.865,0.5, 0],[0.865,-0.5,0],[0,-1,0],[-0.865,-0.5,0],[-0.865,0.5,0]]


d1 = lambda: plan.takeOff(0.5, 2)
d2 = lambda: plan.changeFormation(k_fp_list)
d3 = lambda: avoid.pathAssigment()
d4 = lambda: avoid.moveKarma(0.1,2)
d5 = lambda: plan.formation.initFormation()
#d6 = lambda: plan.wait(3000)
d6 = lambda: plan.land(0.5)

m_list = [d1, d2,d3, d4, d5, d6]

ready = False
time.sleep(2)
while not ready:
    drone_ready = True
    for drone in drone_list:
        if drone.link_uri != 0:
            if not drone.is_connected: # bu olmazsa drone.connected dene
                drone_ready = False
                print(drone.link_uri, "failed to connect")

            if drone.current_battery < 30:
                print(drone.link_uri, "battery lower than %30")
            
            elif drone.current_battery <= 10:
                print(drone.link_uri, "low battery for fly")
                #drone_ready = False

            pos_vec = np.array([drone.current_position_x, drone.current_position_y])
            for i in range(6):
                new_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
                dif = new_pos_vec - pos_vec
                pos_vec = new_pos_vec
                if (np.linalg.norm(dif) > 0.1):
                    drone_ready = False
                time.sleep(0.1)

    if drone_ready:
        ready = drone_ready

    print("bağlanılıyor")
    time.sleep(0.2)

   
if ready:
    # ana döngüyü initialize et
    r = rospy.Rate(FREQUENCY)
    for drone in drone_list:
        drone.dt = 1 / FREQUENCY
    run = True

    time.sleep(2)

    avoid.obstaclesMaps([-20.0,-20.0], [20.0, 20.0], 3) # plot için, cppstar için avoid objesinde veriliyo mapin boyutu
    plan.formation.initFormation()
    avoid.Point_karma(k_fp_list, [plan.formation.virtual_lead_pos[0], plan.formation.virtual_lead_pos[1], 0, 0]) #self.start_points_drone u buluyo, x,y,z li. z si fp listteki z
    avoid.assignment_karma() # start_point drone güncellendi
    avoid.pathDrawProcess()
    avoid.debugPrint([], [], [], 3)

    """ point te sıkıntı yok
        assignmentların ikisi de denenecek
        pathdraw denencek
        """


    while not rospy.is_shutdown() and run:
        """
        b.currentDronePosLog()
        b.virtualPosLog()
        b.assignedFpLog()
        print(len(plan.drone_list))"""
        x = datetime.datetime.now()
        #plan.swarm_control()
        run = plan.preceed(m_list)
        #print("listedeki iha sayısı", len(plan.drone_list))


        #print("orta nokta: ", plan.formation.virtual_lead_pos)
        #r.sleep()
        y = datetime.datetime.now()
        dif = (1/FREQUENCY) - ((y-x).total_seconds())
        if (dif>0):
            time.sleep(dif)
        
        z = datetime.datetime.now()

        #print("periyod", z-x)

