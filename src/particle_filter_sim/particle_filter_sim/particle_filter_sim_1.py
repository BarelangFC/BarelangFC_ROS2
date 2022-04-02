#!/usr/bin/env python3
from scipy.spatial import distance
from numpy.random import uniform, normal
import time
import sys
import numpy as np
import scipy.stats
import math
import cv2
import webbrowser
import os
import threading
import rclpy
from rclpy.node import Node

from main_interface.msg import VisionGoal
from main_interface.msg import VisionLcross
from main_interface.msg import VisionXcross
from main_interface.msg import VisionTcross
from main_interface.msg import HeadCommand
from main_interface.msg import Angle
from main_interface.msg import RobotPos
from main_interface.msg import TotalDetectLandMarks
from main_interface.msg import BallDistance

# Definisi ID robot
robotID = 1

# Set False for real localization
simulationMode = False
headingFromIMU = True

# Configuration in Cm
fieldLength = 900
fieldWidth = 600

# Particles and landmarks
#totalParticles = 100
totalParticles = 80
totalLandmarks = 8 #7#2
deltaTime = 0.5 #1 #update every "deltaTime" second

mapImage = np.zeros((800,1100,3), np.uint8)

# Landmarks position 2D array
landmarksPosition = np.zeros((totalLandmarks, 2))
drawLandmarksPosition = np.zeros((totalLandmarks,2))

# Particles position
particlesGlobalPosition = np.zeros((totalParticles, 3))
particlesLocalPosition = np.zeros((totalParticles, 3))
particlesInitialPosition = np.zeros((totalParticles, 3))

# Estimate position
estimatePosition = np.zeros((3))
estimateInitialPosition = np.zeros((3))
estimateLocalPosition = np.zeros((3))
ballEstimatePosition = np.zeros((2))

distanceRobotToLandmarks = np.zeros((totalLandmarks))
distanceParticlesToLandmarks = np.zeros((totalParticles, totalLandmarks))
particlesWeight= np.zeros((totalParticles))

velFromKinematic = np.zeros((3))

realVelocity = np.zeros([3])


totalDetectLandmarks = 0


									
#app = Flask(__name__)
# http://mattrichardson.com/Raspberry-Pi-Flask/
#@app.route('/')
#def index():
#    """Video streaming home page."""
#    templateData = { 
#            'robotid' : str(robotID),
#        }
#    return render_template('index.html', **templateData)
									
#@app.route('/video_feed')
#def video_feed():
#    """Video streaming route. Put this in the src attribute of an img tag."""
#    return Response(main(), mimetype='multipart/x-mixed-replace; boundary=frame')

class particleFilterSim(Node):

    def __init__(self):
        # Global Variable
        self.arahGoal = np.zeros((0))
        self.robotGlobalPosition = np.zeros((3))
        self.robotInitialPosition = np.zeros((3))
        self.robotLocalPosition = np.zeros((3))
        self.odometryPosition = np.zeros((2))
        self.ballCoorToMain = np.zeros((2))
        self.robotCoorToMain = np.zeros((2))
        super().__init__("particle_filter_sim")
        self.robot_pos_ = self.create_subscription(
            RobotPos, "robot_pos", self.robot_pos, 10)
        self.get_logger().info("Barelang_particle_filter_sim started")

        self.total_detect_ = self.create_subscription(
            TotalDetectLandMarks, "robot_total_LM", self.total_detect_land_marks, 10)

        self.angle_sub_ = self.create_subscription(
            Angle, "robot_angle", self.angle, 10)

        self.head_pan_sub_ = self.create_subscription(
            HeadCommand, "robot_head", self.head_pan, 10)

    def robot_pos(self, msg):
        self.get_logger().info("robot_pos_ has been started")

    def total_detect_land_marks(self, msg):
        self.totalDetectLandmarks = msg.total
        self.main_program()

    def angle(self, msg):
        self.angle_ = msg.yaw

    def head_pan(self, msg):
        self.head_pan_ = msg.pan

    def goal_side(self):
        self.arah_pandang_ = self.angle_ - (57.29 * self.head_pan_)
        if (self.arah_pandang_ >= -90 and self.arah_pandang_):
            self.goal_side_ = 0
        else:
            self.goal_side_ = 1
        return self.goal_side_

    def goal_coor(self, msg):
        self.goal_ld_ = msg.ld
        self.goal_rd_ = msg.rd

    def lcross_coor(self, msg):
        self.lcross_ld_ = msg.ld
        self.lcross_rd_ = msg.rd

    def xcross_coor(self, msg):
        self.xcross_ld_ = msg.ld
        self.xcross_rd_ = msg.rd

    def tcross_coor(self, msg):
        self.tcross_ld_ = msg.ld
        self.tcross_rd_ = msg.rd

    def threadLocalization(self):
        os.system("./mjpgLocalization.sh")

    # Convert robot velocity to cm/s
    # Loaded from calibrated value
    def convertVel(self, robotId, inputVel):
        outputVel = np.zeros((3))
        if self.robotId == 1:
            if self.inputVel[0] == 0:
                outputVel[0] = 0
            else:
                outputVel[0] = 355.27 * inputVel[0] - 0.4811

            if self.inputVel[1] == 0:
                outputVel[1] = 0
            else:
                outputVel[1] = 389.51 * inputVel[1] + 0.1839

            if self.inputVel[2] == 0:
                outputVel[2] = 0
            else:
                outputVel[2] = 124.78 * inputVel[2] + 1.366
        elif self.robotId == 3:				#JIKA ROBOT 1
            if self.inputVel[0] == 0:			#JIKA INPUTVEL[0] == 0
                outputVel[0] = 0			#OUTPUT[0] == 0
            else:							#SELAIN
                outputVel[0] = 355.27 * inputVel[0] - 0.4811	#OUTPUT[0] = RUMUS REGRESI X * INPUT

            if self.inputVel[1] == 0:   			#JIKA INPUTVEL[1] == 0
                outputVel[1] = 0			#OUTPUT[1] == 0
            else:							#SELAIN
                outputVel[1] = 389.51 * inputVel[1] + 0.1839	#OUTPUT[1] = RUMUS REGRESI Y * INPUT

            if self.inputVel[2] == 0:			#JIKA INPUTVEL[2] == 0
                outputVel[2] = 0			#OUTPUT[2] == 0
            else:							#SELAIN
                outputVel[2] = 124.78 * inputVel[2] + 1.366		#OUTPUT[2] = RUMUS REGRESI THETHA

        elif self.robotId == 5:
            if self.inputVel[0] == 0:
                outputVel[0] = 0
            else:
                outputVel[0] = 315.95 * inputVel[0] - 0.5579

            if self.inputVel[1] == 0:
                outputVel[1] = 0
            else:
                outputVel[1] = 338.71 * inputVel[1] + 0.9102

            if self.inputVel[2] == 0:
                outputVel[2] = 0
            else:
                outputVel[2] = 131.66 * inputVel[2] + 0.9137
        return self.outputVel

    def worldCoorToImageCoor(self, x, y):		#FUNGSI DRAW KOORDINAT SUDUT - SUDUT LAPANGAN JIKA TIDAK PAKE FILE MAPIMAGE.JPG
        self.x = x + 100				#OUTPUT X = X + 100 AKAN BERGESER KE KANAN + 100 PIXEL
        self.y = 800 - (y + 100)			#OUTPUT Y = 800 - (Y + 100) FLIP MODE DRAW
        return self.x, self.y				#RETURN HASIL

    def sendToMain(self):
        self.robotCoorToMain[0] = estimatePosition[0] - 450
        self.robotCoorToMain[1] = 300 - estimatePosition[1]
        self.ballCoorToMain[0] = ballEstimatePosition[0] - 450
        self.ballCoorToMain[1] = 300 - ballEstimatePosition[1] 

    def main_program(self):
        print(self.totalDetectLandmarks)
        # Set initial location of robot, Just for simulation
        self.robotInitialPosition[0] = 450 # X
        self.robotInitialPosition[1] = 300 # Y
        self.robotInitialPosition[2] = 0   # Heading
        self.robotGlobalPosition[:] = 0		# DEKLARASI ARRAY DENGAN NILAI 0 TANPA BATAS UNTUK GLOBAL POSITION
        self.robotLocalPosition[:] = 0		# DEKLARASI ARRAY DENGAN NILAI 0 TANPA BATAS UNTUK LOCAL POSITION
        self.odometryPosition[0] = 0
        self.odometryPosition[1] = 0


        # Initialize landmark position (left and right goal pole)
        landmarksPosition[0,0] = 900	#GAWANG         RG      
        landmarksPosition[0,1] = 430	#GAWANG         RG
        landmarksPosition[1,0] = 900	#GAWANG         LG
        landmarksPosition[1,1] = 170	#GAWANG         LG
        landmarksPosition[2,0] = 800        #LCROSS         RIGHT
        landmarksPosition[2,1] = 550        #LCROSS         RIGHT
        landmarksPosition[3,0] = 800        #LCROSS         LEFT
        landmarksPosition[3,1] = 50         #LCROSS         LEFT
        #landmarksPosition[4,0] = 750        #PENALTY
        #landmarksPosition[4,1] = 300        #PENALTY
        #landmarksPosition[5,0] = 450        #XCROSS         RIGHT
        #landmarksPosition[5,1] = 375        #XCROSS         RIGHT
        landmarksPosition[4,0] = 450        #XCROSS         RIGHT
        landmarksPosition[4,1] = 375        #XCROSS         RIGHT
        landmarksPosition[5,0] = 450        #XCROSS         LEFT
        landmarksPosition[5,1] = 225        #XCROSS         LEFT
        #landmarksPosition[7,0] = 450        #TCROSS         RIGHT
        #landmarksPosition[7,1] = 600        #TCROSS         RIGHT
        landmarksPosition[6,0] = 450        #TCROSS         RIGHT
        landmarksPosition[6,1] = 600        #TCROSS         RIGHT
        #landmarksPosition[8,0] = 450        #TCROSS         LEFT
        #landmarksPosition[8,1] = 0          #TCROSS         LEFT
        landmarksPosition[7,0] = 450        #TCROSS         LEFT
        landmarksPosition[7,1] = 0          #TCROSS         LEFT


        velFromKinematic[0] = 0.00		#INIT VALUE WALK X
        velFromKinematic[1] = 0.00		#INIT VALUE WALK Y
        velFromKinematic[2] = 0.00		#INIT VALUE WALK A

        imuInitHeading = 0			#INIT VALUE HEADING
        imuCurrentHeading = 0		#INIT VALUE HEADING SEBELUMNYA

        arahGoal = 0			#Arah gawang lawan (0) gawang tim (1)
        ballDistance = 0			#INIT VALUE JARAK BOLA
        panAngle = 0			#INIT VALUE SUDUT SERVO PAN

        posRobotX = 0
        posRobotY = 0

        startxDraw = 0
        startyDraw = 0

        endxDraw = 0
        endyDraw = 0

        endxDrawOm = 0
        endyDrawOm = 0

        deltaxDraw = 0
        deltayDraw = 0

        deltaxDrawOm = 0
        deltayDrawOm = 0

        setting = 0

        phi = 3.1428571428571428571428571428571
        defineInitialPosition = True
        if defineInitialPosition == True:
            # Create 90 percent random particles from defined initial position and 10 percent from random uniform
            estimateInitialPosition[0] = 450 # X
            estimateInitialPosition[1] = 300 # Y
            estimateInitialPosition[2] = 0 # Heading
            #estimateInitialPosition[2] = 120 # Heading utara
            #estimateInitialPosition[2] = 300 # Heading selatan

            _10PercentParticle = int(totalParticles * 0.1)

            for i in range (0, _10PercentParticle):
                particlesInitialPosition[i,0] = uniform(0, fieldLength)
                particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                particlesInitialPosition[i,2] = uniform(0, 360)
                particlesGlobalPosition[i,:] = 0
                particlesLocalPosition[i,:] = 0

            _90PercentParticle = totalParticles - _10PercentParticle

            for i in range (_10PercentParticle+1, totalParticles):
                particlesInitialPosition[i,0] = normal(estimateInitialPosition[0], 30)
                particlesInitialPosition[i,1] = normal(estimateInitialPosition[1], 30)
                particlesInitialPosition[i,2] = normal(estimateInitialPosition[2], 10)
                particlesGlobalPosition[i,:] = 0
                particlesLocalPosition[i,:] = 0
        else:
            # Create random uniform position of particles
            particlesInitialPosition[:,0] = uniform(0, fieldLength, size=totalParticles)
            particlesInitialPosition[:,1] = uniform(0, fieldWidth, size=totalParticles)
            particlesInitialPosition[:,2] = uniform(0, 360, size=totalParticles)

        # Zero all global and local position of particles
        particlesGlobalPosition[:,:] = 0
        particlesLocalPosition[:,:] = 0

        # Timing value
        nowTime = 0
        lastTime = 0
        loop = 0

        #pthreadLocalization = threading.Thread(target=threadLocalization)
        #pthreadLocalization.start()

        # while True:
        nowTime = time.perf_counter()
        timer = nowTime - lastTime
        halfDeltaTime = deltaTime / 2.00
        # Update every 0.5 * deltatime
        if timer > halfDeltaTime:
            lastTime = nowTime
            loop += 1
            #print ('Runtime : {} s'.format(deltaTime*loop))

            mapFromFile = False
            if mapFromFile == True:
                # image tidak clear
                mapImage[:] = cv2.imread('mapImage.jpg')
                    
            else:
                mapImage[:] = (0, 255, 0)
                cv2.rectangle(mapImage, (100,100), (1000,700), (255,255,255), 3) # Garis Luar
                cv2.rectangle(mapImage, (40,530), (100,270), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (1000,530), (1060,270), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (100,650), (200,150), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.rectangle(mapImage, (900,650), (1000,150), (255,255,255), 3) # Garis Luar Gawang Kiri
                cv2.line(mapImage, (550,100), (550,700), (255,255,255), 3) # Garis Tengah
                cv2.circle(mapImage, (550,400), 75, (255,255,255), 3) # Lingkaran Tengah
                cv2.circle(mapImage, (250,400), 3, (255,255,255), 5)
                cv2.circle(mapImage, (850,400), 3, (255,255,255), 5)

            showGrid = True
            if showGrid == True:
                cv2.line(mapImage, (100,200), (1000,200), (0,0,0), 1)
                cv2.line(mapImage, (100,300), (1000,300), (0,0,0), 1)
                cv2.line(mapImage, (100,400), (1000,400), (0,0,0), 1)
                cv2.line(mapImage, (100,500), (1000,500), (0,0,0), 1)
                cv2.line(mapImage, (100,600), (1000,600), (0,0,0), 1)
                
                cv2.line(mapImage, (200,100), (200,700), (0,0,0), 1)
                cv2.line(mapImage, (300,100), (300,700), (0,0,0), 1)
                cv2.line(mapImage, (400,100), (400,700), (0,0,0), 1)
                cv2.line(mapImage, (500,100), (500,700), (0,0,0), 1)
                cv2.line(mapImage, (600,100), (600,700), (0,0,0), 1)
                cv2.line(mapImage, (700,100), (700,700), (0,0,0), 1)
                cv2.line(mapImage, (800,100), (800,700), (0,0,0), 1)
                cv2.line(mapImage, (900,100), (900,700), (0,0,0), 1)

                textLine = "(0,0)"
                x, y = self.worldCoorToImageCoor(-50,0)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                # cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                textLine = "(0,600)"
                x, y = self.worldCoorToImageCoor(-50,600)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                # cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                textLine = "(900,600)"
                x, y = self.worldCoorToImageCoor(900,600)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                # cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                textLine = "(900,0)"
                x, y = self.worldCoorToImageCoor(900,0)
                cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
                # cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

                drawLandmark = True
                if drawLandmark == True:
                    for i in range(totalLandmarks):
                        x, y = self.worldCoorToImageCoor(int(landmarksPosition[i,0]), int(landmarksPosition[i,1]))
                        cv2.circle(mapImage,(x, y), 10, (127,0,127), -1)

            # Get data from kinematic
            if simulationMode == False:
                totalDetectLandmarks 	= int(self.totalDetectLandmarks)
                arahGoal			    = int(self.goal_side)
                

            # Kalau keluar lapangan random posisi robot yg baru
            if simulationMode == True:
                if self.robotGlobalPosition[0] < 0 or self.robotGlobalPosition[0] >= fieldLength or self.robotGlobalPosition[1] < 0 or self.robotGlobalPosition[1] >= fieldWidth:
                    self.robotInitialPosition[0] = uniform(0, fieldLength)
                    self.robotInitialPosition[1] = uniform(0, fieldWidth)
                    self.robotInitialPosition[2] = uniform(0, 180)
                    self.robotGlobalPosition[:] = 0
                    self.robotLocalPosition[:] = 0

            # Simulate robot movement
            self.robotLocalPosition[0] += realVelocity[0] * deltaTime
            self.robotLocalPosition[1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                self.robotLocalPosition[2] = imuCurrentHeading #- imuInitHeading
            #else:
                    #robotLocalPosition[2] += realVelocity[2] * deltaTime

            # Motion model heading
            #if robotLocalPosition[2] >= 360:
                    #robotLocalPosition[2] = robotLocalPosition[2] - 360
            #if robotLocalPosition[2] < 0:
                    #robotLocalPosition[2] = 360 + robotLocalPosition[2]

            if headingFromIMU:
                angle = self.robotLocalPosition[2]
            #else:
                    #angle = robotInitialPosition[2] + robotLocalPosition[2]

            #if angle >= 360:
                    #angle = angle - 360
            #if angle < 0:
                    #angle = 360 + angle

            # Create matrix rotation
            theta = np.radians(angle)
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c,-s), (s, c)))
            npOutMatMul = np.matmul(R, self.robotLocalPosition[:2]) 
            self.robotGlobalPosition[0] = npOutMatMul[0] + self.robotInitialPosition[0]
            self.robotGlobalPosition[1] = npOutMatMul[1] + self.robotInitialPosition[1]
            self.robotGlobalPosition[2] = angle

            # Predict movement of particles
            particlesLocalPosition[:,0] += realVelocity[0] * deltaTime
            particlesLocalPosition[:,1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                particlesLocalPosition[:,2] = imuCurrentHeading #- imuInitHeading
            #else:
                    #particlesLocalPosition[:,2] += realVelocity[2] * deltaTime
        
            # Simulate noise movement of robot with error stddev = 10
            simulateNoiseMovement = False
            if simulateNoiseMovement == True:
                particlesLocalPosition[:,0] = normal(particlesLocalPosition[:,0], 10)
                particlesLocalPosition[:,1] = normal(particlesLocalPosition[:,1], 10)
                particlesLocalPosition[:,2] = normal(particlesLocalPosition[:,2], 3)

            # Calculate position of particles in global coordinat
            updateParticlesMovement = True
            if updateParticlesMovement == True:
                for i in range (0,totalParticles):
                    #if particlesLocalPosition[i,2] >= 360:
                            #particlesLocalPosition[i,2] = particlesLocalPosition[i,2] - 360
                    #if particlesLocalPosition[i,2] < 0:
                            #particlesLocalPosition[i,2] = 360 + particlesLocalPosition[i,2]
                    particlesLocalPosition[i,2] = particlesLocalPosition[i,2]

                    # Kalau pakai data IMU dianggap tidak ada rotasi 
                    if headingFromIMU:
                        angle = particlesLocalPosition[i,2]
                    # Kalau pakai yaw rate ditambahkan dulu dengan initial position
                    else:
                        angle = particlesInitialPosition[i,2] + particlesLocalPosition[i,2]

                    # Check limit bearing
                    #if angle >= 360:
                            #angle = angle - 360
                    #if angle < 0:
                            #angle = 360 + angle
                    theta = np.radians(angle)
                    c, s = np.cos(theta), np.sin(theta)
                    R = np.array(((c,-s), (s, c)))
                    npOutMatMul = np.matmul(R, particlesLocalPosition[i,:2]) 
                    particlesGlobalPosition[i,0] = npOutMatMul[0] + particlesInitialPosition[i,0]
                    particlesGlobalPosition[i,1] = npOutMatMul[1] + particlesInitialPosition[i,1]
                    particlesGlobalPosition[i,2] = angle

                    # Jika keluar lapangan random partikel yang baru di sekitar estimate position terakhir
                    if particlesGlobalPosition[i,0] < 0 or particlesGlobalPosition[i,1] < 0 or particlesGlobalPosition[i,0] >= fieldLength or particlesGlobalPosition[i,1] >= fieldWidth:
                        # Cek kalau estimatenya tdk nan atau inf
                        if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                            particlesInitialPosition[i,0] = uniform(0, fieldLength)
                            particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                            particlesInitialPosition[i,2] = uniform(0, 3650)
                            particlesGlobalPosition[i,:] = 0
                            particlesLocalPosition[i,:] = 0
                        else:
                            particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
                            particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
                            particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
                            particlesGlobalPosition[i,:] = 0
                            particlesLocalPosition[i,:] = 0

        # Measurement distance between robot and landmarks
        if simulationMode == True:
            for i in range (0,totalLandmarks):
                distanceRobotToLandmarks[i] = distance.euclidean([self.robotGlobalPosition[:2]], [landmarksPosition[i]])

        # Resample only when get valid distance data
        if totalDetectLandmarks >= 2:
            resample = True
        else:
            resample = False

        # Calculate estimate position
        # Jika ada perintah resample
        if resample == True:
            try :
                # Measurement distance between particles and landmarks
                for i in range (0, totalParticles):
                    for j in range (0, totalLandmarks):
                        if distanceRobotToLandmarks[j] > 0 :
                            distanceParticlesToLandmarks[i,j] = distance.euclidean([particlesGlobalPosition[i,:2]], [landmarksPosition[j]])
                        else :
                            distanceParticlesToLandmarks[i,j] = 0

                # Calculating weight
                # Initialize particles weight with 1.00
                particlesWeight.fill(1.0)
                for i in range (0, totalParticles):
                    for j in range (0, totalLandmarks):
                        # mean = jarak robot ke landmark
                        # stddev = 5
                        particlesWeight[i] *= scipy.stats.norm.pdf(distanceParticlesToLandmarks[i,j],distanceRobotToLandmarks[j],5)

                # Normalize weight
                totalWeight = sum(particlesWeight)

                for i in range (0, totalParticles):
                    if totalWeight != 0:
                        particlesWeight[i] = particlesWeight[i] / totalWeight        

                estimatePosition[:] = np.average(particlesGlobalPosition, weights=particlesWeight, axis=0)
                estimateInitialPosition[:] = estimatePosition[:]
                estimateLocalPosition[:] = 0

            except :
                pass

        # Jika tidak update estimate position dengan data dari kinematik
        else:
            estimateLocalPosition[0] += realVelocity[0] * deltaTime
            estimateLocalPosition[1] += realVelocity[1] * deltaTime
            if headingFromIMU:
                estimateLocalPosition[2] = imuCurrentHeading #- imuInitHeading
            #else:
                    #estimateLocalPosition[2] += realVelocity[2] * deltaTime

            #if estimateLocalPosition[2] >= 360:
                    #estimateLocalPosition[2] = estimateLocalPosition[2] - 360
            #if estimateLocalPosition[2] < 0:
                    #stimateLocalPosition[2] = 360 + estimateLocalPosition[2]

            if headingFromIMU:
                angle = estimateLocalPosition[2]
            #else:
                    #angle = estimateInitialPosition[2] + estimateLocalPosition[2]

            #if angle >= 360:
                    #angle = angle - 360
            #if angle < 0:
                    #angle = 360 + angle
            theta = np.radians(angle)
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c,-s), (s, c)))
            npOutMatMul = np.matmul(R, estimateLocalPosition[:2])
            #estimatePosition[0] = npOutMatMul[0] + estimateInitialPosition[0]
            #estimatePosition[1] = npOutMatMul[1] + estimateInitialPosition[1]
            estimatePosition[0] = int(posRobotX+self.robotInitialPosition[0])
            estimatePosition[1] = int(posRobotY+self.robotInitialPosition[1])
            estimatePosition[2] = angle

        # Mark as -888 if result infinity or nan
        if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
            estimatePosition[:] = -888
            ballEstimatePosition[:] = -888
            # random uniform lagi
        else:
            # ini masih dalam koordinat lokal robot
            ballEstimatePosition[0] = ballDistance
            ballEstimatePosition[1] = 0
            # ini nanti ditambahkan sama posisi servo pan
            headHeading = panAngle;
            if headHeading  >= 360:
                    headHeading = headHeading  - 360
            if headHeading  < 0:
                    headHeading  = 360 + headHeading
            theta = np.radians(headHeading)
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c,-s), (s, c)))
            npOutMatMul = np.matmul(R, ballEstimatePosition[:2])
            ballEstimatePosition[0] = npOutMatMul[0] + estimatePosition[0]
            ballEstimatePosition[1] = npOutMatMul[1] + estimatePosition[1]

        #print "Robot Global Position : ", robotGlobalPosition
        #print "Robot Estimate Position : ", estimatePosition
        #print "Ball Estimate Position : ", ballEstimatePosition

        # Kirim X, Y, Theta Robot, Ball X, Ball Y
        if simulationMode == False:
            if resample == True:
                self.sendToMain()
                if arahGoal == 0:
                        msgToMainProgram = "{},{},{},{},{}".format(int(self.robotCoorToMain[0]), int(self.robotCoorToMain[1]), int(estimatePosition[2]), int(self.ballCoorToMain[0]), int(self.ballCoorToMain[1])) 
                #print "Masuk IF"
                #print "DataSendToMain : ", msgToMainProgram
                #print "Estimasi X Pure : ", estimatePosition[0]
                #print "Estimasi Y pure : ", estimatePosition[1]
                        #print "Estimasi X Pure : ", ballCoorToMain[0]
                #print "Estimasi Y pure : ", ballCoorToMain[1]
                else :
                        msgToMainProgram = "{},{},{},{},{}".format(int(self.robotCoorToMain[0]*-1), int(self.robotCoorToMain[1]*-1), int(estimatePosition[2]), int(self.ballCoorToMain[0]*-1), int(self.ballCoorToMain[1]*-1)) 
                #print "Masuk Else"
                #print "DataSendToMain : ", msgToMainProgram
                        #print "Estimasi X Pure : ", estimatePosition[0]
                #print "Estimasi Y pure : ", estimatePosition[1]
                        #print "Estimasi X Pure : ", ballCoorToMain[0]
                #print "Estimasi Y pure : ", ballCoorToMain[1]

            elif resample == False:
                self.sendToMain()
                if arahGoal == 0:
                    msgToMainProgram = "{},{},{},{},{}".format(int(-1),int(-1),int(estimatePosition[2]), int(self.ballCoorToMain[0]), int(self.ballCoorToMain[1]))
                    #print "Masuk IF"
                    #print "DataSendToMain : ", msgToMainProgram
                    #print "Estimasi X Pure : ", estimatePosition[0]
                    #print "Estimasi Y pure : ", estimatePosition[1]
                    #print "Estimasi X Pure : ", ballCoorToMain[0]
                    #print "Estimasi Y pure : ", ballCoorToMain[1]
                else :
                    msgToMainProgram = "{},{},{},{},{}".format(int(-1),int(-1),int(estimatePosition[2]), int(self.ballCoorToMain[0]), int(self.ballCoorToMain[1])) 
                    #print "Masuk Else"
                    #print "DataSendToMain : ", msgToMainProgram
                    #print "Estimasi X Pure : ", estimatePosition[0]
                    #print "Estimasi Y pure : ", estimatePosition[1]
                    #print "Estimasi X Pure : ", ballCoorToMain[0]
                    #print "Estimasi Y pure : ", ballCoorToMain[1]
        drawParticles = False
        if drawParticles == True:
                for i in range (0, totalParticles):
                        x, y = self.worldCoorToImageCoor(int(particlesGlobalPosition[i,0]), int(particlesGlobalPosition[i,1]))
                        cv2.circle(mapImage,(x, y), 7, (0,0,255), -1)

        drawSimRobot = False
        if drawSimRobot == True:
                x, y = self.worldCoorToImageCoor(int(self.robotGlobalPosition[0]), int(self.robotGlobalPosition[1]))
                cv2.circle(mapImage,(x, y), 12, (0,255,255), -1)

        drawEstimatePosition = False
        if drawEstimatePosition == True:
                try:
                        x, y = self.worldCoorToImageCoor(int(estimatePosition[0]), int(estimatePosition[1]))
                        cv2.circle(mapImage,(x, y), 12, (255,0,0), -1)
                except:
                        pass

        drawOdometryPosition = True
        if drawOdometryPosition == True:
                try:
                        self.odometryPosition[0] = int(posRobotX+self.robotInitialPosition[0])
                        self.odometryPosition[1] = int(posRobotY+self.robotInitialPosition[1]) 
                        x,y = self.worldCoorToImageCoor(int(self.odometryPosition[0]),int(self.odometryPosition[1]))
                        '''
                        startxDraw = x
                        startyDraw = y
                        '''
                        cv2.circle(mapImage,(x, y), 10, (0,128,255), -1)

                        '''
                        deltaxDraw = 150*math.cos(imuCurrentHeading*phi/180)
                        deltayDraw = 150*math.sin(imuCurrentHeading*phi/180)
                        cv2.line(mapImage,(startxDraw,startyDraw),(int(startxDraw+deltaxDraw),int(startyDraw+deltayDraw)),(0,127,255),3)

                        xDrawOm = 1000*math.cos(odomHeading*phi/180)
                        yDrawOm = 700*math.sin(odomHeading*phi/180)
                        ax = int(startxDraw+xDrawOm)
                        by = int(startyDraw+yDrawOm)
                        if ax >= 1000:
                                ax = 1000
                        if by >= 700:
                                by = 700
                        cv2.line(mapImage,(startxDraw,startyDraw),(int(1000),int(400)),(255,0,0),3)
                        '''
                except:
                        pass
                #print "Odometry Position : ", odometryPosition
                #print "deltaxDraw : ", xDrawOm
                #print "deltayDraw : ", yDrawOm
                #print "Position X : ", robotInitialPosition[0]
                #print "Position Y : ", robotInitialPosition[1]
                        
        drawBallEstimatePosition = True
        if drawBallEstimatePosition == True:
                try:
                        x, y = self.worldCoorToImageCoor(int(ballEstimatePosition[0]), int(ballEstimatePosition[1]))
                        cv2.circle(mapImage,(x, y), 10, (255,255,0), -1)
                except:
                        pass

        # Put text on display
        textLine = "R%d Velocity : (%.2f, %.2f, %.2f)"%(robotID, realVelocity[0], realVelocity[1], realVelocity[2])
        cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
        # cv2.putText(mapImage, textLine, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

        textLine = "R{} Local Position : ({}, {}, {})".format(robotID, int(self.robotLocalPosition[0]), int(self.robotLocalPosition[1]), int(self.robotLocalPosition[2]))
        cv2.putText(mapImage, textLine, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
        # cv2.putText(mapImage, textLine, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

        textLine = "R{} Global Position : ({}, {}, {})".format(robotID, int(self.robotGlobalPosition[0]), int(self.robotGlobalPosition[1]), int(self.robotGlobalPosition[2]))
        cv2.putText(mapImage, textLine, (340,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
        # cv2.putText(mapImage, textLine, (340,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

        textLine = "R{} Estimate Position : ({}, {}, {})".format(robotID, int(estimatePosition[0]), int(estimatePosition[1]), int(estimatePosition[2]))
        cv2.putText(mapImage, textLine, (340,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA) #OpenCV 3.x
        # cv2.putText(mapImage, textLine, (340,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.CV_AA) #OpenCV 2.x

        # Enable GUI Streaming
        showGUI = True
        if showGUI == True:
                #smallMapImage = cv2.resize(mapImage, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
                smallMapImage = cv2.resize(mapImage, (640,480), interpolation = cv2.INTER_AREA)
                cv2.imshow("Barelang Localization", smallMapImage)

        # Enable URL Streaming
        streamUrl = False
        if streamUrl == True:
                smallMapImage = cv2.resize(mapImage, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
                #cv2.imwrite('stream.jpg', smallMapImage)
                cv2.imwrite('output/localization.jpg', smallMapImage)
                #yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + open('stream.jpg', 'rb').read() + b'\r\n')
            
        # Resample
        if resample == True:
                indexHighestWeight = np.argmax(particlesWeight)
                xHighest = particlesGlobalPosition[indexHighestWeight,0]
                yHighest = particlesGlobalPosition[indexHighestWeight,1]
                thetaHighest = particlesGlobalPosition[indexHighestWeight,2]

                _10PercentParticle = int(totalParticles * 0.1)

                for i in range (0, _10PercentParticle):
                        particlesInitialPosition[i,0] = uniform(0, fieldLength)
                        particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                        particlesInitialPosition[i,2] = uniform(0, 360)
                        particlesGlobalPosition[i,:] = 0
                        particlesLocalPosition[i,:] = 0

                _90PercentParticle = totalParticles - _10PercentParticle

                for i in range (_10PercentParticle + 1, _90PercentParticle):
                        #particlesInitialPosition[i,0] = normal(xHighest, 50)
                        #particlesInitialPosition[i,1] = normal(yHighest, 50)
                        #particlesInitialPosition[i,2] = normal(thetaHighest, 10)
                        if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                                particlesInitialPosition[i,0] = uniform(0, fieldLength)
                                particlesInitialPosition[i,1] = uniform(0, fieldWidth)
                                particlesInitialPosition[i,2] = uniform(0, 360)
                                particlesGlobalPosition[i,:] = 0
                                particlesLocalPosition[i,:] = 0
                        else:
                                particlesInitialPosition[i,0] = normal(estimatePosition[0], 50)
                                particlesInitialPosition[i,1] = normal(estimatePosition[1], 50)
                                particlesInitialPosition[i,2] = normal(estimatePosition[2], 10)
                                particlesGlobalPosition[i,:] = 0
                                particlesLocalPosition[i,:] = 0

        if showGUI:
                #cv2.waitKey(int(deltaTime*1000))
                key = cv2.waitKey(1)
        #         if key == 27:
        #             break

def main(args=None):
    rclpy.init(args=args)
    node = particleFilterSim()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
