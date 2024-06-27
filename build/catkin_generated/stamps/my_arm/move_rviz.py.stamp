# # #!/usr/bin/env python

# # import rospy
# # from sensor_msgs.msg import JointState

# # def publish_joint_states():
# #     rospy.init_node('joint_state_publisher')
# #     pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
# #     rate = rospy.Rate(10)  # 10 Hz

# #     joint_state = JointState()
# #     joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
# #     joint_state.position = [0.5, 0.7, 0.4, 0.9, 1.0]  # example positions
# #     joint_state.velocity = [3.0, 3.0, 3.0, 3.0, 3.0]  # example velocities
# #     joint_state.effort = [300.0, 200.0, 200.0, 200.0, 200.0]  # example efforts

# #     while not rospy.is_shutdown():
# #         joint_state.header.stamp = rospy.Time.now()
# #         pub.publish(joint_state)
# #         rate.sleep()

# # if __name__ == '__main__':
# #     try:
# #         publish_joint_states()
# #     except rospy.ROSInterruptException:
# #         pass

# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import JointState
# import numpy as np

# def publish_joint_states():
#     rospy.init_node('joint_state_publisher')
#     pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
#     rate = rospy.Rate(50)  # 10 Hz

#     joint_state = JointState()
#     joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']

#     while not rospy.is_shutdown():
#         current_time = rospy.Time.now().to_sec()
        
#         # Example motion: sinusoidal motion for demonstration
#         joint_state.position = [np.sin(current_time + i) for i in range(5)]
#         # joint_state.velocity = [np.cos(current_time + i) for i in range(5)]
#         # joint_state.effort = [np.abs(np.sin(current_time + i) * 100) for i in range(5)]
        
#         joint_state.header.stamp = rospy.Time.now()
#         pub.publish(joint_state)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_joint_states()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

import tkinter as tk
from math import comb , ceil , pi , sin, cos, radians , degrees , acos , atan2 , asin , sqrt
import matplotlib.pyplot as plt

delta = 1e-6

def sign(x):
    if(x >= 0):
        return 1
    return -1

class Point:
    def __init__(self , x , y) -> None:
        self.x = x
        self.y = y
    
    def copy(point):
        return Point(point.x , point.y)

    def magnitude(self):
        return sqrt(self.x**2 + self.y**2)

    def changeMagnitude(self , newMagnitude):
        oldMagnitude = self.magnitude()
        self.x *= newMagnitude / oldMagnitude
        self.y *= newMagnitude / oldMagnitude

    def apply(self , func):
        self.x = func(self.x)
        self.y = func(self.y)
        return self

class pathGenerator():
    unitLength = 0.3

    def __init__(self) -> None:
        pass

    def tangentPoint(p_0 , magnitude , angle):
        p_f = list(p_0)
        p_f[0] += magnitude * cos(pi * angle/180)
        p_f[1] += magnitude * sin(pi * angle/180)
        return p_f

    def Bezier_curve(t , c):
        B = [0 , 0]
        for j in range(2):
            for i in range(4):
                B[j] += comb(3 , i) * c[i][j] * (1 - t)**(3 - i) * t**i         
        return B

    def calculateCurveLength(c):
        length = 0
        lastPoint = pathGenerator.Bezier_curve(0 , c)
        for i in range(1 , 1001):
            point = pathGenerator.Bezier_curve(i/1000 , c)
            length += sqrt((point[0] - lastPoint[0])**2 + (point[1] - lastPoint[1])**2)
            lastPoint = point
        return length
    
    def getNumberOfPoints(l):
        return ceil(l / pathGenerator.unitLength)

    def segmentFunction(x):
        return (1 - cos(2 * pi * x))/2

    def bellSegmenter(pointsPerPath = 100):
        pointsPerPath = max(1 , pointsPerPath + 1)
        timestamps = [pathGenerator.segmentFunction(1 / (pointsPerPath + 1))]
        accumlator = timestamps[0]
        for i in range(2 , pointsPerPath):
            timestamp = pathGenerator.segmentFunction(i / (pointsPerPath + 1))
            timestamps.append(timestamps[-1] + timestamp)
            accumlator += timestamp
        timestamps = [timestamp / accumlator for timestamp in timestamps]
        return timestamps

    def graph1DList(data):
        # Creating y-values (all zeros) for plotting purposes
        y_values = [0] * len(data)
        # Plotting the values on a one-dimensional axis
        plt.scatter(data, y_values)
        # Setting y-axis limits
        plt.ylim(-1, 1)
        # Removing y-axis ticks and labels
        plt.yticks([])
        # Display the plot
        plt.show()

    def graph2DList(data):
        # Creating y-values (all zeros) for plotting purposes
        x_values , y_values = [[point[j] for point in data] for j in range(2)]
        # Plotting the values on a one-dimensional axis
        plt.scatter(x_values, y_values)
        # Show grid
        plt.grid(True)
        # Display the plot
        plt.show()

point_0 = [4 , 6]
point_f = [10 , -1]
mag_ang_0 = [2 , 90]
mag_ang_f = [3 , 180]

c = [point_0 , None , None , point_f]
c[1] = pathGenerator.tangentPoint(c[0] , magnitude = mag_ang_0[0] , angle = mag_ang_0[1])
c[2] = pathGenerator.tangentPoint(c[3] , magnitude = mag_ang_f[0] , angle = mag_ang_f[1])

curve_length = pathGenerator.calculateCurveLength(c)
n = pathGenerator.getNumberOfPoints(curve_length)
numberOfPoints =  ceil(n/8 + 2 * sqrt(n))
pathTimeStamps = pathGenerator.bellSegmenter(60)
Bezier_curve = [pathGenerator.Bezier_curve(timestamp , c) for timestamp in pathTimeStamps]

class Robot:
    origin = Point(0 , 0)
    shoulder_length = 120
    elbow_length = 118
    wrist_length = 50
    mergeWristElbow = False

    # configurable
    dynamicMode = True
    approachFromUp = True
    wristApproachAngle = 40
    mappedApproachPair = (120 , shoulder_length/2)

    def handlePoint(point):
        if(point.magnitude() > Robot.shoulder_length + Robot.elbow_length + Robot.wrist_length * Robot.dynamicMode - delta): # + Robot.wrist_length 
            point.changeMagnitude(Robot.shoulder_length + Robot.elbow_length + Robot.wrist_length * Robot.dynamicMode - delta) # + Robot.wrist_length 
        return point

    def dof3_constantAngleApproach(point , angle = 0): # angle here in degrees
        newPoint = Point.copy(point)
        deltax = Robot.wrist_length * cos(radians(angle))
        deltay = Robot.wrist_length * sin(radians(angle))

        newPoint.x -= deltax
        newPoint.y -= deltay
        return newPoint , radians(angle)

    def dof3_mappedAngleApproach(point , approachPair = None):
        Robot.mergeWristElbow = False
        if(approachPair == None):
            approachPair = Robot.mappedApproachPair

        # 90 @ degree90radius_local , point direction @  Robot.shoulder_length + Robot.elbow_length + Robot.wrist_length  - delta
        pointDirection = degrees(atan2(point.y , point.x))
        
        m = (pointDirection - approachPair[0]) / ((Robot.shoulder_length + Robot.elbow_length + Robot.wrist_length - delta) - approachPair[1])
        c = approachPair[0] - approachPair[1] * m

        distance = point.magnitude()
        mappedAngle = m * distance + c

        if(distance < approachPair[1]):
            Robot.mergeWristElbow = True
            return point , 0

        return Robot.dof3_constantAngleApproach(point , mappedAngle)

    def dof3_Handler(point):
        if Robot.dynamicMode:
            return Robot.dof3_mappedAngleApproach(point)
        return Robot.dof3_constantAngleApproach(point , angle = Robot.wristApproachAngle)

    def handle_negative_beta(phi , theta):
        if((phi + theta < phi - theta) ^ Robot.approachFromUp):
            return phi - theta
        return phi + theta

    def inverseKinematics(point):
        point = Robot.handlePoint(point)
        point , motor3 = Robot.dof3_Handler(point)

        a = point.x
        z = point.y

        phi = atan2(z , a)
        L = sqrt(a ** 2 + z ** 2)

        tooNearCondition = Robot.dynamicMode and Robot.mergeWristElbow

        # unequal condition
        cos_theta_nom = Robot.shoulder_length ** 2 + L ** 2 - (Robot.elbow_length + Robot.wrist_length * tooNearCondition) ** 2
        cos_theta = cos_theta_nom / (2 * Robot.shoulder_length * L)
        theta = acos(cos_theta)

        # # equal condition
        # theta = acos(L/2/Robot.shoulder_length)

        beta_1 = Robot.handle_negative_beta(phi , theta)
        
        # unequal condition
        tan_beta_2_nom = (z - Robot.shoulder_length * sin(beta_1))
        tan_beta_2_den = (a - Robot.shoulder_length * cos(beta_1))
        beta_2 = atan2(tan_beta_2_nom , tan_beta_2_den) 

        # # equal condition
        # beta_2 = phi - theta

        motor1 = beta_1
        motor2 = beta_2 - motor1
        motor3 = motor3 - motor2 - motor1

        if(tooNearCondition):
            motor3 = 0

        return motor1 , motor2 , motor3

moveindex = 0

# dynamicMode = True
# approachFromUp = True
# wristApproachAngle = 40

# scriptedMoves = [[True , True , 0] , [True , False , 0] , [False , True , 0] , [False , True , 45] , [False , True , 90] , [False , False , 90]]
scriptedMoves = [ [False , True , 0] , [False , True , 45] , [False , True , 90]  , [True , True , 90] ]


# self.initial_x = int(self.initial_x_entry.get())
# self.initial_y = int(self.initial_y_entry.get())
# self.target_x = int(self.target_x_entry.get())
# self.target_y = int(self.target_y_entry.get())

# self.angle_depature = int(self.angle_depature_entry.get())
# self.magnitude_depature = int(self.magnitude_depature_entry.get())
# self.angle_arrival = int(self.angle_arrival_entry.get())
# self.magnitude_arrival = int(self.magnitude_arrival_entry.get())

# scriptIndex = 0
# scriptedPaths = [[[2 , 2] , [3 , 2] , [45 , 2] , [-22 , 2]] , [[-3 , 2] , [6 , -1] , [90 , 2] , [45 , 4]]]
# scriptedPaths = [[[3 , 2] , [8 , -1] , [90 , 2] , [45 , 4]]]

class RobotArmSimulator:
    origin = Point(200 , 200)
    grid_size = 20
    endEffectorPostion = Point(0 , 0)

    def __init__(self, master):
        self.master = master
        master.title("zoghby simulator")

        self.canvas = tk.Canvas(master, width=400, height=400, bg="white")
        self.canvas.pack(side=tk.LEFT)

        self.base_angle = 0
        self.shoulder_angle = 0
        self.elbow_angle = 0

        self.manual_mode = tk.BooleanVar()
        self.manual_mode.set(True)

        self.mode_label = tk.Label(master, text="Mode:", font=("Arial", 12))
        self.mode_label.pack(anchor="w", padx=10, pady=5)

        self.mode_switch = tk.Checkbutton(master, text="Manual Mode", variable=self.manual_mode, command=self.toggle_mode, font=("Arial", 10))
        self.mode_switch.pack(anchor="w", padx=10, pady=5)

        self.slider_frame = tk.Frame(master, bg="lightgray")
        self.slider_frame.pack(padx=10, pady=10)

        self.base_label = tk.Label(self.slider_frame, text="Base Angle", font=("Arial", 10))
        self.base_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.base_slider = tk.Scale(self.slider_frame, from_= -180, to=180, orient="horizontal", command=self.update_base_angle)
        self.base_slider.grid(row=0, column=1, sticky="w", padx=10, pady=5)

        self.shoulder_label = tk.Label(self.slider_frame, text="Shoulder Angle", font=("Arial", 10))
        self.shoulder_label.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        self.shoulder_slider = tk.Scale(self.slider_frame, from_=-180, to=180, orient="horizontal", command=self.update_shoulder_angle)
        self.shoulder_slider.grid(row=1, column=1, sticky="w", padx=10, pady=5)

        self.elbow_label = tk.Label(self.slider_frame, text="Elbow Angle", font=("Arial", 10))
        self.elbow_label.grid(row=2, column=0, sticky="w", padx=10, pady=5)
        self.elbow_slider = tk.Scale(self.slider_frame, from_=-180, to=180,orient="horizontal", command=self.update_elbow_angle )
        self.elbow_slider.grid(row=2, column=1, sticky="w", padx=10, pady=5)

        self.target_panel = tk.Frame(master, bg="lightgray")
        self.target_panel.pack(padx=10, pady=10)

        self.initial_x_label = tk.Label(self.target_panel, text="initial X", font=("Arial", 10))
        self.initial_x_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.initial_x_entry = tk.Entry(self.target_panel)
        self.initial_x_entry.grid(row=0, column=1, padx=10, pady=5)

        self.target_x_label = tk.Label(self.target_panel, text="Target X", font=("Arial", 10))
        self.target_x_label.grid(row=0, column=2, sticky="w", padx=10, pady=5)
        self.target_x_entry = tk.Entry(self.target_panel)
        self.target_x_entry.grid(row=0, column=3, padx=10, pady=5)

        self.initial_y_label = tk.Label(self.target_panel, text="initial Y", font=("Arial", 10))
        self.initial_y_label.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        self.initial_y_entry = tk.Entry(self.target_panel)
        self.initial_y_entry.grid(row=1, column=1, padx=10, pady=5)

        self.target_y_label = tk.Label(self.target_panel, text="Target Y", font=("Arial", 10))
        self.target_y_label.grid(row=1, column=2, sticky="w", padx=10, pady=5)
        self.target_y_entry = tk.Entry(self.target_panel)
        self.target_y_entry.grid(row=1, column=3, padx=10, pady=5)

        self.angle_panel = tk.Frame(master, bg="lightgray")
        self.angle_panel.pack(padx=10, pady=10)

        self.angle_depature_label = tk.Label(self.angle_panel, text="Depature θ", font=("Arial", 10))
        self.angle_depature_label.grid(row=2, column=0, sticky="w", padx=10, pady=5)
        self.angle_depature_entry = tk.Entry(self.angle_panel)
        self.angle_depature_entry.grid(row=2, column=1, padx=10, pady=5)

        self.magnitude_depature_label = tk.Label(self.angle_panel, text="Depature mag.", font=("Arial", 10))
        self.magnitude_depature_label.grid(row=3, column=0, sticky="w", padx=10, pady=5)
        self.magnitude_depature_entry = tk.Entry(self.angle_panel)
        self.magnitude_depature_entry.grid(row=3, column=1, padx=10, pady=5)

        self.angle_arrival_label = tk.Label(self.angle_panel, text="Arrival θ", font=("Arial", 10))
        self.angle_arrival_label.grid(row=2, column=2, sticky="w", padx=10, pady=5)
        self.angle_arrival_entry = tk.Entry(self.angle_panel)
        self.angle_arrival_entry.grid(row=2, column=3, padx=10, pady=5)

        self.magnitude_arrival_label = tk.Label(self.angle_panel, text="Arrival mag.", font=("Arial", 10))
        self.magnitude_arrival_label.grid(row=3, column=2, sticky="w", padx=10, pady=5)
        self.magnitude_arrival_entry = tk.Entry(self.angle_panel)
        self.magnitude_arrival_entry.grid(row=3, column=3, padx=10, pady=5)

        self.set_target_button = tk.Button(self.angle_panel, text="Set Target", command=self.set_target)
        self.set_target_button.grid(row=4, columnspan= 4, pady=5)

        self.angle_depature = mag_ang_0[1]
        self.magnitude_depature = mag_ang_0[0]
        self.angle_depature_entry.insert(0, self.angle_depature)
        self.magnitude_depature_entry.insert(0, self.magnitude_depature)

        self.angle_arrival = mag_ang_f[1]
        self.magnitude_arrival = mag_ang_f[0]
        self.angle_arrival_entry.insert(0, self.angle_arrival)
        self.magnitude_arrival_entry.insert(0, self.magnitude_arrival)

        self.initial_x = point_0[0]
        self.initial_y = point_0[1]
        self.initial_x_entry.insert(0, self.initial_x)
        self.initial_y_entry.insert(0, self.initial_y)

        self.target_x = point_f[0]
        self.target_y = point_f[1]
        self.target_x_entry.insert(0, self.target_x)
        self.target_y_entry.insert(0, self.target_y)

        self.update_delay = 25  # Milliseconds between updates      # 50  
        self.RepeatDelayCount = 30

        # Add labels x and y
        self.x_label = tk.Label(master, text="x: 0", font=("Arial", 10))
        self.x_label.pack(anchor="w", padx=2, pady=2)
        self.y_label = tk.Label(master, text="y: 0", font=("Arial", 10))
        self.y_label.pack(anchor="w", padx=2, pady=2)

        self.draw_grid()
        self.draw_arm()
        for point in Bezier_curve:
            self.draw_dot_atGrid(Point(point[0], point[1]) , tags="target" , dot_size=2)

        # self.automatedMode()

    def update_label(self, label, value):
        label.config(text=f"{label.cget('text').split(':')[0]}: {value}")

    def toggle_mode(self):
        if self.manual_mode.get():
            self.base_slider.config(state="normal")
            self.shoulder_slider.config(state="normal")
            self.elbow_slider.config(state="normal")
            self.target_x_entry.config(state="normal")
            self.target_y_entry.config(state="normal")
            self.set_target_button.config(state="normal")
        else:
            global stepTracker
            stepTracker = 0
            self.automatedMode()

    def automatedMode(self):
        if not self.manual_mode.get():
            global stepTracker , timeCounter , moveindex , scriptIndex , Bezier_curve

            if(stepTracker != len(Bezier_curve) - 1):
                stepTracker += 1
                timeCounter = 0
            else:
                timeCounter += 1
                if(timeCounter == self.RepeatDelayCount):
                    stepTracker = 0

                    Robot.dynamicMode = scriptedMoves[moveindex][0]
                    Robot.approachFromUp = scriptedMoves[moveindex][1]
                    Robot.wristApproachAngle = scriptedMoves[moveindex][2]
                    moveindex += 1

                    if(moveindex == len(scriptedMoves)):
                        moveindex = 0


        # print("Setting target:", self.target_x,",", self.target_y)

            # Update angles
            targetPoint = Point(Bezier_curve[stepTracker][0] , Bezier_curve[stepTracker][1])
            pixelTarget = self.GridPoint_to_PixelPoint_Relative_To_Origin(targetPoint)
            self.base_angle , self.shoulder_angle , self.elbow_angle = [degrees(angle) for angle in Robot.inverseKinematics(pixelTarget)]

            # Update slider positions
            self.base_slider.set(self.base_angle)
            self.shoulder_slider.set(self.shoulder_angle)
            self.elbow_slider.set(self.elbow_angle)

            # Redraw arm
            self.draw_arm()

            # Schedule next rotation
            self.master.after(self.update_delay, self.automatedMode)

    def update_base_angle(self, angle):
        if self.manual_mode.get():
            self.base_angle = int(angle)
            self.draw_arm()

    def update_shoulder_angle(self, angle):
        if self.manual_mode.get():
            self.shoulder_angle = int(angle)
            self.draw_arm()

    def update_elbow_angle(self, angle):
        if self.manual_mode.get():
            self.elbow_angle = int(angle)
            self.draw_arm()

    def set_target(self):
        self.initial_x = int(self.initial_x_entry.get())
        self.initial_y = int(self.initial_y_entry.get())
        self.target_x = int(self.target_x_entry.get())
        self.target_y = int(self.target_y_entry.get())
        
        self.angle_depature = int(self.angle_depature_entry.get())
        self.magnitude_depature = int(self.magnitude_depature_entry.get())
        self.angle_arrival = int(self.angle_arrival_entry.get())
        self.magnitude_arrival = int(self.magnitude_arrival_entry.get())

        point_0 = [self.initial_x , self.initial_y]
        point_f = [self.target_x , self.target_y]
        mag_ang_0 = [self.magnitude_depature , self.angle_depature]
        mag_ang_f = [self.magnitude_arrival , self.angle_arrival]

        global Bezier_curve
        c = [point_0 , None , None , point_f]
        c[1] = pathGenerator.tangentPoint(c[0] , magnitude = mag_ang_0[0] , angle = mag_ang_0[1])
        c[2] = pathGenerator.tangentPoint(c[3] , magnitude = mag_ang_f[0] , angle = mag_ang_f[1])

        curve_length = pathGenerator.calculateCurveLength(c)
        n = pathGenerator.getNumberOfPoints(curve_length)
        numberOfPoints =  ceil(n/8 + 2 * sqrt(n))
        pathTimeStamps = pathGenerator.bellSegmenter(60)
        Bezier_curve = [pathGenerator.Bezier_curve(timestamp , c) for timestamp in pathTimeStamps]

        self.canvas.delete("target")
        
        for point in Bezier_curve:
            self.draw_dot_atGrid(Point(point[0], point[1]) , tags="target" , dot_size=2)
        
        global stepTracker
        stepTracker = 0

        # print("Setting target:", self.target_x,",", self.target_y)
    
    def draw_grid(self):
        for i in range(0, 400, RobotArmSimulator.grid_size):
            self.canvas.create_line(0, i, 400, i, fill="lightgray")
            self.canvas.create_line(i, 0, i, 400, fill="lightgray")

    def pixelToGrid(self , x):
        return x / RobotArmSimulator.grid_size

    def gridToPixel(self , x):
        return x * RobotArmSimulator.grid_size
    
    def GridPoint_to_PixelPoint_Relative_To_Origin(self , point):
        point.apply(self.gridToPixel)
        point.y = - point.y
        return point

    def PixelPoint_to_GridPoint(self , point):
        gridPoint = Point.copy(point)
        gridPoint.x = self.pixelToGrid(point.x - RobotArmSimulator.origin.x)
        gridPoint.y = self.pixelToGrid(RobotArmSimulator.origin.y - point.y)
        return gridPoint

    def reverse_PixelPoint_to_GridPoint(self , point):
        pixelPoint = Point.copy(point)
        pixelPoint.x = RobotArmSimulator.origin.x + self.gridToPixel(point.x)
        pixelPoint.y = RobotArmSimulator.origin.y - self.gridToPixel(point.y)
        return pixelPoint
    
    def draw_dot_atPixel(self, point , dot_size = 5 , dot_color = "red" , tags = None):
        self.canvas.create_oval(point.x - dot_size, point.y - dot_size, point.x + dot_size, point.y + dot_size, fill=dot_color , tags = tags)

    def draw_dot_atGrid(self, gridPoint , dot_size = 5 , dot_color = "red" , tags = None):
        self.draw_dot_atPixel(self.reverse_PixelPoint_to_GridPoint(gridPoint) , dot_size = dot_size , dot_color = dot_color , tags = tags)

    def draw_arm(self):
        global joint_state , pub , point
        # point.positions = [1.2, 0.78, -1.56, 0, 0.8]
        # point.positions = [0 , (-pi/2) -(radians(self.base_angle)), (radians(-self.shoulder_angle) - pi/2) , 0 , (radians(-self.elbow_angle) + pi/2)]
        point.positions = [0 , 0, 0,0,0]
        # joint_state.position = [0 , -pi/2 , -pi/2 , 0 , pi/2]
        joint_state.header.stamp = rospy.Time.now()
        joint_state.points.append(point)
        print(rospy.Time.now())
        pub.publish(joint_state)

        self.canvas.delete("arm")
            
        base = self.reverse_PixelPoint_to_GridPoint(Robot.origin)
        base_x = base.x
        base_y = base.y

        shoulder_length = Robot.shoulder_length
        elbow_length = Robot.elbow_length
        wrist_length = Robot.wrist_length

        shoulder_x = base_x + shoulder_length * cos(radians(self.base_angle))
        shoulder_y = base_y + shoulder_length * sin(radians(self.base_angle))

        elbow_x = shoulder_x + elbow_length * cos(radians(self.base_angle + self.shoulder_angle))
        elbow_y = shoulder_y + elbow_length * sin(radians(self.base_angle + self.shoulder_angle))

        end_x = elbow_x + wrist_length * cos(radians(self.base_angle + self.shoulder_angle + self.elbow_angle))
        end_y = elbow_y + wrist_length * sin(radians(self.base_angle + self.shoulder_angle + self.elbow_angle))

        self.canvas.create_line(base_x, base_y, shoulder_x, shoulder_y, tags="arm" , width = 7)
        self.canvas.create_line(shoulder_x, shoulder_y, elbow_x, elbow_y, tags="arm" , width = 5)
        self.canvas.create_line(elbow_x, elbow_y, end_x, end_y, tags="arm" , width = 3)

        self.draw_dot_atPixel(Point(shoulder_x , shoulder_y) , 6 , "blue" , tags="arm" )
        self.draw_dot_atPixel(Point(elbow_x , elbow_y) , 4 , "blue" , tags="arm" )

        endEffector = Point(end_x , end_y)
        RobotArmSimulator.endEffectorPostion = self.PixelPoint_to_GridPoint(endEffector)

        self.update_label(self.x_label, round(RobotArmSimulator.endEffectorPostion.x , 2))
        self.update_label(self.y_label, round(RobotArmSimulator.endEffectorPostion.y , 2))

        # print(self.currentPostion)


if __name__ == '__main__':
    rospy.init_node('joint_state_publisher')
    global joint_state , pub
    pub = rospy.Publisher('/set_joint_trajectory', JointTrajectory, queue_size=10)
    joint_state = JointTrajectory()
    joint_state.header.frame_id = "base_link"
    point = JointTrajectoryPoint()
    joint_state.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
try:
    root = tk.Tk()
    app = RobotArmSimulator(root)
    root.mainloop()

except rospy.ROSInterruptException:
    pass

# 2 ^ 8