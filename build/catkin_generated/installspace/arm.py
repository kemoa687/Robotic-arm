#!/usr/bin/env python3
from math import pi
import threading
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int16MultiArray
import numpy as np
import tkinter as tk
from math import comb, ceil, pi, sin, cos, radians, degrees, acos, atan2, sqrt
import matplotlib.pyplot as plt

delta = 1e-6

def sign(x):
    return 1 if x >= 0 else -1

class cylindericalPoint:
    def __init__(self , theta , x , y) -> None:
        self.theta = theta
        self.x = x
        self.y = y
    def toList(self):
        return [self.theta , self.x , self.y] 
    
class Point2D:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y
    
    @staticmethod
    def copy(point):
        return Point2D(point.x, point.y)

    def magnitude(self):
        return sqrt(self.x**2 + self.y**2)

    def changeMagnitude(self, newMagnitude):
        oldMagnitude = self.magnitude()
        self.x *= newMagnitude / oldMagnitude
        self.y *= newMagnitude / oldMagnitude

    def apply(self, func):
        self.x = func(self.x)
        self.y = func(self.y)
        return self
    
    def toList(self):
        return [self.x , self.y] 

class Point3D:
    def __init__(self, x, y, z=0) -> None:
        self.x = x
        self.y = y
        self.z = z
    
    @staticmethod
    def copy(point):
        return Point3D(point.x, point.y , point.z)

    def magnitude(self):
        return sqrt(self.x**2 + self.y**2 + self.z**2)

    def changeMagnitude(self, newMagnitude):
        oldMagnitude = self.magnitude()
        self.x *= newMagnitude / oldMagnitude
        self.y *= newMagnitude / oldMagnitude
        self.z *= newMagnitude / oldMagnitude

    def apply(self, func):
        self.x = func(self.x)
        self.y = func(self.y)
        self.z = func(self.y)
        return self
    
    def toList(self):
        return [self.x , self.y , self.z] 

class PathGenerator:
    unitLength = 0.3

    @staticmethod
    def tangentPoint(p_0, magnitude, angle):
        p_f = list(p_0)
        p_f[0] += magnitude * cos(pi * angle / 180)
        p_f[1] += magnitude * sin(pi * angle / 180)
        return p_f

    @staticmethod
    def Bezier_curve(t, c):
        B = [0, 0]
        for j in range(2):
            for i in range(4):
                B[j] += comb(3, i) * c[i][j] * (1 - t)**(3 - i) * t**i
        return B

    @staticmethod
    def calculateCurveLength(c):
        length = 0
        lastPoint = PathGenerator.Bezier_curve(0, c)
        for i in range(1, 1001):
            point = PathGenerator.Bezier_curve(i / 1000, c)
            length += sqrt((point[0] - lastPoint[0])**2 + (point[1] - lastPoint[1])**2)
            lastPoint = point
        return length
    
    @staticmethod
    def getNumberOfPoints(l):
        return ceil(l / PathGenerator.unitLength)

    @staticmethod
    def segmentFunction(x):
        return (1 - cos(2 * pi * x)) / 2

    @staticmethod
    def bellSegmenter(pointsPerPath=100):
        pointsPerPath = max(1, pointsPerPath + 1)
        timestamps = [PathGenerator.segmentFunction(1 / (pointsPerPath + 1))]
        accumlator = timestamps[0]
        for i in range(2, pointsPerPath):
            timestamp = PathGenerator.segmentFunction(i / (pointsPerPath + 1))
            timestamps.append(timestamps[-1] + timestamp)
            accumlator += timestamp
        timestamps = [timestamp / accumlator for timestamp in timestamps]
        return timestamps

    @staticmethod
    def graph1DList(data):
        y_values = [0] * len(data)
        plt.scatter(data, y_values)
        plt.ylim(-1, 1)
        plt.yticks([])
        plt.show(block=False)  # Use non-blocking mode
        plt.pause(0.001)  # Add a small pause to allow the plot to be displayed

    @staticmethod
    def graph2DList(data):
        x_values, y_values = [[point[j] for point in data] for j in range(2)]
        plt.scatter(x_values, y_values)
        plt.grid(True)
        plt.show(block=False)  # Use non-blocking mode
        plt.pause(0.001)  # Add a small pause to allow the plot to be displayed

class ArmJointTrajectory(JointTrajectory):
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.header.frame_id = "base_link"
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        self.header.stamp = rospy.Time.now()


class Robot:
    def __init__(self, shoulder_length=120, elbow_length=118, wrist_length=50, 
                 dynamicMode=False, approachFromUp=True, wristApproachAngle=90, 
                 mappedApproachPair=(120, 60), visualization=False):
        
        #ROS
        rospy.init_node('joint_state_publisher')
        self.Gazebo_publisher  = rospy.Publisher('/set_joint_trajectory', JointTrajectory, queue_size=10)
        self.arduino_publisher = rospy.Publisher('/arm_joint_int16_array', Int16MultiArray, queue_size=10)
        rospy.on_shutdown(lambda : self.close_windows)

        self.origin = Point2D(0, 0)
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        self.mergeWristElbow = False

        self.dynamicMode = dynamicMode
        self.approachFromUp = approachFromUp
        self.wristApproachAngle = wristApproachAngle
        self.mappedApproachPair = mappedApproachPair
        self.currentPostion = cylindericalPoint(0 , 10 , 10) #TODO:to be revised and initizalized

        self.base_angle = 0
        self.shoulder_angle = 0
        self.elbow_angle = 0

        self.visualization = visualization

        if self.visualization:
            self.root = tk.Tk()
            self.simulator = RobotArmSimulator(self.root , self)
            self.root.after(0, self.run_visualization)
            self.wait_until_window_ready()

    def close_windows(self):
        if self.visualization:
            self.root.destroy()
            plt.close('all')  # Close all Matplotlib plots

    def run_visualization(self):
        self.root.update()
        if not self.simulator.closed:
            self.root.after(10, self.run_visualization)
        else:
            self.root.destroy()
    
    def wait_until_window_ready(self):
        self.root.update_idletasks()  # Update the Tkinter window
        self.root.update()  # Process any pending events to ensure the window is ready

    def handlePoint(self, point):
        if point.magnitude() > self.shoulder_length + self.elbow_length + self.wrist_length * self.dynamicMode - delta:
            point.changeMagnitude(self.shoulder_length + self.elbow_length + self.wrist_length * self.dynamicMode - delta)
        return point

    def dof3_constantAngleApproach(self, point, angle=0):
        newPoint = Point2D.copy(point)
        deltax = self.wrist_length * cos(radians(angle))
        deltay = self.wrist_length * sin(radians(angle))

        newPoint.x -= deltax
        newPoint.y -= deltay
        return newPoint, radians(angle)

    def dof3_mappedAngleApproach(self, point, approachPair=None):
        self.mergeWristElbow = False
        if approachPair is None:
            approachPair = self.mappedApproachPair

        pointDirection = degrees(atan2(point.y, point.x))
        
        m = (pointDirection - approachPair[0]) / ((self.shoulder_length + self.elbow_length + self.wrist_length - delta) - approachPair[1])
        c = approachPair[0] - approachPair[1] * m

        distance = point.magnitude()
        mappedAngle = m * distance + c

        if distance < approachPair[1]:
            self.mergeWristElbow = True
            return point, 0
        return self.dof3_constantAngleApproach(point, mappedAngle)

    def dof3_Handler(self, point):
        if self.dynamicMode:
            return self.dof3_mappedAngleApproach(point)
        return self.dof3_constantAngleApproach(point, angle=self.wristApproachAngle)

    def handle_negative_beta(self, phi, theta):
        if (phi + theta < phi - theta) ^ self.approachFromUp:
            return phi - theta
        return phi + theta

    def inverseKinematics(self, point):
        point = self.handlePoint(point)
        point , motor3 = self.dof3_Handler(point)

        a = point.x
        z = point.y

        phi = atan2(z , a)
        L = sqrt(a ** 2 + z ** 2)

        tooNearCondition = self.dynamicMode and self.mergeWristElbow

        # unequal condition
        cos_theta_nom = self.shoulder_length ** 2 + L ** 2 - (self.elbow_length + self.wrist_length * tooNearCondition) ** 2
        cos_theta = cos_theta_nom / (2 * self.shoulder_length * L)
        theta = acos(cos_theta)

        # # equal condition
        # theta = acos(L/2/Robot.shoulder_length)

        beta_1 = self.handle_negative_beta(phi , theta)
        
        # unequal condition
        tan_beta_2_nom = (z - self.shoulder_length * sin(beta_1))
        tan_beta_2_den = (a - self.shoulder_length * cos(beta_1))
        beta_2 = atan2(tan_beta_2_nom , tan_beta_2_den) 

        # # equal condition
        # beta_2 = phi - theta

        motor1 = beta_1
        motor2 = beta_2 - motor1
        motor3 = motor3 - motor2 - motor1

        if(tooNearCondition):
            motor3 = 0

        return motor1 , motor2 , motor3

    def goto(self, theta , x, y, theta_out = None, theta_in = None, mag_out = None, mag_in = None, steps = 50, t_step = 100):
        rate = rospy.Rate(1000/t_step)
        
        point_0 = self.currentPostion.toList()[1:]
        point_f = [x, y]

        if(mag_out == None):
            if(theta_out != None):
                mag_out = sqrt((point_f[0] - point_0[0])**2 + (point_f[1] - point_0[1])**2) * 0.33
            else:
                mag_out = 0

        if(mag_in == None):
            if(theta_in != None):
                mag_in = sqrt((point_f[0] - point_0[0])**2 + (point_f[1] - point_0[1])**2) * 0.33
            else:
                mag_in = 0

        if(theta_out == None):
            theta_out = atan2(point_f[1] - point_0[1] , point_f[0] - point_0[0])

        if(theta_in == None):
            theta_in = atan2(point_0[1] - point_f[1] , point_0[0] - point_f[0])


        mag_ang_f = [mag_in, theta_in]
        mag_ang_0 = [mag_out, theta_out]

        c = [point_0, None, None, point_f]
        c[1] = PathGenerator.tangentPoint(c[0], magnitude=mag_ang_0[0], angle=mag_ang_0[1])
        c[2] = PathGenerator.tangentPoint(c[3], magnitude=mag_ang_f[0], angle=mag_ang_f[1])

        pathTimeStamps = PathGenerator.bellSegmenter(steps)
        Bezier_curve = [PathGenerator.Bezier_curve(timestamp, c) for timestamp in pathTimeStamps]
        
        if self.visualization:
            self.simulator.canvas.delete("target")
            PathGenerator.graph2DList(Bezier_curve)

        for i , point in enumerate(Bezier_curve):
            p = Point2D(point[0], -point[1])
            angles = self.inverseKinematics(p)

            self.base_angle = angles[0]
            self.shoulder_angle = angles[1]
            self.elbow_angle = angles[2]

            if(self.visualization):
                self.simulator.draw_arm()
                self.simulator.draw_dot_atPixel(Point2D(point[0] + self.simulator.origin.x, -point[1] + self.simulator.origin.y) , tags="target" , dot_size=2)
                self.simulator.canvas.update()
            
            theta_i = self.currentPostion.theta * (1 - pathTimeStamps[i]) + theta * (pathTimeStamps[i])
            # published_Angles = [ theta_i , -(pi/2) - angles[0], (angles[1] - (pi/2)), 0, (-angles[2] + pi/2)]
            published_Angles = [theta_i, pi/2 + angles[0], -pi/2 + angles[1], 0, pi/2+angles[2]] #gazebo
            joint_state = ArmJointTrajectory()
            joint_state_point = JointTrajectoryPoint()
            joint_state_point.positions = published_Angles
            joint_state_point.time_from_start = rospy.Duration(t_step)  # Set the duration for the trajectory point
            joint_state.points.append(joint_state_point)  # Append the trajectory point to the list of points
            self.Gazebo_publisher.publish(joint_state)
        
            published_Angles = [theta_i, pi/2 + angles[0], -pi/2 + angles[1], 0, pi/2+angles[2]]  #arduino
            int16_array = Int16MultiArray()
            int16_array.data = [int(degrees(angle)) for angle in published_Angles]
            self.arduino_publisher.publish(int16_array)
            
            rospy.loginfo(f"currently @ theta : {theta_i} , x : {p.x} , y : {p.y}")
            rate.sleep()

        self.currentPostion.theta = theta
        self.currentPostion.x = x
        self.currentPostion.y = y

class RobotArmSimulator:
    def __init__(self, master, robot , width = 800 , height = 600 , grid_size = 20):
        self.closed = False
        self.robot = robot
        self.width = width
        self.height = height
        self.grid_size = grid_size

        self.endEffectorPostion = None
    
        self.canvas = tk.Canvas(master, width=self.width, height=self.height)
        self.canvas.pack()

        self.origin = Point2D(self.width//2, self.height//2)

        self.draw_grid()
        self.draw_arm()

    def draw_grid(self):
        for i in range(0, self.height + 1, self.grid_size):
            self.canvas.create_line(0, i, self.width, i, fill="gray")
        for i in range(0 , self.width + 1, self.grid_size):
            self.canvas.create_line(i, 0, i, self.height, fill="gray")

    def pixelToGrid(self, x):
        return x / self.grid_size

    def gridToPixel(self, x):
        return x * self.grid_size

    def GridPoint_to_PixelPoint_Relative_To_Origin(self, point):
        point.apply(self.gridToPixel)
        point.y = -point.y
        return point

    def PixelPoint_to_GridPoint(self, point):
        gridPoint = Point2D.copy(point)
        gridPoint.x = self.pixelToGrid(point.x - self.origin.x)
        gridPoint.y = self.pixelToGrid(self.origin.y - point.y)
        return gridPoint

    def reverse_PixelPoint_to_GridPoint(self, point):
        pixelPoint = Point2D.copy(point)
        pixelPoint.x = self.origin.x + self.gridToPixel(point.x)
        pixelPoint.y = self.origin.y - self.gridToPixel(point.y)
        return pixelPoint

    def draw_dot_atPixel(self, point, dot_size=5, dot_color="red", tags=None):
        self.canvas.create_oval(point.x - dot_size, point.y - dot_size, point.x + dot_size, point.y + dot_size, fill=dot_color, tags=tags)

    def draw_dot_atGrid(self, gridPoint, dot_size=5, dot_color="red", tags=None):
        self.draw_dot_atPixel(self.reverse_PixelPoint_to_GridPoint(gridPoint), dot_size=dot_size, dot_color=dot_color, tags=tags)

    def draw_arm(self):
        self.canvas.delete("arm")
        base = self.reverse_PixelPoint_to_GridPoint(self.robot.origin)
        base_x = base.x
        base_y = base.y

        shoulder_length = self.robot.shoulder_length
        elbow_length = self.robot.elbow_length
        wrist_length = self.robot.wrist_length

        shoulder_x = base_x + shoulder_length * cos(self.robot.base_angle)
        shoulder_y = base_y + shoulder_length * sin(self.robot.base_angle)

        elbow_x = shoulder_x + elbow_length * cos(self.robot.base_angle + self.robot.shoulder_angle)
        elbow_y = shoulder_y + elbow_length * sin(self.robot.base_angle + self.robot.shoulder_angle)

        end_x = elbow_x + wrist_length * cos(self.robot.base_angle + self.robot.shoulder_angle + self.robot.elbow_angle)
        end_y = elbow_y + wrist_length * sin(self.robot.base_angle + self.robot.shoulder_angle + self.robot.elbow_angle)
        
        self.canvas.create_line(base_x, base_y, shoulder_x, shoulder_y, tags="arm", width=7)
        self.canvas.create_line(shoulder_x, shoulder_y, elbow_x, elbow_y, tags="arm", width=5)
        self.canvas.create_line(elbow_x, elbow_y, end_x, end_y, tags="arm", width=3)

        self.draw_dot_atPixel(Point2D(shoulder_x, shoulder_y), 6, "blue", tags="arm")
        self.draw_dot_atPixel(Point2D(elbow_x, elbow_y), 4, "blue", tags="arm")

        endEffector = Point2D(end_x, end_y)
        self.endEffectorPostion = self.PixelPoint_to_GridPoint(endEffector)


