from arm import Robot
import rospy

if __name__ == '__main__':
    while not rospy.is_shutdown():
        robot = Robot(visualization=True)
        robot.goto(theta = 0 , x = 100 , y = 75 , theta_out = 90 , steps=50, t_step=100) #theta in rad
        robot.goto(theta = 0 , x = 50 ,  y= 75 , theta_out = 90 ,steps=50, t_step=100)
        robot.goto(theta = 0 , x = 50 , y = 25 , theta_out = 90 ,steps=50, t_step=100)
        robot.goto(theta = 0 , x = 100 , y = 100 , theta_out = 90, steps=50 ,t_step=100)
        while True:
            pass