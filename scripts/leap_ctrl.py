#!/usr/bin/env python
import os, rospy, copy, time, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class Run():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors',LightSensorValues,self.callback)

    def callback(self,messages):
        self.sensor_values = messages

    def wait_for_operation(self):

        while not rospy.is_shutdown():

            time.sleep(0.5)

            filepath = "/tmp/opval"
            if os.path.exists(filepath):
                with open(filepath,"r") as f:
                    opval = int(f.readline().rstrip())

                if opval == 0: # stop
                    self.stop()
                elif opval == 1: # run
                    self.run(0.08,   0)
                elif opval == 2: # turn to right
                    self.run(0.05, -math.pi/4.0)
                elif opval == 3: # turn to left
                    self.run(0.05,  math.pi/4.0)
                else:
                    self.stop()
            else:
                opval = 0

    def stop(self):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = 0.0
        data.angular.z = 0.0
        self.cmd_vel.publish(data)
        rate.sleep()

    def run(self, speed, rotate):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = speed
        data.angular.z = rotate

        s = self.sensor_values
        if s.sum_forward >= 50:
            data.linear.x = 0.0

        self.cmd_vel.publish(data)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('leap_ctrl')

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    Run().wait_for_operation()


