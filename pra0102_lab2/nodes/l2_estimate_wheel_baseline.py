#!/usr/bin/env python

import rospy
import numpy as np
import threading
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

INT32_MAX = 2**31
NUM_ROTATIONS = 3 
TICKS_PER_ROTATION = 4096
WHEEL_RADIUS = 0.066 / 2 #In meters

left_encoders = 1
right_encoders = 1
old_left_encoders = -5000
old_right_encoders = -5000

delta_left = 0
delta_right = 0

b = -1



class wheelBaselineEstimator():
    def __init__(self):
        rospy.init_node('encoder_data', anonymous=True) # Initialize node

        #Subscriber bank
        rospy.Subscriber("cmd_vel", Twist, self.startStopCallback)
        rospy.Subscriber("sensor_state", SensorState, self.sensorCallback) #Subscribe to the sensor state msg

        #Publisher bank
        self.reset_pub = rospy.Publisher('reset', Empty, queue_size=1)

        #Initialize variables
        self.left_encoder_prev = None
        self.right_encoder_prev = None
        self.del_left_encoder = 0
        self.del_right_encoder = 0
        self.isMoving = False #Moving or not moving
        self.lock = threading.Lock()

        #Reset the robot 
        reset_msg = Empty()
        self.reset_pub.publish(reset_msg)
        print('Ready to start wheel radius calibration!')
        return

    def safeDelPhi(self, a, b):
        #Need to check if the encoder storage variable has overflowed
        diff = np.int64(b) - np.int64(a)
        if diff < -np.int64(INT32_MAX): #Overflowed
            delPhi = (INT32_MAX - 1 - a) + (INT32_MAX + b) + 1
        elif diff > np.int64(INT32_MAX) - 1: #Underflowed
            delPhi = (INT32_MAX + a) + (INT32_MAX - 1 - b) + 1
        else:
            delPhi = b - a  
        return delPhi

    def sensorCallback(self, msg):
	global delta_left, delta_right
        #Retrieve the encoder data form the sensor state msg
        self.lock.acquire()

        #YOUR CODE HERE!
	if old_left_encoders is -5000 or old_right_encoders is -5000:
		old_left_econders = msg.left_encoder
		old_right_econders = msg.right_encoder
	else:
		left_econders = msg.left_encoder
		right_econders = msg.right_encoder

	delta_left = delta_left+left_encoders - old_left_encoders
	delta_right = delta_right+right_encoders - old_right_encoders

        # Accumulate the encoder ticks here
        
        self.lock.release()
        return

    def startStopCallback(self, msg):
        if self.isMoving is False and np.absolute(msg.angular.z) > 0:
            self.isMoving = True #Set state to moving
            print('Starting Calibration Procedure')

        elif self.isMoving is True and np.isclose(msg.angular.z, 0):
            self.isMoving = False #Set the state to stopped
            self.lock.acquire()
            
            #YOUR CODE HERE!
            #Calculate the wheel baseline here
	    b = (0.033/2)*(delta_right - delta_left)/(2*3.1415926*4)

            #Reset the robot and calibration routine
            self.left_encoder_prev = None
            self.right_encoder_prev = None
            self.del_left_encoder = 0
            self.del_right_encoder = 0
            self.lock.release()
            reset_msg = Empty()
            self.reset_pub.publish(reset_msg)
            print(b)
            print('Resetted the robot to calibrate again!')

        return


if __name__ == '__main__':
    Estimator = wheelBaselineEstimator() #create instance
    rospy.spin()
