#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
cur_pose = (0, 0, 0)
ready = False
ready2 = False
def get_yaw_from_quarternion(q):
	siny_cosp = 2* (q.w*q.z + q.x*q.y)
	cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
	yaw = math.atan(siny_cosp/cosy_cosp)
	return yaw

def callback(odom_data):
	global cur_pose, ready
	point = odom_data.pose.pose.position
	quart = odom_data.pose.pose.orientation
	theta = get_yaw_from_quarternion(quart)
	cur_pose = (point.x, point.y, theta)
	ready = True
	#rospy.loginfo(cur_pose)


def main():
	global cur_pose, ready, ready2
	try:
		rospy.init_node('odometery')
		cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		odom_subscriber = rospy.Subscriber('odom', Odometry, callback, queue_size=1)
		while not ready:
			print('waiting')	
		rate = rospy.Rate(10)
		twist = Twist()
		p1 = cur_pose
		#go 1 meter
		twist.linear.x = 0.3
		while not rospy.is_shutdown():
			cmd_pub.publish(twist)
			print("cur_pose[1]:" + str(cur_pose[1]) + " p1[1]:" + str(p1[1]) + "\n\r")
			if abs(cur_pose[1] - p1[1]) > 0.1:
				break
			rate.sleep()
		
		print('spinning')
		twist.linear.x = 0.0
		twist.angular.z = 0.3
		cur_or = cur_pose
		while not rospy.is_shutdown():
			cmd_pub.publish(twist)
			print(cur_or[2], cur_pose[2])
			if abs(cur_or[2] - cur_pose[2]) > 0.2*3.14:
				ready2 = True
			if abs(cur_or[2] - cur_pose[2]) < 0.1 and ready2:
				break
			rate.sleep()
		cur_or = cur_pose
		ready2 = False
                while not rospy.is_shutdown():
                        cmd_pub.publish(twist)
                        print(cur_or[2], cur_pose[2])
                        if abs(cur_or[2] - cur_pose[2]) > 0.2*3.14:
                                ready2 = True
                        if abs(cur_or[2] - cur_pose[2]) < 0.1 and ready2:
                                break
                        rate.sleep()

                twist.linear.x = 0.0
                twist.angular.z = 0.0
		cmd_pub.publish(twist)
	except rospy.ROSInterruptException:
		pass
	

if __name__ == '__main__':
	main()
