#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

import RTIMU
import os
import math
import time

path=os.path.join("/home/pi/Desktop/RTIMULib/RTIMULib")

msg=Float32MultiArray()

SETTINGS_FILE = path + "RTIMULib"


def to_360_deg(arr):
    arr=list(arr)
    for i in range(len(arr)):
        if arr[i]<0:
            arr[i]=360+arr[i]
    return arr


def degree_convertion(arr):
    arr=list(arr)
    for i in range(len(arr)):
        arr[i]=math.degrees(arr[i])

    return arr


def main():
	print("Initializig imu publisher :)")
	rospy.init_node('ImuValuesPub')
	pub= rospy.Publisher('/axo/hardware/imu_fusion_pose',Float32MultiArray, queue_size=1)
	loop=rospy.Rate(10)

	print("Using settings file " + SETTINGS_FILE + ".ini")
	if not os.path.exists(SETTINGS_FILE + ".ini"):
	    print("Settings file does not exist, will be created")

	s = RTIMU.Settings(SETTINGS_FILE)
	imu = RTIMU.RTIMU(s)

	print("IMU Name: " + imu.IMUName())

	if (not imu.IMUInit()):
	    print("IMU Init Failed")
	    imu_conected=False
	else:
	    print("IMU Init Succeeded")

	# this is a good time to set any fusion parameters

	imu.setSlerpPower(0.02)
	imu.setGyroEnable(True)
	imu.setAccelEnable(True)
	imu.setCompassEnable(True)

	poll_interval = imu.IMUGetPollInterval()
	print("Recommended Poll Interval: %dmS\n" % poll_interval)
	poll_interval=poll_interval*1.0/1000.0
	print "reading"
	if imu.IMURead():
		data = imu.getIMUData()
	time.sleep(poll_interval)


	while not rospy.is_shutdown():
		if imu.IMURead():
			# x, y, z = imu.getFusionData()
			# print("%f %f %f" % (x,y,z))
			data = imu.getIMUData()
			fusionPose = data["fusionPose"]
			#print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), 
			   #math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
			compass = data["compass"]
			#print("X: %f Y: %f Z: %f" % (math.degrees(compass[0]), 
			    #math.degrees(compass[1]), math.degrees(compass[2])))
			accel = data["accel"]
			#print("X: %f Y: %f Z: %f" % (math.degrees(accel[0]), 
			    #math.degrees(accel[1]), math.degrees(accel[2])))
			gyro = data["gyro"]
			#print("X: %f Y: %f Z: %f" % (math.degrees(gyro[0]), 
			    #math.degrees(gyro[1]), math.degrees(gyro[2])))
			All=accel+gyro+compass
			#print"degrees"
			fusionPose=degree_convertion(fusionPose)
			#print fusionPose
			#print"360"
			fusionPose=to_360_deg(fusionPose)
			#print fusionPose
			
		msg.data=fusionPose
		pub.publish(msg)
		loop.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


