
from RobotRaconteur.Client import *     #import RR client library
import sys, time, traceback
import numpy as np
url='rr+tcp://192.168.1.64:23232/?service=stretch'

#Startup, connect, and pull out different objects from robot object   
robot=RRN.ConnectService(url)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()
base=robot.get_base()

#Connect to arm status RR wire
arm_status=arm.status_rr.Connect()
arm_motor_status=arm.motor_status_rr.Connect()

#Go to initial position first
lift.move_to(0.5) 
arm.move_to(0.3)
end_of_arm.move_to('wrist_yaw',0)
end_of_arm.move_to('stretch_gripper',25)		##(50 to -100)
robot.push_command()
time.sleep(3)
now=time.time()
while True:
	try:
		#Command joint to move sinusoidal motion
		lift.move_to(0.5+0.2*np.sin((time.time()-now)/2.))
		arm.move_to(0.3+0.28*np.cos((time.time()-now)/2.))
		end_of_arm.move_to('wrist_yaw',np.sin((time.time()-now)/2.))
		end_of_arm.move_to('stretch_gripper',25+25*np.sin((time.time()-now)/2.))
		base.translate_by(0.01*np.sin((time.time()-now)/2.))
		robot.push_command()
		#print force reading from arm
		print('arm force: ',arm_status.InValue['force'])
		print('arm motor current ', arm_motor_status.InValue['current'])
		time.sleep(0.05)
	except:
		traceback.print_exc()
		break

print ('Retracting...')
lift.move_to(0.0)
arm.move_to(0.0)
end_of_arm.move_to('wrist_yaw',0.)
end_of_arm.move_to('stretch_gripper',0.)
robot.push_command( )
time.sleep(3)