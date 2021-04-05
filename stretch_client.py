
from RobotRaconteur.Client import *     #import RR client library
import sys, time
import numpy as np
url='rr+tcp://localhost:23232/?service=stretch'

#Startup, connect, and pull out the camera from the objref    
robot=RRN.ConnectService(url)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()

lift.move_to(0.5) #Reach all the way out
arm.move_to(0.3)
end_of_arm.move_to('wrist_yaw',0)
end_of_arm.move_to('stretch_gripper',25)		##(50 to -100)
robot.push_command()
time.sleep(3)
now=time.time()
while True:
	try:
		lift.move_to(0.5+0.2*np.sin((time.time()-now)/2.))
		arm.move_to(0.3+0.28*np.cos((time.time()-now)/2.))
		end_of_arm.move_to('wrist_yaw',np.sin((time.time()-now)/2.))
		end_of_arm.move_to('stretch_gripper',25+25*np.sin((time.time()-now)/2.))
		robot.push_command()
		time.sleep(0.05)
	except:
		break

print 'Retracting...'
lift.move_to(0.0)
arm.move_to(0.0)
end_of_arm.move_to('wrist_yaw',0.)
end_of_arm.move_to('stretch_gripper',0.)
robot.push_command( )
time.sleep(3)