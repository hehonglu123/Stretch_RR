
from RobotRaconteur.Client import *     #import RR client library
import sys, time, traceback
import numpy as np

#url of each robot
url1='rr+tcp://192.168.1.64:23232/?service=stretch'
url2='rr+tcp://192.168.1.28:23232/?service=stretch'

#Startup, connect, and pull out the arm/lift/eoa from the objref in robot obj  
robot1=RRN.ConnectService(url1)
robot2=RRN.ConnectService(url2)

lift1=robot1.get_lift()
arm1=robot1.get_arm()
end_of_arm1=robot1.get_end_of_arm()
lift2=robot2.get_lift()
arm2=robot2.get_arm()
end_of_arm2=robot2.get_end_of_arm()

lift1.move_to(0.5) #Reach all the way out
arm1.move_to(0.3)
end_of_arm1.move_to('wrist_yaw',0)
end_of_arm1.move_to('stretch_gripper',25)		##(50 to -100)
lift2.move_to(0.5) #Reach all the way out
arm2.move_to(0.3)
end_of_arm2.move_to('wrist_yaw',0)
end_of_arm2.move_to('stretch_gripper',25)		##(50 to -100)


robot1.push_command()
robot2.push_command()


#Connect to arm status RR wire
arm1_status=arm1.status_rr.Connect()
arm2_status=arm2.status_rr.Connect()

time.sleep(3)
now=time.time()
while True:
	try:
		now2=time.time()
		lift1.move_to(0.5+0.2*np.sin((time.time()-now)/2.))
		arm1.move_to(0.3+0.28*np.cos((time.time()-now)/2.))
		end_of_arm1.move_to('wrist_yaw',np.sin((time.time()-now)/2.))
		end_of_arm1.move_to('stretch_gripper',25+25*np.sin((time.time()-now)/2.))
		

		lift2.move_to(0.5+0.2*np.sin((time.time()-now)/2.))
		arm2.move_to(0.3+0.28*np.cos((time.time()-now)/2.))
		end_of_arm2.move_to('wrist_yaw',np.sin((time.time()-now)/2.))
		end_of_arm2.move_to('stretch_gripper',25+25*np.sin((time.time()-now)/2.))

		robot1.push_command()
		robot2.push_command()
		#example to read RR status wire
		print('arm1 pos: '+str(arm1_status.InValue['pos']))
		print('arm2 pos: '+str(arm2_status.InValue['pos']))
		#timing test
		print(time.time()-now2)
		# time.sleep(0.02)
	except:
		traceback.print_exc()
		break

print ('Retracting...')
lift1.move_to(0.0)
arm1.move_to(0.0)
end_of_arm1.move_to('wrist_yaw',0.)
end_of_arm1.move_to('stretch_gripper',0.)

lift2.move_to(0.0)
arm2.move_to(0.0)
end_of_arm2.move_to('wrist_yaw',0.)
end_of_arm2.move_to('stretch_gripper',0.)

robot1.push_command( )
robot2.push_command( )

time.sleep(3)