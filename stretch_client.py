
from RobotRaconteur.Client import *     #import RR client library
import sys, time

url='rr+tcp://localhost:23232/?service=stretch'

#Startup, connect, and pull out the camera from the objref    
robot=RRN.ConnectService(url)
lift=robot.get_lift()

lift.move_to(0.8) #Reach all the way out
robot.push_command()

time.sleep(5)
# while lift.status['pos']<0.5:
#     print (lift.status['pos'])
#     time.sleep(0.1)

print 'Retracting...'
lift.move_to(0.0)
robot.push_command( )