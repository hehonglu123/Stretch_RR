import termios, fcntl, sys, os, time
from RobotRaconteur.Client import *  
url='rr+tcp://192.168.1.64:23232/?service=stretch'

robot=RRN.ConnectService(url)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()
base=robot.get_base()



#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

#TODO: Initialize ROS/RR node
#ROS: create publisher to publish Twist message to corresponding topic name

#RR: connect to service with url

print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
try:
    #TODO: hold the script running with ROS/RR way
    while True:
        try:
            #read input and print "command"
            c = sys.stdin.read()
            #TODO: ROS create message type variable, publish command
            #TODO: RR call drive function
            if "\x1b[A" in c:
                print("drive forward")          ####Drive forward
                base.translate_by(0.05)
            if "\x1b[B" in c:
                print("drive backward")         ####Drive backward  
                base.translate_by(-0.05)             
            if "\x1b[C" in c:
                print("drive right")            ####Drive right
                base.rotate_by(-0.1)
            if "\x1b[D" in c:
                print("drive left")             ####Drive left
                base.rotate_by(0.1)
            if "w" in c:
                print("lift up")             
                lift.move_by(0.02)
            if "s" in c:
                print("lift down")             
                lift.move_by(-0.02)
            if "a" in c:
                print("arm retract")           
                arm.move_by(-0.02)
            if "d" in c:
                print("arm stretch")         
                arm.move_by(0.02)
            if "q" in c:
                break
            robot.push_command()
            time.sleep(0.1)

        except IOError: pass
        except TypeError: pass
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)