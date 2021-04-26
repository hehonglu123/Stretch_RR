import termios, fcntl, sys, os, time
from RobotRaconteur.Client import *  
url='rr+tcp://192.168.50.107:23232/?service=stretch'

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



print("Running")
print("Press Arrow Key to Control Stretch")
print("Press q to quit")
try:
    while True:
        try:

            #read input and print "command"
            c = sys.stdin.read()
            if "\x1b[A" in c:
                print("drive forward")          ####Drive forward
                base.r_set_translate_velocity(0.2)
            if "\x1b[B" in c:
                print("drive backward")         ####Drive backward  
                base.r_set_translate_velocity(-0.2)
         
            if "\x1b[C" in c:
                print("rotate right")            ####Drive right
                base.r_set_rotational_velocity(0.1)
            if "\x1b[D" in c:
                print("rotate left")             ####Drive left
                base.r_set_rotational_velocity(-0.1)

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
            

        except IOError: pass
        except TypeError: base.r_set_velocity(0,0)
        robot.push_command()
        time.sleep(0.05)
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)