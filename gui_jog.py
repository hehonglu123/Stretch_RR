#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback
from tkinter import *
from tkinter import messagebox
import numpy as np


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()


url='rr+tcp://192.168.1.64:23232/?service=stretch'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()
base.robot.get_base()


top=Tk()
top.title(args.robot_name)
jobid = None


def move_lift(vd):
	global jobid
	try:
		lift.move_by(vd)
		lift.push_command()
		jobid = top.after(10, lambda: move_lift(vd))
	except:
		traceback.print_exc()
	return
def move_arm(vd):
	global jobid
	try:
		arm.move_by(vd)
		arm.push_command()
		jobid = top.after(10, lambda: move_arm(vd))
	except:
		traceback.print_exc()
	return
def move_wrist(vd):
	global jobid
	try:
		end_of_arm.move_by('wrist_yaw',vd)
		end_of_arm.push_command()
		jobid = top.after(10, lambda: move_wrist(vd))
	except:
		traceback.print_exc()
	return
def move_gripper(vd):
	global jobid
	try:
		end_of_arm.move_by('stretch_gripper',vd)
		end_of_arm.push_command()
		jobid = top.after(10, lambda: move_gripper(vd))
	except:
		traceback.print_exc()
	return

def move_base(vd,wd):
	global jobid
	try:
		if vd!=0.:
			base.translate_by(vd)
		if wd!=0.:
			base.rotate_by(wd)
		base.push_command()
		jobid = top.after(10, lambda: move_base(vd))
	except:
		traceback.print_exc()
	return


lift_up=Button(top,text='lift+')
lift_down=Button(top,text='lift-')
arm_up=Button(top,text='arm+')
arm_down=Button(top,text='arm-')
wrist_up=Button(top,text='wrist+')
wrist_down=Button(top,text='wrist-')
gripper_up=Button(top,text='gripper+')
gripper_down=Button(top,text='gripper-')
base_forward=Button(top,text='base+')
base_backward=Button(top,text='base-')
base_ccl=Button(top,text='base_r+')
base_cl=Button(top,text='base_r-')

lift_up.bind('<ButtonPress-1>', lambda event: move_lift(0.005)
lift_down.bind('<ButtonPress-1>', lambda event: move_lift(-0.005)
arm_up.bind('<ButtonPress-1>', lambda event: move_arm(0.005))
arm_down.bind('<ButtonPress-1>', lambda event: move_arm(-0.005))
wrist_up.bind('<ButtonPress-1>', lambda event: move_wrist(0.005)
wrist_down.bind('<ButtonPress-1>', lambda event: move_wrist(-0.005)
gripper_up.bind('<ButtonPress-1>', lambda event: move_gripper(0.005))
gripper_down.bind('<ButtonPress-1>', lambda event: move_gripper(-0.005))
base_forward.bind('<ButtonPress-1>', lambda event: move_base(0.01,0))
base_backward.bind('<ButtonPress-1>', lambda event: move_base(-0.01,0))
base_ccl.bind('<ButtonPress-1>', lambda event: move_base(0.0,0.1))
base_cl.bind('<ButtonPress-1>', lambda event: move_base(0.0,-0.1))



lift_up.pack()
lift_down.pack()
arm_up.pack()
arm_down.pack()
wrist_up.pack()
wrist_down.pack()
gripper_up.pack()
gripper_down.pack()



top.mainloop()
