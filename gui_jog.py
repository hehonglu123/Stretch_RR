#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, threading
from tkinter import *
from tkinter import messagebox
import numpy as np


#Accept the names of the webcams and the nodename from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str,help="List of camera names separated with commas")
args, _ = parser.parse_known_args()


url='rr+tcp://192.168.50.107:23232/?service=stretch'
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
lift=robot.get_lift()
arm=robot.get_arm()
end_of_arm=robot.get_end_of_arm()
base=robot.get_base()
arm_status=arm.status_rr.Connect()
lift_status=lift.status_rr.Connect()
base_status=base.status_rr.Connect()



top=Tk()
top.title(args.robot_name)
jobid = None


def move_lift(vd):
	global jobid1
	try:
		lift.move_by(vd)
		robot.push_command()
		jobid1 = top.after(100, lambda: move_lift(vd))
	except:
		traceback.print_exc()
	return
def move_arm(vd):
	global jobid2
	try:
		arm.move_by(vd)
		robot.push_command()
		jobid2 = top.after(100, lambda: move_arm(vd))
	except:
		traceback.print_exc()
	return
def move_wrist(vd):
	global jobid3
	try:
		end_of_arm.move_by('wrist_yaw',vd)
		jobid3 = top.after(10, lambda: move_wrist(vd))
	except:
		traceback.print_exc()
	return
def move_gripper(vd):
	global jobid4
	try:
		end_of_arm.move_by('stretch_gripper',vd)
		jobid4 = top.after(10, lambda: move_gripper(vd))
	except:
		traceback.print_exc()
	return

def move_base(vd,wd):
	global jobid5
	try:
		if vd!=0.:
			base.translate_by(vd)
		if wd!=0.:
			base.rotate_by(wd)
		robot.push_command()
		jobid5 = top.after(100, lambda: move_base(vd,wd))
	except:
		traceback.print_exc()
	return

def stop(jobid):
	top.after_cancel(jobid)
	return

def update_label():
	joint_text='lift force: '+str(lift_status.InValue['force'])+ "\n"
	joint_text+='arm force: '+str(arm_status.InValue['force'])+ "\n"
		

	label.config(text=joint_text)

	label.after(250, update_label)

label = Label(top, fg = "black", justify=LEFT)
label.pack()
label.after(250,update_label)




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

lift_up.bind('<ButtonPress-1>', lambda event: move_lift(0.01))
lift_down.bind('<ButtonPress-1>', lambda event: move_lift(-0.01))
arm_up.bind('<ButtonPress-1>', lambda event: move_arm(0.01))
arm_down.bind('<ButtonPress-1>', lambda event: move_arm(-0.01))
wrist_up.bind('<ButtonPress-1>', lambda event: move_wrist(0.05))
wrist_down.bind('<ButtonPress-1>', lambda event: move_wrist(-0.05))
gripper_up.bind('<ButtonPress-1>', lambda event: move_gripper(1))
gripper_down.bind('<ButtonPress-1>', lambda event: move_gripper(-1))
base_forward.bind('<ButtonPress-1>', lambda event: move_base(0.01,0))
base_backward.bind('<ButtonPress-1>', lambda event: move_base(-0.01,0))
base_ccl.bind('<ButtonPress-1>', lambda event: move_base(0.0,0.1))
base_cl.bind('<ButtonPress-1>', lambda event: move_base(0.0,-0.1))

lift_up.bind('<ButtonRelease-1>', lambda event: stop(jobid1))
lift_down.bind('<ButtonRelease-1>', lambda event: stop(jobid1))
arm_up.bind('<ButtonRelease-1>', lambda event: stop(jobid2))
arm_down.bind('<ButtonRelease-1>', lambda event: stop(jobid2))
wrist_up.bind('<ButtonRelease-1>', lambda event: stop(jobid3))
wrist_down.bind('<ButtonRelease-1>', lambda event: stop(jobid3))
gripper_up.bind('<ButtonRelease-1>', lambda event: stop(jobid4))
gripper_down.bind('<ButtonRelease-1>', lambda event: stop(jobid4))
base_forward.bind('<ButtonRelease-1>', lambda event: stop(jobid5))
base_backward.bind('<ButtonRelease-1>', lambda event: stop(jobid5))
base_ccl.bind('<ButtonRelease-1>', lambda event: stop(jobid5))
base_cl.bind('<ButtonRelease-1>', lambda event: stop(jobid5))


lift_up.pack()
lift_down.pack()
arm_up.pack()
arm_down.pack()
wrist_up.pack()
wrist_down.pack()
gripper_up.pack()
gripper_down.pack()

base_forward.pack()
base_backward.pack()
base_ccl.pack()
base_cl.pack()

top.mainloop()
