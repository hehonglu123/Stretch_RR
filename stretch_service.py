import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import time, copy
from stretch_body.robot import Robot

###
#### python2.7 RR depreciate
# # 1. set_ reserved, need overwritten
####base
# 	function 	void			set_translate_velocity(double v_m)
# 	function 	void			set_rotational_velocity(double v_r)
# 	function 	void			set_velocity(double v_m, double w_r)
####wacc
# 	function 	void			set_D2(uint8 on)
# 	function 	void			set_D3(uint8 on)
####stretch
# 	function 	stretch_status	get_status()
# # 2. would be nice to have logs on client side
# # 3. RR request timeout for home()
# # 4. ALL status dict needs overwritten

class stretch_RR(Robot):
	def __init__(self):
		Robot.__init__(self)
		self.arm
	def get_arm(self):
		return self.arm, "edu.rpi.robotics.stretch.stretch_arm"
	def get_base(self):
		return self.base, "edu.rpi.robotics.stretch.stretch_base"
	def get_head(self):
		return self.head, "edu.rpi.robotics.stretch.stretch_head"
	def get_wacc(self):
		return self.wacc, "edu.rpi.robotics.stretch.stretch_wacc"
	def get_lift(self):
		return self.lift, "edu.rpi.robotics.stretch.stretch_lift"

def main():
	robot=stretch_RR()
	robot.startup()
	#auto homing
	if not robot.is_calibrated():
		robot.home()
	with RR.ServerNodeSetup("Stretch_Node", 23232) as node_setup:

		#Register Service types
		RRN.RegisterServiceTypeFromFile('robdef/edu.rpi.robotics.stretch')
		#create object
		RRN.RegisterService("stretch", "edu.rpi.robotics.stretch.stretch", robot)
		

		input("Press enter to quit")

		robot.stop()




if __name__ == "__main__":
	main()