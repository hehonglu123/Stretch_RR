import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import time, copy
from stretch_body.robot import Robot
from stretch_body.base import Base
from stretch_body.arm import Arm
from stretch_body.lift import Lift
from stretch_body.pimu import Pimu
from stretch_body.head import Head
from stretch_body.wacc import Wacc


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
####timeout option
# # --robotraconteur-disable-timeouts=true
class arm_RR(Arm):
	def __init__(self):
		Arm.__init__(self)
		self.status_rr= {'pos': 0.0, 'vel': 0.0, 'force':0.0,'timestamp_pc':0}
	#status overwrite, no motor
	def pull_status(self):
		self.motor.pull_status()
		self.status['timestamp_pc']=time.time()
		self.status['pos']= self.motor_rad_to_translate(self.status['motor']['pos'])
		self.status['vel'] = self.motor_rad_to_translate(self.status['motor']['vel'])
		self.status['force'] = self.motor_current_to_translate_force(self.status['motor']['current'])
		self.status_rr=copy.deepcopy(self.status)
		self.status_rr.pop('motor', None)

class base_RR(Base):
	def __init__(self):
		Base.__init__(self)
		self.status_rr= {'timestamp_pc':0,'x':0,'y':0,'theta':0,'x_vel':0,'y_vel':0,'theta_vel':0, 'pose_time_s':0,'effort': [0, 0]}
	#status overwrite, no wheel
	def pull_status(self):
		"""
		Computes base odometery based on stepper positions / velocities
		"""
		self.left_wheel.pull_status()
		self.right_wheel.pull_status()
		self.status['timestamp_pc'] = time.time()

		p0 = self.status['left_wheel']['pos']
		p1 = self.status['right_wheel']['pos']
		v0 = self.status['left_wheel']['vel']
		v1 = self.status['right_wheel']['vel']
		e0 = self.status['left_wheel']['effort']
		e1 = self.status['right_wheel']['effort']
		t0 = self.status['left_wheel']['timestamp']
		t1 = self.status['right_wheel']['timestamp']
		self.status['translation_force'] = self.motor_current_to_translation_force(self.left_wheel.status['current'],self.right_wheel.status['current'])
		self.status['rotation_torque'] = self.motor_current_to_rotation_torque(self.left_wheel.status['current'],self.right_wheel.status['current'])

		if self.first_step:
			# Upon the first step, simply set the initial pose, since
			# no movement has yet been recorded.
			self.first_step = False

			self.p0 = p0
			self.p1 = p1

			self.t0_s = t0
			self.t1_s = t1

			self.status['x'] = 0.0
			self.status['y'] = 0.0
			self.status['theta'] = 0.0

			self.status['x_vel'] = 0.0
			self.status['y_vel'] = 0.0
			self.status['theta_vel'] = 0.0

		else:
			######################################################
			# The odometry related was wrtten starting on January 14,
			# 2019 whle looking at the following BSD-3-Clause licensed
			# code for reference.

			# https://github.com/merose/diff_drive/blob/master/src/diff_drive/odometry.py

			# The code uses standard calculations. The reference code
			# specifically cites a document with the following link,
			# which appears to be broken.

			# https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf

			# There are many sources on the internet for these
			# equations. For example, the following document is
			# helpful:

			# http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf

			prev_t0_s = self.t0_s
			prev_t1_s = self.t1_s
			t0_s = t0
			t1_s = t1

			delta_t0_s = t0_s - prev_t0_s
			delta_t1_s = t1_s - prev_t1_s

			if (delta_t0_s > 0.0) and (delta_t1_s > 0.0):
				# update if time has passed for both motor readings, otherwise do nothing

				average_delta_t_s = (delta_t0_s + delta_t1_s) / 2.0

				# update the times, since both motors have new readings
				self.prev_t0_s = self.t0_s
				self.prev_t1_s = self.t1_s
				self.t0_s = t0_s
				self.t1_s = t1_s

				# need to check on wrap around / rollover for wheel positions
				self.prev_p0 = self.p0
				self.prev_p1 = self.p1
				self.p0 = p0
				self.p1 = p1

				# Transform the wheel rotations so that left and right
				# wheel distances in meters have the convention that
				# positive values for each wheel corresponds with forward
				# motion of the mobile base.
				prev_left_m = self.prev_p0 * self.meters_per_motor_rad
				left_m = self.p0 * self.meters_per_motor_rad

				prev_right_m = self.prev_p1 * self.meters_per_motor_rad
				right_m = self.p1 * self.meters_per_motor_rad

				delta_left_m = left_m - prev_left_m
				delta_right_m = right_m - prev_right_m

				delta_travel = (delta_right_m + delta_left_m) / 2.0
				delta_theta = (delta_right_m - delta_left_m) / self.wheel_separation_m

				prev_x = self.status['x']
				prev_y = self.status['y']
				prev_theta = self.status['theta']

				if delta_left_m == delta_right_m:
					# delta_theta is 0.0, which would result in a divide
					# by zero error and corresponds with an infinite
					# radius of curvature (0 curvature).
					delta_x = delta_travel * cos(prev_theta)
					delta_y = delta_travel * sin(prev_theta)
				else:
					# calculate the instantaneous center of curvature (ICC)
					icc_radius = delta_travel / delta_theta
					icc_x = prev_x - (icc_radius * sin(prev_theta))
					icc_y = prev_y + (icc_radius * cos(prev_theta))

					# calculate the change in position based on the ICC
					delta_x = ((cos(delta_theta) * (prev_x - icc_x))
							   - (sin(delta_theta) * (prev_y - icc_y))
							   + icc_x - prev_x)

					delta_y = ((sin(delta_theta) * (prev_x - icc_x))
							   + (cos(delta_theta) * (prev_y - icc_y))
							   + icc_y - prev_y)

				# update the estimated total time passed since odometry started
				self.status['pose_time_s'] = self.status['pose_time_s'] + average_delta_t_s

				# update the robot's velocity estimates
				self.status['x_vel'] = delta_travel / average_delta_t_s
				self.status['y_vel'] = 0.0
				self.status['theta_vel'] = delta_theta / average_delta_t_s

				# update the robot's pose estimates
				self.status['x'] = prev_x + delta_x
				self.status['y'] = prev_y + delta_y
				self.status['theta'] = (prev_theta + delta_theta) % (2.0 * pi)

		self.status_rr=copy.deepcopy(self.status)
		self.status_rr.pop('left_wheel', None)
		self.status_rr.pop('right_wheel', None)

	def rr_set_translate_velocity(self, v_m):
		self.set_translate_velocity(v_m)
	def rr_set_rotational_velocity(self, v_r):
		self.rr_set_rotational_velocity(v_r)
	def rr_set_velocity(self, v_m, w_r, a=None):
		self.rr_set_velocity(v_m, w_r)

class wacc_RR(Wacc):
	def __init__(self):
		Wacc.__init__(self)
		self.status_rr= { 'ax':0,'ay':0,'az':0,'a0':0,'d0':0,'d1':0, 'd2':0,'d3':0,'single_tap_count': 0, 'state':0, 'debug':0,
					   'timestamp': 0}
	#status overwrite, no wheel
	def unpack_status(self,s):
		with self.lock:
			sidx=0
			if self.ext_status_cb is not None:
				sidx+=self.ext_status_cb(s[sidx:])
			self.status['ax'] = unpack_float_t(s[sidx:]);sidx+=4
			self.status['ay'] = unpack_float_t(s[sidx:]);sidx+=4
			self.status['az'] = unpack_float_t(s[sidx:]);sidx+=4
			self.status['a0'] = unpack_int16_t(s[sidx:]);sidx+=2
			self.status['d0'] = unpack_uint8_t(s[sidx:]); sidx += 1
			self.status['d1'] = unpack_uint8_t(s[sidx:]); sidx += 1
			self.status['d2'] = unpack_uint8_t(s[sidx:]); sidx += 1
			self.status['d3'] = unpack_uint8_t(s[sidx:]); sidx += 1
			self.status['single_tap_count'] = unpack_uint32_t(s[sidx:]);sidx += 4
			self.status['state'] = unpack_uint32_t(s[sidx:]); sidx += 4
			self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
			self.status['debug'] = unpack_uint32_t(s[sidx:]);sidx += 4
			
			self.status_rr=copy.deepcopy(self.status)
			self.status_rr.pop('transport', None)
			return sidx
	def rr_set_D2(self,on):#0 or 1
		self.set_D2(on)

	def rr_set_D3(self,on): #0 or 1
		self.set_D3(on)

class pimu_RR(Wacc):
	def __init__(self):
		Pimu.__init__(self)
		self.status_rr= {'voltage': 0, 'current': 0, 'temp': 0,'cpu_temp': 0, 'frame_id': 0,
                       'timestamp': 0, 'runstop_event': False, 'bump_event_cnt': 0,
                       'cliff_event': False, 'fan_on': False, 'buzzer_on': False, 'low_voltage_alert':False,'high_current_alert':False,'over_tilt_alert':False,
                       'debug':0}
    def rr_get_voltage(self,raw):
    	return self.get_voltage(raw)
	def rr_get_temp(self,raw):
        
        return self.get_temp(raw)

    def rr_get_current(self,raw):
    	return self.get_current(raw)

    def rr_set_fan_on(self):
        return self.set_fan_on()

    def rr_set_fan_off(self):
        return self.set_fan_off()

    def rr_set_buzzer_on(self):
        return self.set_buzzer_on()

    def rr_set_buzzer_off(self):
    	return self.set_buzzer_off()

    def pull_status(self,exiting=False):
        if not self.hw_valid:
            return
        with self.lock:
            if self._dirty_board_info:
                self.transport.payload_out[0] = RPC_GET_PIMU_BOARD_INFO
                self.transport.queue_rpc(1, self.rpc_board_info_reply)
                self._dirty_board_info=False

            # Queue Body Status RPC
            self.transport.payload_out[0] = RPC_GET_PIMU_STATUS
            self.transport.queue_rpc(1, self.rpc_status_reply)
            self.transport.step(exiting=exiting)
        self.status_rr=copy.deepcopy(self.status)
		self.status_rr.pop('transport', None)
		self.status_rr.pop('imu', None)
		self.status_rr.pop('at_cliff', None)
		self.status_rr.pop('cliff_range', None)


class stretch_RR(Robot):
	def __init__(self):
		Robot.__init__(self)
		self.base=Base_RR()

		self.lift=Lift_RR()

		self.arm=Arm_RR()

		self.wacc=Wacc_RR()

		self.pimu=pimu_RR()

	def get_arm(self):
		return self.arm, "edu.rpi.robotics.stretch.stretch_Arm"
	def get_base(self):
		return self.base, "edu.rpi.robotics.stretch.stretch_Base"
	def get_head(self):
		return self.head, "edu.rpi.robotics.stretch.stretch_Head"
	def get_wacc(self):
		return self.wacc, "edu.rpi.robotics.stretch.stretch_Wacc"
	def get_lift(self):
		return self.lift, "edu.rpi.robotics.stretch.stretch_Lift"
	def get_end_of_arm(self):
		return self.end_of_arm, "edu.rpi.robotics.stretch.stretch_EndOfArm"
	def get_pimu(self):
		return self.pimu, "edu.rpi.robotics.stretch.stretch_Pimu"



def main():
	robot=stretch_RR()
	robot.startup()
	#auto homing
	if not robot.is_calibrated():
		robot.home()
	with RR.ServerNodeSetup("Stretch_Node", 23232) as node_setup:
		#adjust RR timeout
		RRN.RequestTimeout=20
		#Register Service types
		RRN.RegisterServiceTypeFromFile('robdef/edu.rpi.robotics.stretch')
		#create object
		RRN.RegisterService("stretch", "edu.rpi.robotics.stretch.stretch", robot)
		

		input("Press enter to quit")

		robot.stop()




if __name__ == "__main__":
	main()