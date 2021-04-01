import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import stretch_body.robot

def main():
	robot=stretch_body.robot.Robot()
	robot.startup()
	with RR.ServerNodeSetup("Stretch_Node", 23232) as node_setup:

		#Register Service types
		RRC.RegisterStdRobDefServiceTypes(RRN)
		#create object

		

		input("Press enter to quit")


		RS_obj.pipeline.stop()




if __name__ == "__main__":
	main()