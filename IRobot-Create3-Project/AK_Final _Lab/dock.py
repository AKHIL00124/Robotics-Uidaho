# ROS packages
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Create3 packages
import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, AudioNoteSequence, NavigateToPosition
from irobot_create_msgs.msg import IrOpcode, AudioNote, AudioNoteVector 
from irobot_create_msgs.srv._reset_pose import ResetPose
# Your ROS Node packages
from check_sensor import DockStatusPublisher
# Garrett packages (Easy program start)
from pynput.keyboard import KeyCode
from key_commander import KeyCommander
# Python packages
from std_msgs.msg import String
import random
from builtin_interfaces.msg import Duration
from ir_opcode import OpcodePublisher
from geometry_msgs.msg import PoseStamped

# Globals
rclpy.init()
namespace = 'create3_03EE'
sensor = DockStatusPublisher(namespace)
opcodesensor = OpcodePublisher(namespace)

opc = 1

 

class Roomba(Node):
	def __init__(self, namespace):
		super().__init__("robot")
		
		# Callback Groups
		cb_Subscripions0 = MutuallyExclusiveCallbackGroup()
		cb_Subscripions1 = MutuallyExclusiveCallbackGroup()
		cb_Actions = MutuallyExclusiveCallbackGroup()
		cb_Notes = MutuallyExclusiveCallbackGroup()
		
		# Subscriptions
		self.subscription = self.create_subscription(String,'/check_dock_status',self.listener_callback, 10, callback_group=cb_Subscripions0)
		self.is_docked = False

		# Subscriptions for IrOpcode
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=10)
		self.subscription = self.create_subscription(IrOpcode,f'{namespace}/ir_opcode', self.listener_opcode_callback, qos_profile=qos_policy, callback_group=cb_Subscripions1)
		self.latest_ir_opcode = None
		self.new_ir_opcode = False
	
		# Actions
		self.undock_ac = ActionClient(self, Undock, f'/{namespace}/undock', callback_group=cb_Actions)
		self.drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance', callback_group=cb_Actions)
		self.angle_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle', callback_group=cb_Actions)
		self.audio_ac= ActionClient(self, AudioNoteSequence, f'/{namespace}/audio_note_sequence', callback_group=cb_Notes)
		self.nav_to_pos_ac= ActionClient(self,NavigateToPosition, f'/{namespace}/navigate_to_position', callback_group=cb_Notes)
		self.reset_pose = self.create_client(ResetPose, f'/{namespace}/reset_pose', callback_group=MutuallyExclusiveCallbackGroup())
		self.request = ResetPose.Request()




	def play_audio_seq(self):

		self.audio_ac.wait_for_server()		
		goal_msg = AudioNoteSequence.Goal()
		goal_msg.note_sequence = AudioNoteVector(notes=[AudioNote(frequency=500, max_runtime=Duration(sec=1)),
		   AudioNote(frequency=1000, max_runtime=Duration(sec=1))], append = True)
		goal_msg.iterations = 1
		self.audio_ac.send_goal_async(goal_msg)
		self.get_logger().info("Playing audio sequence")


	def ir_opcode_callback(self,msg):
		self.latest_ir_opcode = msg.opcode
		self.new_ir_opcode = True
		return msg.opcode
            
	def opcode_print(self):
		print('from ir callback print ' + str(opc))

	def process_ir_opcode(self, msg):
		if msg.opcode == IrOpcode.CODE_IR_FORCE_FIELD:
			print("Strong Field stopped the robot")
			self.stop_robot()

		elif msg.opcode in (IrOpcode.CODE_IR_BUOY_GREEN, IrOpcode.CODE_IR_BUOY_RED, IrOpcode.CODE_IR_BUOY_BOTH):
			print("Got red or green or both")
			if msg.sensor == IrOpcode.SENSOR_DIRECTIONAL_FRONT:
				self.align_with_dock(msg)
			elif msg.sensor == IrOpcode.SENSOR_OMNI:
				self.adjust_position()
                        
	def align_with_dock(self, opcode):
		print("Got red or green or both")
		if opcode == IrOpcode.CODE_IR_BUOY_GREEN:
			self.turn_right()
		elif opcode == IrOpcode.CODE_IR_BUOY_RED:
			self.turn_left()
		elif opcode == IrOpcode.CODE_IR_BUOY_BOTH:
			self.drive_forward()


	def turn_left(self):
        # Turn the robot left 
		self.angle_ac.wait_for_server()
		turn_goal = RotateAngle.Goal()
		turn_goal.angle = 0.523599/2# Turning 15 degrees to the left
		self.angle_ac.send_goal_async(turn_goal)
		self.get_logger().info("Turning left.")
 

	def turn_right(self):

		# Turn the robot right
		self.angle_ac.wait_for_server()
		turn_goal = RotateAngle.Goal()
		turn_goal.angle = -0.523599/2# Turning 15 degrees to the right
		self.angle_ac.send_goal_async(turn_goal)
		self.get_logger().info("Turning right.")
 

	def stop_robot(self):
		rclpy.shutdown()
        # Replace this with your robot's stop command
		self.get_logger().info("Stopping robot.")

	def drive_forward(self):
		self.get_logger().info("Moving forward towards the dock.")
		self.drive_ac.wait_for_server()
		drive_goalD = DriveDistance.Goal()
		drive_goalD.max_translation_speed = 0.1
		drive_goalD.distance = 0.13
		print("Sending drive goal")
		self.drive_ac.send_goal(drive_goalD)
		print("moved by 0.3 m")


	def adjust_position(self):
        # Make general position adjustments
		self.get_logger().info("Adjusting position.")
		self.get_logger().info("Moving forward towards the dock.")
		self.drive_ac.wait_for_server()
		drive_goalD = DriveDistance.Goal()
		drive_goalD.distance = 0.05
		print("Sending drive goal")
		self.drive_ac.send_goal(drive_goalD)
		print("moved by 0.05 m")


	
	def listener_callback(self,msg):
		"""
		This function will run when the subscription receives a message from the publisher
		"""
		self.is_docked = msg.data == 'True'
		self.get_logger().info(f"Docking status updated: {self.is_docked}")
		print("I got: ",msg.data)

	def listener_opcode_callback(self,msg):
		"""
		This function will run when the subscription receives a message from the publisher
		"""
		# print("Opcode: ", msg.opcode)

		
	def drive(self):
		# sensor.poll() # Read current dock status

		self.play_audio_seq()

		# Undock
		self.undock_ac.wait_for_server() # Wait till its ready
		undock_goal = Undock.Goal() # Make goal
		print("Sending undock goal")
		self.undock_ac.send_goal(undock_goal) # Send goal blocking
		print("Undocked")
		# sensor.poll()
		self.play_audio_seq()

		# self.reset_pose.call(self.request)
		# self.home = PoseStamped()
		
		# Drive
		self.drive_ac.wait_for_server()
		drive_goal = DriveDistance.Goal()
		drive_goal.distance = 1.0 # 1 meter
		print("Sending drive goal")
		self.drive_ac.send_goal(drive_goal)
		# sensor.poll() # Read current dock status

		self.play_audio_seq()

		# rotate by 1.2 radians
		self.angle_ac.wait_for_server()
		angle = random.uniform(-1.2,1.2)
		turn_goal = RotateAngle.Goal()
		turn_goal.angle = angle
		self.angle_ac.send_goal(turn_goal)

		self.play_audio_seq()

		# drive 0.5m
		self.drive_ac.wait_for_server()
		drive_goal2 = DriveDistance.Goal()
		drive_goal2.distance = 0.5 # 1 meter
		self.drive_ac.send_goal(drive_goal2)
		print("Turned and moved by 0.5m")

		self.play_audio_seq()

		# nav_goal = NavigateToPosition.Goal()
		# nav_goal.goal_pose = self.home
		# self.nav_to_pos_ac.send_goal(nav_goal)

		# Turn back
		self.angle_ac.wait_for_server()
		turnB_goal = RotateAngle.Goal()
		turnB_goal.angle = 3.14159265
		self.angle_ac.send_goal(turnB_goal)

		self.drive_ac.wait_for_server()
		drive_goal3 = DriveDistance.Goal()
		drive_goal3.distance = 0.5 # 1 meter
		self.drive_ac.send_goal(drive_goal3)

		self.play_audio_seq()

		# Rotate
		self.angle_ac.wait_for_server()
		turn_goal = RotateAngle.Goal()
		turn_goal.angle = -angle
		self.angle_ac.send_goal(turn_goal)

		# move back 1m
		self.drive_ac.wait_for_server()
		drive_goal = DriveDistance.Goal()
		drive_goal.distance = 1.0# 1 meter
		print("Sending drive goal")
		self.drive_ac.send_goal(drive_goal)
		print("moved by 1m")

		self.play_audio_seq()

		# print(sensor.poll())
		print(opcodesensor.poll())

		while not opcodesensor.poll() == IrOpcode.CODE_IR_EVAC_BOTH_FIELD:

			if opcodesensor.poll() == IrOpcode.CODE_IR_EVAC_BOTH_FIELD or opcodesensor.poll() == IrOpcode.CODE_IR_FORCE_FIELD or opcodesensor.poll() == IrOpcode.CODE_IR_VIRTUAL_WALL:
				print("Both buoy so go forward")
				self.drive_forward()
				break
			elif opcodesensor.poll() == IrOpcode.CODE_IR_BUOY_GREEN or opcodesensor.poll() ==  IrOpcode.CODE_IR_EVAC_GREEN_FIELD:
				print("Green so go bit right")
				self.turn_right()
			elif opcodesensor.poll() == IrOpcode.CODE_IR_BUOY_RED or opcodesensor.poll() == IrOpcode.CODE_IR_EVAC_RED_FIELD:
				print("Red So go left a bit")
				self.turn_left()
			else:
				print('nothing')

		self.stop_robot()


















	def turn(self):
		# rotate by 1.2 radians
		self.angle_ac.wait_for_server()
		angle = random.uniform(-1.2,1.2)
		turn_goal = RotateAngle.Goal()
		turn_goal.angle = angle
		self.angle_ac.send_goal(turn_goal)
		# drive 0.5m
		self.drive_ac.wait_for_server()
		drive_goal2 = DriveDistance.Goal()
		drive_goal2.distance = 0.5 # 1 meter
		self.drive_ac.send_goal(drive_goal2)
		
		sensor.poll() # Read current dock status
		print("Turned and moved by 0.5m")

	def dockingBack(self):
		sensor.poll() 
		
		
		
		
if __name__ == '__main__':
	roomba = Roomba(namespace)
	exec = MultiThreadedExecutor(5)
	exec.add_node(roomba)
	exec.add_node(sensor)
	exec.add_node(opcodesensor)
    
	keycom = KeyCommander([
		(KeyCode(char='s'), roomba.drive),
		(KeyCode(char='t'), roomba.turn),
		])
	print("S")

	# Try/Except to shutdown "gracefully"
	try:
		exec.spin()
	except KeyboardInterrupt:
		rclpy.shutdown()


def main(args=None):
    rclpy.shutdown()