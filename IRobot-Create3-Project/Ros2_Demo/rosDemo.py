import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock

from pynput.keyboard import KeyCode
from key_commander import KeyCommander

class Roomba(Node):
    def __init__(self, namespace):
        super().__init__('robot') # Init parent and give it a name
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock')
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance')

    def drive_away(self):
        """
        Undocks the robot using the blocking version of send_goal()
        so that the robot doesn't attempt to drive until the
        goal is complete.

        Afterward an asynchronous goal is sent which allows
        the goal to be cancelled before it finishes executing.
        """
        self.get_logger().warning('WAITING FOR SERVER')
        # wait until the robot server is found and
        #   ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

        # create new Undock goal object to send to server
        undock_goal = Undock.Goal()

        # send the goal: send_goal() blocks until action
        #   is complete without returning a UUID for the goal.
        #   Thus it is impossible to cancel the goal from this
        #   script.
        self._undock_ac.send_goal(undock_goal)

        # print statement after goal completes since send_goal() blocks
        self.get_logger().warning('UNDOCKED')

        # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

        # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 1.0

        # send goal asynchronously:
        #   when the goal is sent asynchronously we are able
        #   to attach a callback which will capture the goal
        #   unique identifier(UUID) which can then be used to
        #   cancel the goal's execution prematurely
        goal_response = self._drive_ac.send_goal_async(drive_goal)

        # attach the callback to get goal UUID after it has
        #   been accepted by the server
        #goal_response.add_done_callback()


if __name__ == '__main__':
    rclpy.init()
   
    namespace = 'create3_03B8'
    s = Roomba(namespace)
    keycom = KeyCommander([
        (KeyCode(char='s'), s.drive_away), # attach start action callback
        ])
    print("s: Garretts undock and drive a meter")

    rclpy.spin(s) # execute slash callbacks until shutdown or destroy is called
    rclpy.shutdown()










import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.action import DriveDistance, RotateAngle, PlayAudio
from irobot_create_msgs.msg import DockStatus
from std_msgs.msg import Bool
import random

class Create3Explorer(Node):
    def __init__(self):
        super().__init__('create3_explorer')
        # Action clients
        self.drive_client = ActionClient(self, DriveDistance, '/create3_03B8/drive_distance')
        self.rotate_client = ActionClient(self, RotateAngle, '/create3_03B8/rotate_angle')
        self.audio_client = ActionClient(self, PlayAudio, '/create3_03B8/play_audio')
        # Subscriptions
        self.dock_subscriber = self.create_subscription(DockStatus, '/create3_03B8/dock_status', self.dock_status_callback, qos_profile_sensor_data)
        self.fully_docked = False

    def dock_status_callback(self, msg):
        self.fully_docked = msg.is_docked

    def play_chirp(self, tone):
        # Assuming PlayAudio action has a 'tone' parameter
        goal = PlayAudio.Goal()
        goal.tone = tone
        self.audio_client.send_goal_async(goal)

    def undock(self):
        # Assuming there's a similar action or service for undocking
        # Implement undocking logic here
        pass

    def drive_distance(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = distance
        self.drive_client.send_goal_async(goal)

    def rotate_randomly(self):
        angle = random.uniform(-180, 180)  # Random angle between -180 and 180 degrees
        goal = RotateAngle.Goal()
        goal.angle = angle
        self.rotate_client.send_goal_async(goal)

    def check_docking(self):
        # Implement logic to move towards the dock
        # and check self.fully_docked status
        pass

    def execute_sequence(self):
        self.undock()
        self.play_chirp('chirp1')
        self.drive_distance(1.0)  # Drive 1 meter
        self.play_chirp('chirp2')
        self.rotate_randomly()
        self.play_chirp('chirp3')
        self.drive_distance(0.5)  # Drive 0.5 meter
        while not self.fully_docked:
            self.check_docking()
        self.play_chirp('chirp4')

def main(args=None):
    rclpy.init(args=args)
    explorer = Create3Explorer()
    explorer.execute_sequence()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






from some_robot_msgs.srv import PlayAudio  # Import the correct service type
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

class Roomba(Node):
    def __init__(self, namespace):
        # ... other initializations ...
        self.play_audio_client = self.create_client(PlayAudio, '/play_audio_service')
        # ... other initializations ...

    def play_audio_note(self, frequency, duration_seconds):
        if not self.play_audio_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Audio service not available...')
            return

        request = PlayAudio.Request()
        request.frequency = frequency
        request.max_runtime = Duration(seconds=duration_seconds)

        self.play_audio_client.call_async(request)
        self.get_logger().info(f'Playing audio note at {frequency} Hz for {duration_seconds} seconds')




class Roomba(Node):
    # ... existing methods ...

    def some_transition_function(self):
        # ... some logic ...

        # Play an audio note
        self.play_audio_note(440, 2)  # Play a 440 Hz note for 2 seconds

        # ... rest of the logic ...


