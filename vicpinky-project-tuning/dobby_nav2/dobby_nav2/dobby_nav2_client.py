import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
from geomety_msgs.msg import PoseStamped
import tf_transformations

class DobbyNav2Client(Node):
    def __init__(self):
        super().__init__(self, 'dobby_nav2_client')

    def dobby_go_to_pose(self, goal_pose: PoseStamped):
        nav = BasicNavigator()
        nav.waitUntilNav2Active()

        nav.goToPose(goal_pose)

    def get_quaternion_from_yaw(yaw_degrees):
        yaw_radians = math.radians(yaw_degrees)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)
        return quaternion
def main(args=None):
    rclpy.init(args=args)
    client = DobbyNav2Client()

