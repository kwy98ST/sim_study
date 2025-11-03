import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
from geometry_msgs.msg import PoseStamped
import tf_transformations
from rclpy.task import Future
from dobby_test_interfaces.action import DobbyNav2

class DobbyNav2Client(Node):
    def __init__(self):
        super().__init__(self, 'dobby_nav2_client')
        self.dobby_nav2_client = ActionClient(self, PoseStamped, 'dobby_nav2')

        self.task_done_future: Future | None = None

    def send_goal(self, goal_pose: PoseStamped) -> Future:

        goal_msg = DobbyNav2.Goal()
        goal_msg.goal_pose = goal_pose

        self.get_logger().info('dobby nav2 test 서버 기다리는 중')
        if not self.dobby_nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('dobby nav2 test 서버가 응답하지 않습니다.')
            raise RuntimeError('dobby nav2 test 서버가 응답하지 않습니다.')
        return self.dobby_nav2_client.send_goal_async(goal_msg, feedback_callback=self.dobby_nav2_callback)
    
    def dobby_nav2_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'feedback: {feedback}')
    
    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        self.get_logger().info('goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'result: {result}')
        if self.task_done_future is not None and self.task_done_future.done():
            self.task_done_future.set_result({'success': result.success, 'message': result.message})


def main(args=None):
    rclpy.init(args=args)
    client = DobbyNav2Client()

