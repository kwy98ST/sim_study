import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from dobby_test_interfaces.action import DobbyNav2
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from nav2_simple_commander.robot_navigator import BasicNavigator
import math
import tf_transformations

class DobbyNav2Server(Node):
    def __init__(self):
        super().__init__('dobby_nav2_server')
        self.goal_pose = PoseStamped()
        self.nav2 = BasicNavigator()
        self.nav2.waitUntilNav2Active()
        initial_pose = self.init_pose()
        self.nav2.setInitialPose(initial_pose)

        self.dobby_nav2_server = ActionServer(self, DobbyNav2, 'dobby_nav2',
                                              execute_callback=self.execute_callback,
                                              goal_callback=self.goal_callback,
                                              cancel_callback=self.cancel_callback)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self._path_pub = self.create_publisher(
            Path,
            '/dobby_path',
            10
        )

    def amcl_callback(self, msg):
        self.amcl_pose = msg
    def goal_callback(self, goal_request: DobbyNav2.Goal):
        self.get_logger().info(f"이동 명령 요청 {goal_request.goal_pose}")

        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info("이동 명령 취소")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info("이동 명령 실행")

        request = goal_handle.request
        feedback_msg = DobbyNav2.Feedback()
        result = DobbyNav2.Result()

        self.get_logger().info("이동 시작")

        goal_pose = self.set_goal_pose(request)
        dobby_path = Path()
        if self.amcl_pose is None:
            self.amcl_pose = self.init_pose()
        
        dobby_path = self.nav2.getPath(self.convert_to_pose_stamped(self.amcl_pose), goal_pose)
        self._path_pub.publish(dobby_path)
        
        # goToPose를 사용하여 목표 지점으로 이동 요청
        self.nav2.goToPose(goal_pose)
        
        #self.get_logger().info(f'경로 :{dobby_path}')


        # 작업이 완료될 때까지 피드백 전송
        while not self.nav2.isTaskComplete():
            feedback = self.nav2.getFeedback()
            if feedback:
                feedback_msg.part_of_path = feedback.current_pose
                goal_handle.publish_feedback(feedback_msg)
                # self.get_logger().info(f'피드백 전송: {feedback.current_pose.header.stamp}')

        # 결과 처리
        nav2_result = self.nav2.getResult()
        
        if nav2_result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('목표 지점 도착 성공!')
            goal_handle.succeed()
            result.success = True
            result.message = 'Navigation succeeded'
            
        else:
            self.get_logger().error(f'목표 지점 도착 실패! 상태: {nav2_result}')
            goal_handle.abort()
            result.success = False
            result.message = f'Navigation failed with status: {nav2_result}'
            

        return result
    
    

    def init_pose(self):
         
        # 초기 포즈의 yaw 값을 명확히 지정합니다.
        initial_yaw_degrees = -3.08823
        q = self.get_quaternion_from_yaw(initial_yaw_degrees)
        
        init_pose_msg = PoseStamped()
        init_pose_msg.header.frame_id = 'map'
        init_pose_msg.header.stamp = self.nav2.get_clock().now().to_msg()
        init_pose_msg.pose.position.x = 0.0
        init_pose_msg.pose.position.y = 0.0
        init_pose_msg.pose.position.z = 0.0
        init_pose_msg.pose.orientation.x = q[0]
        init_pose_msg.pose.orientation.y = q[1]
        init_pose_msg.pose.orientation.z = q[2]
        init_pose_msg.pose.orientation.w = q[3]
        return init_pose_msg
    
    def set_goal_pose(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav2.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal.goal_pose.pose.position.x
        goal_pose.pose.position.y = goal.goal_pose.pose.position.y
        goal_pose.pose.position.z = goal.goal_pose.pose.position.z
        goal_pose.pose.orientation.x = goal.goal_pose.pose.orientation.x
        goal_pose.pose.orientation.y = goal.goal_pose.pose.orientation.y
        goal_pose.pose.orientation.z = goal.goal_pose.pose.orientation.z
        goal_pose.pose.orientation.w = goal.goal_pose.pose.orientation.w
        return goal_pose

    def get_quaternion_from_yaw(self, yaw_degrees):
        yaw_radians = math.radians(yaw_degrees)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)
        return quaternion
    def convert_to_pose_stamped(self, cov_stamped_msg: PoseWithCovarianceStamped) -> PoseStamped:
        new_pose_stamped = PoseStamped()
        new_pose_stamped.header = cov_stamped_msg.header
        new_pose_stamped.pose = cov_stamped_msg.pose.pose
        return new_pose_stamped
def main(args=None):
    rclpy.init(args=args)
    node = DobbyNav2Server()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('dobby_nav2_server 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main