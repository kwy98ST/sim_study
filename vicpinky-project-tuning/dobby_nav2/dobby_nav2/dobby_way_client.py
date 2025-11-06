from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from dobby_test_interfaces.action import DobbyNav2
import rclpy
import sys

class DobbyWayClient(Node):
    def __init__(self):
        super().__init__('dobby_way_client')
        self.dobby_way_client = ActionClient(self, DobbyNav2, 'dobby_way')
        self.waypoints = []
        self._next_waypoint_to_send_index = 0

    def send_waypoints(self, waypoints_list: list[PoseStamped]):
        self.waypoints = waypoints_list
        self._next_waypoint_to_send_index = 0

        if not self.waypoints:
            self.get_logger().warn('전송할 웨이포인트가 없습니다. 노드를 종료합니다.')
            rclpy.shutdown()
            return

        self.get_logger().info('dobby nav2 test 서버 기다리는 중')
        if not self.dobby_way_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('dobby nav2 test 서버가 응답하지 않습니다.')
            rclpy.shutdown()
            return

        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self._next_waypoint_to_send_index < len(self.waypoints):
            current_waypoint_to_send_idx = self._next_waypoint_to_send_index
            goal_pose = self.waypoints[current_waypoint_to_send_idx]
            self.get_logger().info(f'웨이포인트 {current_waypoint_to_send_idx + 1}/{len(self.waypoints)} 전송: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}')
            
            goal_msg = DobbyNav2.Goal()
            goal_msg.goal_pose = goal_pose

            send_goal_future = self.dobby_way_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
            self._next_waypoint_to_send_index += 1
        else:
            self.get_logger().info('모든 웨이포인트에 성공적으로 도달했습니다. 노드를 종료합니다.')
            rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        current_waypoint_idx_for_feedback = self._next_waypoint_to_send_index - 1
        feedback = feedback_msg.feedback
        if feedback.part_of_path.header.stamp.sec > 0:
            self.get_logger().info(f'웨이포인트 {current_waypoint_idx_for_feedback + 1} 진행 중: 현재 위치 ({feedback.part_of_path.pose.position.x:.2f}, {feedback.part_of_path.pose.position.y:.2f})')
    
    def goal_response_callback(self, future):
        current_waypoint_idx_for_response = self._next_waypoint_to_send_index - 1
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'웨이포인트 {current_waypoint_idx_for_response + 1} 목표가 거절되었습니다. 노드를 종료합니다.')
            rclpy.shutdown()
            return
        self.get_logger().info(f'웨이포인트 {current_waypoint_idx_for_response + 1} 목표가 수락되었습니다.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        completed_waypoint_idx = self._next_waypoint_to_send_index - 1
        result = future.result().result
        if result.success:
            self.get_logger().info(f'웨이포인트 {completed_waypoint_idx + 1} 도착 성공: {result.message}')
        else:
            self.get_logger().error(f'웨이포인트 {completed_waypoint_idx + 1} 도착 실패: {result.message}. 노드를 종료합니다.')
            rclpy.shutdown()
            return
        
        self._send_next_waypoint()

def create_pose_stamped(x, y, z, ox, oy, oz, ow):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = ox
    pose.pose.orientation.y = oy
    pose.pose.orientation.z = oz
    pose.pose.orientation.w = ow
    return pose

def main(args=None):
    rclpy.init(args=args)
    
    node = DobbyWayClient()
    
    waypoints_list = []

    # waypoints_list.append(create_pose_stamped(
    #     x=0.002104863736113543,
    #     y=-2.814145041145435,
    #     z=0.0,
    #     ox=0.0,
    #     oy=0.0,
    #     oz=-0.6398474934894186,
    #     ow=0.7685019096107104
    # ))

    waypoints_list.append(create_pose_stamped(
        x=-0.04583737149771297,
        y=-5.652920456214616,
        z=0.0,
        ox=0.0,
        oy=0.0,
        oz=0.9965768993351637,
        ow=0.6795402409084962
    ))

    waypoints_list.append(create_pose_stamped(
        x=-5.105231431386234,
        y=-7.376737876300931,
        z=0.0,
        ox=0.0,
        oy=0.0,
        oz=0.6690682999336836,
        ow=0.08267093631688845
    ))

    waypoints_list.append(create_pose_stamped(
        x=-5.96711894122843,
        y=-5.009338924913205,
        z=0.0,
        ox=0.0,
        oy=0.0,
        oz=0.7440585313263648,
        ow=0.6681144377727913
        
    ))

    waypoints_list.append(create_pose_stamped(
        x=-5.896065233500684,
        y=-2.428836424992029,
        z=0.0,
        ox=0.0,
        oy=0.0,
        oz=0.7094075696289585,
        ow=0.7047984819458214
        
    ))

    try:
        node.get_logger().info('dobby_way_client 노드 실행')
        node.send_waypoints(waypoints_list)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('dobby_way_client 노드 종료')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
