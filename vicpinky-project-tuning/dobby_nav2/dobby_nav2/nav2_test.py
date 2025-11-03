from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

import math
import tf_transformations


def get_quaternion_from_yaw(yaw_degrees):
    yaw_radians = math.radians(yaw_degrees)
    quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)
    return quaternion


def main(args=None):
    rclpy.init(args=args)
    nav2 = BasicNavigator()
    nav2.waitUntilNav2Active() 
    goal_yaw =  -3.08823
    q = get_quaternion_from_yaw(goal_yaw)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav2.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 0.1
    init_pose = PoseStamped()
    init_pose = goal_pose

    nav2.setInitialPose(init_pose)

    goal_pose.pose.position.x = -0.515067774510408
    goal_pose.pose.position.y = -5.580084493604952
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = -0.6704549427225669
    goal_pose.pose.orientation.w = 0.7419502475091436
    path = nav2.getPath(init_pose, goal_pose)

    if path is not None:
        nav2.followPath(path) 
        nav2.get_logger().info(f'경로 :{path}')
    
    else:
        print("경로를 찾을 수 없습니다.")

    # nav2.goToPose(goal_pose)
    # while not nav2.isTaskComplete():
    #     feedback = nav2.getFeedback()
    #     if feedback.navigation_time.sec > 600:
    #         nav2.cancelTask()

        #nav2.get_logger().info(f'feedback: {feedback}')


    rclpy.shutdown()

if __name__ == '__main__':
    main()