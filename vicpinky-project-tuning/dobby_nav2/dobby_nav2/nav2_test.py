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
    goal_pose.pose.position.x = -5.7914299964904785
    goal_pose.pose.position.y = -8.801392555236816
    goal_pose.pose.position.z = -0.001434326171875
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    nav2.setInitialPose(goal_pose)

    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0

    nav2.goToPose(goal_pose)
    while not nav2.isTaskComplete():
        feedback = nav2.getFeedback()
        if feedback.navigation_time > 600:
            nav2.cancelTask()

        nav2.get_logger().info(f'feedback: {feedback}')


    rclpy.shutdown()

if __name__ == '__main__':
    main()