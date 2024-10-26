import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('trajectory_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/trajectory_controller/follow_joint_trajectory')

    def send_goal(self, positions, duration):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['first_joint', 'second_joint', 'third_joint', 'fourth_joint', 'fifth_joint', 'sixth_joint']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration
        goal_msg.trajectory.points.append(point)
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()
    action_client.send_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
