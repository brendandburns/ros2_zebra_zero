import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rclpy.init()
    node = rclpy.create_node('joint_trajectory_publisher')

    # Create a publisher for the joint trajectory
    pub = node.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)

    # Create a JointTrajectory message
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['first_joint', 'second_joint', 'third_joint', 'fourth_joint', 'fifth_joint', 'sixth_joint']

    # Add waypoints (JointTrajectoryPoints) to the trajectory
    #for i in range(0, 15):
    #    point = JointTrajectoryPoint()
    #    point.positions = [0.0, 0.1 * i, -0.1 * i, 0.0, -0.1 * i, 0.0]  # Joint positions at time t=0
    #    # point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #    point.time_from_start.sec = i 
    #    joint_trajectory.points.append(point)
    point = JointTrajectoryPoint()
    point.positions = [0.0, 1.57, -1.57, 0.0, -1.57, 0.0]
    point.time_from_start.sec = 15
    joint_trajectory.points.append(point)

    # Add more waypoints as needed

    # Publish the trajectory
    pub.publish(joint_trajectory)

    # rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
