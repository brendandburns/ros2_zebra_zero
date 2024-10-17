import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


path = [
    [0, 0, 0, 0, 0, 0],
[-0.03634, -0.0383777, -0.0868619, -5.73745e-06, -0.0484842, 0.0363457],
[-0.0530737, -0.0767788, -0.173819, 3.08304e-06, -0.0970406, 0.0530706],
[-0.0852864, -0.115134, -0.260759, 8.02719e-07, -0.145625, 0.0852856],
[-0.13105, -0.15351, -0.347878, -2.75656e-08, -0.194368, 0.131051],
[-0.189027, -0.191941, -0.435299, -3.80996e-07, -0.243358, 0.189027],
[-0.257949, -0.230282, -0.522748, -7.10037e-07, -0.292466, 0.25795],
[-0.335637, -0.268627, -0.610494, -1.04056e-06, -0.341868, 0.335638],
[-0.420322, -0.30696, -0.698564, -3.38301e-09, -0.391604, 0.420322],
[-0.508985, -0.345269, -0.787001, -5.25069e-07, -0.441732, 0.508985],
[-0.599049, -0.38354, -0.875853, -1.0145e-06, -0.492314, 0.59905],
[-0.688457, -0.421752, -0.965172, 1.3194e-08, -0.54342, 0.688457],
[-0.688458, -0.459883, -1.05501, 1.46866e-06, -0.595132, 0.688457],
[-0.688461, -0.497895, -1.14543, 9.15238e-06, -0.647536, 0.688453],
[-0.700009, -0.535754, -1.23651, -6.33664e-07, -0.700752, 0.7],
[-0.699731, -0.573407, -1.3283, -1.20969e-05, -0.754893, 0.699739],
[-0.699735, -0.610769, -1.42088, -1.14438e-06, -0.810113, 0.699736]
]

def main():
    rclpy.init()
    node = rclpy.create_node('joint_trajectory_publisher')

    # Create a publisher for the joint trajectory
    pub = node.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)

    # Create a JointTrajectory message
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ['first_joint', 'second_joint', 'third_joint', 'fourth_joint', 'fifth_joint', 'sixth_joint']
    delta_time = 0.5

    i = 0
    for pt in path:
        point = JointTrajectoryPoint()
        point.positions = pt
        point.time_from_start.sec = i
        i = i + 1
        joint_trajectory.points.append(point)

    pub.publish(joint_trajectory)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
