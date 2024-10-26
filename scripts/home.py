import rclpy
import action

def main(args=None):
    rclpy.init(args=args)
    action_client = action.TrajectoryActionClient()
    action_client.send_goal([0.0, 1.57, -1.57, 0.0, -1.57, 0.0], 15)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
