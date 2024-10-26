import rclpy
import action
import argparse

def main():
    # Create the parser
    parser = argparse.ArgumentParser(description="Move to any position")

    # Add the arguments
    parser.add_argument('angles', metavar='N', type=float, nargs='+', help='The joint angles to move to')
    args = parser.parse_args()

    angles = args.angles
    if len(angles) != 6:
        print("Expected 6 angles, exiting.")
        return

    rclpy.init()
    action_client = action.TrajectoryActionClient()
    action_client.send_goal(angles, 15)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
