import rclpy
from rclpy.action import ActionClient
from shr_msgs.action import DockingRequest

def send_cancel_request():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('cancel_docking_action_client')

    # Create an action client for the docking action server
    action_client = ActionClient(node, DockingRequest, 'docking')

    # Wait for the action server to become available
    if not action_client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error('Action server not available')
        return

    # Create a goal handle
    goal_handle = action_client.wait_for_accepted(timeout_sec=20.0)

    if goal_handle:
        node.get_logger().info('Goal accepted. Sending cancel request...')
        # Send a cancel request to the action server
        cancel_future = action_client.async_cancel_goal(goal_handle)

        # Wait for the cancellation response
        if cancel_future:
            node.get_logger().info('Cancel request sent successfully.')
        else:
            node.get_logger().warning('Failed to send cancel request.')

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    send_cancel_request()

if __name__ == '__main__':
    main()