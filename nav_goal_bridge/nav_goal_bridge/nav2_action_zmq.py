import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# from shr_msgs.action import NavToGoal
from nav2_msgs.action import NavigateToPose
import json
import time
import zmq


class Nav2ActionServer(Node):
    def __init__(self):
        super().__init__('nav2_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback)

        print('server initiated')

        self.nav_goal_json = None
        self.msg_in_work = False
        context = zmq.Context()
        print("Connecting to Server on port 5555")
        self.socket = context.socket(zmq.REQ)
        self.socket.connect("tcp://192.168.1.14:5555")

        # to convert msg to json

    def serialize_goal(self, goal):
        return {
            'pose': {
                'header': {
                    'stamp': {
                        'sec': goal.pose.header.stamp.sec,
                        'nanosec': goal.pose.header.stamp.nanosec
                    },
                    'frame_id': goal.pose.header.frame_id
                },
                'position': {
                    'x': goal.pose.pose.position.x,
                    'y': goal.pose.pose.position.y,
                    'z': goal.pose.pose.position.z
                },
                'orientation': {
                    'x': goal.pose.pose.orientation.x,
                    'y': goal.pose.pose.orientation.y,
                    'z': goal.pose.pose.orientation.z,
                    'w': goal.pose.pose.orientation.w
                }
            }
        }

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        print('goal_handle', goal_handle.request)
        if not self.msg_in_work:
            self.msg_in_work = True
            # Convert the goal object to a JSON string
            nav_goal_json = json.dumps(self.serialize_goal(goal_handle.request))
            print(f"Sending message: {nav_goal_json}")
            # #run server
            self.nav_goal_json = nav_goal_json
            if self.nav_goal_json is not None:
                self.socket.send_string(self.nav_goal_json)
                print('Waiting for message')

                message = self.socket.recv()  # block until a message is received
                print('##########message recieved', message)
                if message.decode() == '1':  # True
                    self.nav_goal_json = None
                    print('ffffffffffffffff')
                else:
                    self.nav_goal_json = None
                    print('Goal aborted')
                    print('cccccccccccccccc')

            self.msg_in_work = False

        else:
            goal_handle.abort()

        result = NavigateToPose.Result()
        # result.error_code = result.NONE
        return result


def main(args=None):
    rclpy.init(args=None)
    nav2_action_server = Nav2ActionServer()
    rclpy.spin(nav2_action_server)


if __name__ == '__main__':
    main()
