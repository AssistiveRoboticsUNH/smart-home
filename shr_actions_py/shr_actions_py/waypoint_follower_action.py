import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from shr_msgs.action import WaypointRequest
from rclpy.action import ActionServer
from rclpy.node import Node

class WaypointRequestActionServer(Node):
    def __init__(self):
        super().__init__('waypoint_request_action')
        self.waypoint_request_action_server = ActionServer(
            self, WaypointRequest, 'waypoint_request', self.waypoint_request_callback
        )
        self.navigator = BasicNavigator()

        self.waypoint_routes = {
            "home":{
                "kitchen": [
                    [-0.2739, -0.032, -0.722, 0.6915],
                    [-0.0549, 0.00278, -0.1854, 0.982],
                    [0.3919, -0.111, -0.389, 0.921],
                    [0.685, -0.400, -0.403, 0.915],
                    [0.935, -0.737, -0.584, 0.811],
                    [1.07, -0.850, 0.0, 1.0],
                ],
                "dining_room": [
                    [-0.2739, -0.032, -0.722, 0.6915],
                    [-0.042, 0.1713, -0.39, 0.921],
                    [0.129, 0.408, 0.4509, 0.892],
                    [0.245, 0.706, 0.668, 0.74],
                    [0.275, 1.185, 0.6882, 0.725],
                    [0.129, 1.62, 0.0, 1.0],
                ],
                "door": [
                    [-0.228, 0.361, 0.5104,  0.8598],
                    [-0.292, 0.562,  0.788, 0.615],
                    [-0.411, 0.900, 0.881, 0.472],
                    [-0.502, 1.055, 0.896, 0.443],
                    [-0.578, 1.130, 1.000, -0.013],
                ],
            },
            "kitchen":{
                "home":[
                    [1.0209, -0.769, 0.942, 0.335],
                    [0.738, -0.508, 0.932, 0.361],
                    [0.535, -0.186,  0.956, 0.292],
                    [0.299, 0.0237,  0.994, 0.104],
                    [ -0.109, 0.0889, -0.912,  0.408],
                    [-0.2739, -0.032, -0.722,  0.6915],
                ],
                "dining_room": [
                    [1.018, -0.586, 0.858, 0.512],
                    [0.829, -0.159, 0.911, 0.411],
                    [0.532, 0.159, 0.819, 0.572],
                    [0.355, 0.680,  0.784, 0.619],
                    [ 0.225, 1.237, 0.737, 0.675],
                    [0.129, 1.62, 0.0, 1.0],
                ],
                "door": [
                    [1.018, -0.586, 0.858, 0.512],
                    [0.767, -0.249, 0.941,  0.337],
                    [0.486, 0.0029, 0.929,  0.368],
                    [0.222, 0.372, 0.911, 0.411],
                    [0.0410, 0.717, 0.944, 0.329],
                    [-0.257, 1.057, 0.983, 0.181],
                    [-0.578, 1.130, 1.000, -0.013],
                ],
            },

            "dining_room":{
                "home":[
                    [0.046, 1.526, -0.757, 0.652],
                    [0.0234, 1.18, -0.693, 0.720],
                    [0.0175, 0.846, -0.743, 0.66],
                    [-0.005, 0.465, -0.707, 0.707],
                    [-0.2739, -0.032, -0.722,  0.6915],
                ],
                "kitchen": [
                    [0.149, 1.44, -0.657, 0.753],
                    [0.188, 1.095, -0.536, 0.844],
                    [0.326, 0.772, -0.598, 0.801],
                    [0.455, 0.445, -0.566, 0.824],
                    [0.614, 0.105, -0.513, 0.858],
                    [0.804, -0.213, -0.588, 0.808],
                    [0.920, -0.566, -0.517, 0.855],
                    [1.07, -0.850, 0.0, 1.0],
                ],

                "door": [
                    [5.731, 1.55, -0.928, 0.372],
                    [-0.192, 1.353, -0.975, 0.220],
                    [-0.504, 1.223, -0.974, 0.223],
                    [-0.578, 1.130, 1.000, -0.013],
                ],
            },

            "door":{
                "home":[
                    [-0.436, 0.924, -0.525, 0.850],
                    [-0.2914, 0.586, -0.609, 0.792],
                    [-0.1869, 0.225, -0.507, 0.862],
                    [-0.0582, 0.0522, -0.707, 0.707],
                    #[-0.1576, -0.358, -0.799, 0.600],
                    [-0.2739, -0.032, -0.722,  0.6915],
                ],
                "kitchen": [
                    [-0.462, 1.149, -0.345, 0.9383],
                    [-0.2188, 0.7953, -0.4086, 0.912],
                    [0.0303, 0.468, -0.382, 0.9238],
                    [0.3295, 0.246, -0.391, 0.920],
                    [0.628, -0.018, -0.462, 0.886],
                    [0.855, -0.412, -0.513, 0.857],
                    [1.07, -0.850, 0.0, 1.0],
                ],

                "dining_room": [
                    [-0.401, 1.238, 0.234, 0.972],
                    [-0.141, 1.410, 0.406, 0.913],
                    [-0.019, 1.559, 0.371, 0.928],
                    [0.129, 1.620, -1.639, 0.0, 1.0],
                ],
            }

        }


    def waypoint_request_callback(self, goal_handle):
        """
        Handle waypoint requests.

        Args:
            goal_handle: The goal handle containing the waypoint request.

        Returns:
            WaypointRequest.Result: The result of the waypoint execution.
        """
        from_location = goal_handle.request.from_location
        to_location = goal_handle.request.to_location

        result_custom = WaypointRequest.Result()

        waypoints = None
        try:
            if from_location in self.waypoint_routes and to_location in self.waypoint_routes[from_location]:
                waypoints = self.waypoint_routes[from_location][to_location]
            else:
                error_message = f'Route from {from_location} to {to_location} is not defined.'
                print(error_message)
                result_custom.status = error_message
                goal_handle.abort()
                return result_custom

            if waypoints is not None:
                inspection_points = self.create_inspection_points(waypoints)
                self.navigator.followWaypoints(inspection_points)

                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        print('Executing current waypoint:', feedback.current_waypoint + 1)
                        result_custom.status = f"Executing waypoint {feedback.current_waypoint + 1}"

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f'Inspection of {to_location} complete!')
                    result_custom.status = f'Inspection of {to_location} complete!'
                    goal_handle.succeed()

                elif result == TaskResult.CANCELED:
                    print(f'Inspection of {to_location} was canceled.')
                    result_custom.status = f'Inspection of {to_location} was canceled.'
                    goal_handle.abort()
                elif result == TaskResult.FAILED:
                    print(f'Inspection of {to_location} failed!')
                    result_custom.status = f'Inspection of {to_location} failed!'
                    goal_handle.abort()

        except Exception as e:
            print("An error occurred:", e)
            result_custom.status = "An error occurred during waypoint execution"
            goal_handle.abort()

        return result_custom



    def create_inspection_points(self, waypoints):
        inspection_points = []
        for pt in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.z = pt[2]
            pose.pose.orientation.w = pt[3]
            inspection_points.append(pose)
        return inspection_points

def main(args=None):
    rclpy.init(args=args)
    waypoint_request_action_server = WaypointRequestActionServer()
    rclpy.spin(waypoint_request_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


