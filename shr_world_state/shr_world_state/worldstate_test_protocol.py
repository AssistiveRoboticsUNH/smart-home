import os

from shr_msgs.action import FindPersonRequest, GatherInformationRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.action import CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy

from std_msgs.msg import Bool
from shr_msgs.msg import WorldState

import datetime
import time


class SensorData:
    def __init__(self):
        self.pills_motion_sensor = False
        self.door_motion_sensor = False
        self.door_sensor = False
        self.eat_motion_sensor = False


class WorldStateNode(Node):
    def __init__(self):
        super().__init__('world_state_node')
        self.took_medicine_time = None
        self.ate_time = None
        self.patient_located_time = None
        self._send_goal_future = None
        self._get_result_future = None
        self.goal_handle = None
        self.running_action = False
        self.sensor_data = SensorData()
        self.world_state = WorldState()
        self.world_state.ate = -1
        self.world_state.took_medicine = -1
        self.world_state.too_late_to_leave = -1
        self.world_state.door_open = -1

        self.declare_parameter('patient_name', "nathan")
        self.declare_parameter('locations',
                               ["bedroom_robot_pos", "door_robot_pos", "kitchen_robot_pos", "couch_robot_pos",
                                "outside"])
        self.declare_parameter('door_location', "door_robot_pos")
        self.declare_parameter('outside_location', "outside")
        self.declare_parameter('medicine_location', "kitchen_robot_pos")
        self.declare_parameter('eat_location', "kitchen_robot_pos")

        self.declare_parameter('take_medication_time', "8h15m")
        self.declare_parameter('eat_time', "4h30m")

        self.world_state.patient_name = self.get_parameter('patient_name').value
        self.world_state.locations = self.get_parameter('locations').value
        self.world_state.door_location = self.get_parameter('door_location').value
        self.world_state.outside_location = self.get_parameter('outside_location').value
        self.world_state.medicine_location = self.get_parameter('medicine_location').value
        self.world_state.eat_location = self.get_parameter('eat_location').value

        tmp = self.get_parameter('take_medication_time').value
        self.take_medication_time = 60 * int(tmp.split('h')[0]) + int(tmp.split('h')[1][:-1])

        tmp_eat = self.get_parameter('eat_time').value
        self.eat_time = 60 * int(tmp_eat.split('h')[0]) + int(tmp_eat.split('h')[1][:-1])

        self.subscriber_motion_door = self.create_subscription(Bool, 'smartthings_sensors_motion_door',
                                                               self.door_motion_callback, 10)
        self.subscriber_motion_pills = self.create_subscription(Bool, 'smartthings_sensors_motion_pills',
                                                                self.pill_motion_callback, 10)
        self.subscriber_sensor_door = self.create_subscription(Bool, 'smartthings_sensors_door',
                                                               self.door_open_callback, 10)
        self.subscriber_motion_eat = self.create_subscription(Bool, 'smartthings_sensors_motion_eat',
                                                              self.eat_motion_callback, 10)
        self.world_state_pub = self.create_publisher(WorldState, 'world_state', 10)

        self.gather_information_action_server = ActionServer(self, GatherInformationRequest, 'gather_information',
                                                             self.gather_information_callback,
                                                             cancel_callback=self.cancel_callback)
        self.find_person_action_client = ActionClient(self, FindPersonRequest, 'find_person')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.publish_world_state()

    def pill_motion_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.medicine_location
            self.patient_located_time = time.time()

        if self.sensor_data.pills_motion_sensor != msg.data:
            self.sensor_data.pills_motion_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def eat_motion_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.eat_location
            self.patient_located_time = time.time()

        if self.sensor_data.eat_motion_sensor != msg.data:
            self.sensor_data.eat_motion_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def door_motion_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.door_location
            self.patient_located_time = time.time()

        if self.sensor_data.door_motion_sensor != msg.data:
            self.sensor_data.door_motion_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def action_hub_callback(msg):
        if msg.type == plansys2_msgs.msg.ActionExecution.FINISH:
            print(f"Action: {msg.action} has completed!!!!!!!!!!")
            completed_actions_.append(msg.action)
    def publish_world_state(self, sensor_data=None, patient_location=None):
        self.world_state.ate = -1
        self.world_state.took_medicine = -1
        self.world_state.too_late_to_leave = -1
        self.world_state.door_open = -1

        self.world_state.time_to_take_medicine = 0
        self.world_state.time_to_eat = 1
        self.world_state.too_late_to_leave = 0

        self.world_state_pub.publish(self.world_state)

    def gather_information_callback(self, goal_handle):
        result = GatherInformationRequest.Result()

        states = goal_handle.request.states
        for state in states:
            if state == "patient_location":
                goal_msg = FindPersonRequest.Goal()
                tmp = set(self.world_state.locations) - {self.world_state.outside_location}
                goal_msg.locations = list(tmp)
                goal_msg.name = self.world_state.patient_name

                self.running_action = True
                self.find_person_action_client.wait_for_server()
                self._send_goal_future = self.find_person_action_client.send_goal_async(goal_msg)
                self._send_goal_future.add_done_callback(self.goal_response_callback)
                while self.running_action:
                    rclpy.spin_once(self, timeout_sec=0)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info('Goal canceled')
                        return GatherInformationRequest.Result()

        result.world_state = self.world_state
        goal_handle.succeed()

        return result

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.running_action = False
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Location: {0}'.format(result.location))
        self.running_action = False
        self.world_state.patient_location = result.location
        self.patient_located_time = time.time()
        self.publish_world_state()

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request')
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

        self.running_action = False
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    world_state_node = WorldStateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(world_state_node)

    while True:
        rclpy.spin_once(world_state_node)


if __name__ == '__main__':
    main()
