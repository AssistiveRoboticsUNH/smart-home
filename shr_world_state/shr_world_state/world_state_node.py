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

from shr_world_state.shr_world_state_parameters import parameters


class SensorData:
    def __init__(self):
        self.pills_motion_sensor = False
        self.door_motion_sensor = False
        self.door_sensor = False
        self.eat_sensor = False
        self.bedside_motion_sensor = False


class WorldStateNode(Node):
    def __init__(self):
        super().__init__('world_state_node')
        self.took_medicine_time = None
        self.patient_located_time = None
        self._send_goal_future = None
        self._get_result_future = None
        self.goal_handle = None
        self.running_action = False

        self.sensor_data = SensorData()

        self.world_state = WorldState()
        self.world_state.ate_breakfast = 0
        self.world_state.ate_lunch = 0
        self.world_state.ate_dinner = 0

        self.world_state.took_medicine = -1
        self.world_state.too_late_to_leave = -1
        self.world_state.door_open = -1

        self.param_listener = parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.world_state.patient_name = self.params.patient_name
        self.world_state.locations = self.params.locations
        self.world_state.door_location = self.params.door_location
        self.world_state.outside_location = self.params.outside_location
        self.world_state.medicine_location = self.params.medicine_location
        self.world_state.eat_location = self.params.eat_location
        self.world_state.bedroom_location = self.params.bedroom_location

        tmp = self.params.take_medication_time
        self.take_medication_time = 60 * int(tmp.split('h')[0]) + int(tmp.split('h')[1][:-1])

        tmp_eat_breakfast = self.params.eat_time[0]
        self.eat_time_breakfast = 60 * int(tmp_eat_breakfast.split('h')[0]) + int(tmp_eat_breakfast.split('h')[1][:-1])

        tmp_eat_lunch = self.params.eat_time[1]
        self.eat_time_lunch = 60 * int(tmp_eat_lunch.split('h')[0]) + int(tmp_eat_lunch.split('h')[1][:-1])

        tmp_eat_dinner = self.params.eat_time[2]
        self.eat_time_dinner = 60 * int(tmp_eat_dinner.split('h')[0]) + int(tmp_eat_dinner.split('h')[1][:-1])

        self.subscriber_motion_door = self.create_subscription(Bool, '/smartthings_sensors_motion_door',
                                                               self.door_motion_callback, 10)
        self.subscriber_motion_pills = self.create_subscription(Bool, '/smartthings_sensors_motion_pills',
                                                                self.pill_motion_callback, 10)
        self.subscriber_sensor_door = self.create_subscription(Bool, '/smartthings_sensors_door',
                                                               self.door_open_callback, 10)
        self.subscriber_eat = self.create_subscription(Bool, '/eating_sensor',
                                                       self.eat_callback, 10)
        self.subscriber_motion_bedside = self.create_subscription(Bool, '/smartthings_sensors_motion_bed_side',
                                                                  self.bedside_motion_callback, 10)

        self.world_state_pub = self.create_publisher(WorldState, '/world_state', 10)

        self.gather_information_action_server = ActionServer(self, GatherInformationRequest, '/gather_information',
                                                             self.gather_information_callback,
                                                             cancel_callback=self.cancel_callback)
        self.find_person_action_client = ActionClient(self, FindPersonRequest, 'find_person')

        self.timer = self.create_timer(1, self.timer_callback)

        # Debugging
        self.subscriber_debug_too_late_to_leave = self.create_subscription(Bool, '/too_late_to_leave',
                                                                           self.debug_too_late_to_leave_callback, 10)

        self.subscriber_debug_time_to_take_medicine = self.create_subscription(Bool, '/time_to_take_medicine',
                                                                               self.debug_time_to_take_medicine_callback,
                                                                               10)

        self.subscriber_debug_time_to_eat_breakfast = self.create_subscription(Bool, '/time_to_eat_breakfast',
                                                                     self.debug_time_to_eat_breakfast_callback, 10)
        self.subscriber_debug_time_to_eat_lunch = self.create_subscription(Bool, '/time_to_eat_lunch',
                                                                     self.debug_time_to_eat_lunch_callback, 10)
        self.subscriber_debug_time_to_eat_dinner = self.create_subscription(Bool, '/time_to_eat_dinner',
                                                                     self.debug_time_to_eat_dinner_callback, 10)

        self.subscriber_debug_too_late_to_leave = 0

    def timer_callback(self):
        self.publish_world_state()

    def pill_motion_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.medicine_location
            self.patient_located_time = time.time()

        if self.sensor_data.pills_motion_sensor != msg.data:
            self.sensor_data.pills_motion_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def bedside_motion_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.bedroom_location
            self.patient_located_time = time.time()

        if self.sensor_data.bedside_motion_sensor != msg.data:
            self.sensor_data.bedside_motion_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def eat_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.eat_location
            self.patient_located_time = time.time()

        if self.sensor_data.eat_sensor != msg.data:
            self.sensor_data.eat_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def door_motion_callback(self, msg):
        print('6666666666666666door_motion_callback')
        if msg.data:
            print('door_motion_callback')
            print('door', msg.data)
            self.world_state.patient_location = self.world_state.door_location
            self.patient_located_time = time.time()
            self.sensor_data.door_motion_sensor = msg.data
            self.world_state.door_motion_sensor = True

            self.publish_world_state(sensor_data=self.sensor_data)

        # if self.sensor_data.door_motion_sensor != msg.data:
        #     self.sensor_data.door_motion_sensor = msg.data
        #     self.publish_world_state(sensor_data=self.sensor_data)

    def door_open_callback(self, msg):

        if self.sensor_data.door_sensor != msg.data:
            self.sensor_data.door_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def debug_too_late_to_leave_callback(self, msg):
        print('here')

        self.world_state.too_late_to_leave = msg.data
        print('lllllllll', self.world_state.too_late_to_leave)

    def debug_time_to_take_medicine_callback(self, msg):
        print('_to_take_medicine', msg.data)
        if msg:
            self.world_state.time_to_take_medicine = msg.data
            print(self.world_state.time_to_take_medicine)

    def debug_time_to_eat_breakfast_callback(self, msg):
        print('time_to_eat_breakfast', msg.data)
        if msg:
            self.world_state.time_to_eat_breakfast = msg.data

    def debug_time_to_eat_lunch_callback(self, msg):
        print('time_to_eat_lunch', msg.data)
        if msg:
            self.world_state.time_to_eat_lunch = msg.data

    def debug_time_to_eat_dinner_callback(self, msg):
        print('time_to_eat_dinner', msg.data)
        if msg:
            self.world_state.time_to_eat_dinner = msg.data

    def publish_world_state(self, sensor_data=None, patient_location=None):
        if patient_location:
            self.world_state.patient_location = patient_location
        if sensor_data:
            self.world_state.door_open = sensor_data.door_sensor

            if self.sensor_data.pills_motion_sensor:
                self.took_medicine_time = datetime.datetime.now().day
            self.world_state.took_medicine = self.world_state.took_medicine or sensor_data.pills_motion_sensor

            if self.sensor_data.eat_sensor:
                time_per_ate = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute
                if time_per_ate < self.eat_time_breakfast and time_per_ate < self.eat_time_lunch:
                    self.world_state.ate_breakfast = True
                elif time_per_ate < self.eat_time_dinner and time_per_ate > self.eat_time_lunch:
                    self.world_state.ate_lunch = True
                else:
                    self.world_state.ate_lunch = True

        # self.world_state.too_late_to_leave = datetime.datetime.now().hour <= 6 # uncomment after debugging

        if self.took_medicine_time is None or self.took_medicine_time != datetime.datetime.now().day:
            self.world_state.took_medicine = False

        ### add a stopper (condifiton) for wandering problem

        if self.patient_located_time is None or time.time() - self.patient_located_time > 15:
            self.world_state.patient_location = ""

        # self.world_state.time_to_take_medicine = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute > self.take_medication_time
        # self.world_state.time_to_eat_breakfast = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute > self.eat_time_breakfast
        # self.world_state.time_to_eat_lunch = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute > self.eat_time_lunch
        # self.world_state.time_to_eat_dinner = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute > self.eat_time_dinner

        # self.world_state.too_late_to_leave = True  # DEBUG
        # self.world_state.time_to_take_medicine = False  # True  # DEBUG
        # self.world_state.time_to_eat = True  # DEBUG
        # print('before_pub leave,med,eat', self.world_state.too_late_to_leave, self.world_state.time_to_take_medicine, self.world_state.time_to_eat)
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
