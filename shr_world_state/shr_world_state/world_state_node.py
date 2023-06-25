import os
import sys

sys.path.insert(0, '/home/olagh/Downloads/pybind11_example/build/')

from shr_msgs.action import FindPersonRequest
from rclpy.action import ActionServer, ActionClient
from rclpy.action import CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy

from std_msgs.msg import Bool
from shr_msgs.msg import WorldState, SuccessProtocol

import datetime
import time

from shr_parameters_py.shr_parameters import shr_parameters


class SensorData:
    def __init__(self):
        self.pills_sensor = False
        self.door_motion_sensor = False
        self.door_sensor = False
        self.food_sensor = False
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
        self.world_state.door_open = 0

        self.param_listener = shr_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        ## Subscribe to sensors
        self.subscriber_motion_door = self.create_subscription(Bool, '/smartthings_sensors_motion_door',
                                                               self.door_motion_callback, 10)
        # self.subscriber_motion_pills = self.create_subscription(Bool, '/smartthings_sensors_motion_pills',
        #                                                         self.pill_motion_callback, 10)
        self.subscriber_sensor_door = self.create_subscription(Bool, '/smartthings_sensors_door',
                                                               self.door_open_callback, 10)
        # self.subscriber_eat = self.create_subscription(Bool, '/smartthings_sensors_motion_eat',
        #                                                self.food_callback, 10)
        self.subscriber_motion_bedside = self.create_subscription(Bool, '/smartthings_sensors_motion_bed_side',
                                                                  self.bedside_motion_callback, 10)

        # observe actions from camera
        self.subscriber_observe_pill = self.create_subscription(Bool, '/observe/pill_detection',
                                                                self.observe_pill_callback, 10)
        self.subscriber_observe_eating = self.create_subscription(Bool, '/observe/eat_detection',
                                                                  self.observe_eating_callback, 10)

        self.world_state_pub = self.create_publisher(WorldState, '/world_state', 10)

        # to publish world state
        self.timer = self.create_timer(1, self.timer_callback)
        self.subscriber_successful_protocol = self.create_subscription(SuccessProtocol, '/indicates_success_protocol',
                                                                       self.indicate_success_protocol, 10)

        self.start_stopped_Wandering = None

        # Debugging
        self.subscriber_debug_too_late_to_leave = self.create_subscription(Bool, '/too_late_to_leave',
                                                                           self.debug_too_late_to_leave_callback, 10)
        self.subscriber_debug_time_to_take_medicine = self.create_subscription(Bool, '/time_to_take_medicine',
                                                                               self.debug_time_to_take_medicine_callback,
                                                                               10)
        self.subscriber_debug_time_to_eat_breakfast = self.create_subscription(Bool, '/time_to_eat_breakfast',
                                                                               self.debug_time_to_eat_breakfast_callback,
                                                                               10)
        self.subscriber_debug_time_to_eat_lunch = self.create_subscription(Bool, '/time_to_eat_lunch',
                                                                           self.debug_time_to_eat_lunch_callback, 10)
        self.subscriber_debug_time_to_eat_dinner = self.create_subscription(Bool, '/time_to_eat_dinner',
                                                                            self.debug_time_to_eat_dinner_callback, 10)

        self.too_late_to_leave_debug = 0
        self.time_to_take_medicine_debug = 0
        self.time_to_eat_breakfast_debug = 0
        self.time_to_eat_lunch_debug = 0
        self.time_to_eat_dinner_debug = 0

    def indicate_success_protocol(self, msg):
        pass
        # if msg.success:
        #     if msg.protocol == "midnight_reminder":
        #         self.world_state.took_medicine = 1
        #
        #     elif msg.protocol == "medicine_reminder":
        #         self.world_state.stopped_wandering = 1
        #         self.start_stopped_Wandering = time.time()
        #
        #     elif msg.protocol == "food_reminder":
        #         time_per_ate = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute
        #         if time_per_ate < self.eat_time_breakfast and time_per_ate < self.eat_time_lunch:
        #             self.world_state.ate_breakfast = True
        #         elif time_per_ate < self.eat_time_dinner and time_per_ate > self.eat_time_lunch:
        #             self.world_state.ate_lunch = True
        #         else:
        #             self.world_state.ate_lunch = True

    ## check whether it helps to use snesor  + camera given that the camera is only turned on when motion is detected so redundancy.
    def observe_pill_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.medicine_location
            self.patient_located_time = time.time()

        if self.sensor_data.pills_sensor != msg.data:
            self.sensor_data.pills_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def observe_eating_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.eat_location
            self.patient_located_time = time.time()

        if self.sensor_data.food_sensor != msg.data:
            self.sensor_data.food_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    def timer_callback(self):
        self.publish_world_state()

    # def pill_motion_callback(self, msg):
    #     if msg.data:
    #         self.world_state.patient_location = self.world_state.medicine_location
    #         self.patient_located_time = time.time()
    #
    #     if self.sensor_data.pills_sensor != msg.data:
    #         self.sensor_data.pills_sensor = msg.data
    #         self.publish_world_state(sensor_data=self.sensor_data)

    def bedside_motion_callback(self, msg):
        if msg.data:
            self.world_state.patient_location = self.world_state.bedroom_location
            self.patient_located_time = time.time()

        if self.sensor_data.bedside_motion_sensor != msg.data:
            self.sensor_data.bedside_motion_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    # def food_callback(self, msg):
    #     if msg.data:
    #         self.world_state.patient_location = self.world_state.eat_location
    #         self.patient_located_time = time.time()
    #
    #     if self.sensor_data.food_sensor != msg.data:
    #         self.sensor_data.food_sensor = msg.data
    #         self.publish_world_state(sensor_data=self.sensor_data)

    def door_motion_callback(self, msg):
        print('door_motion_callback')
        if msg.data:
            print('door_motion_callback')
            print('door', msg.data)
            self.world_state.patient_location = self.world_state.door_location
            self.patient_located_time = time.time()
            self.sensor_data.door_motion_sensor = msg.data
            self.world_state.door_motion_sensor = True

            self.publish_world_state(sensor_data=self.sensor_data)

    def door_open_callback(self, msg):

        if self.sensor_data.door_sensor != msg.data:
            self.sensor_data.door_sensor = msg.data
            self.publish_world_state(sensor_data=self.sensor_data)

    ## for debugging
    def debug_too_late_to_leave_callback(self, msg):
        print('here')
        self.too_late_to_leave_debug = msg.data

    def debug_time_to_take_medicine_callback(self, msg):
        print('_to_take_medicine', msg.data)
        if msg:
            self.time_to_take_medicine_debug = msg.data

    def debug_time_to_eat_breakfast_callback(self, msg):
        print('time_to_eat_breakfast', msg.data)
        if msg:
            self.time_to_eat_breakfast_debug = msg.data

    def debug_time_to_eat_lunch_callback(self, msg):
        print('time_to_eat_lunch', msg.data)
        if msg:
            self.time_to_eat_lunch_debug = msg.data

    def debug_time_to_eat_dinner_callback(self, msg):
        print('time_to_eat_dinner', msg.data)
        if msg:
            self.time_to_eat_dinner_debug = msg.data

    def publish_world_state(self, sensor_data=None, patient_location=None):
        current_time = datetime.datetime.now()
        hours = current_time.hour
        minutes = current_time.minute
        seconds = current_time.second
        self.world_state.time.sec = hours * 60 * 60 + minutes * 60 + seconds

        if patient_location:
            self.world_state.patient_location = patient_location

        if sensor_data:
            self.world_state.door_open = sensor_data.door_sensor
            # captures eating food even if the protocol isn't triggered
            if self.sensor_data.food_sensor:
                time_per_ate = datetime.datetime.now().hour * 60 + datetime.datetime.now().minute
                if time_per_ate < self.eat_time_breakfast and time_per_ate < self.eat_time_lunch:
                    self.world_state.ate_breakfast = True
                elif time_per_ate < self.eat_time_dinner and time_per_ate > self.eat_time_lunch:
                    self.world_state.ate_lunch = True
                else:
                    self.world_state.ate_lunch = True

        if self.patient_located_time is None or time.time() - self.patient_located_time > 15:
            self.world_state.patient_location = ""

        ## check time
        print("checking time")
        # if self.too_late_to_leave_debug:
        #     self.world_state.too_late_to_leave = self.too_late_to_leave_debug
        # else:
        #     # before midnight
        #     if datetime.datetime.now().hour * 60 + datetime.datetime.now().minute >= self.too_late_to_leave or datetime.datetime.now().hour * 60 + datetime.datetime.now().minute <= 5*60:
        #         self.world_state.too_late_to_leave = True
        #     # after midnight till 5am
        #     else:
        #         self.world_state.too_late_to_leave = False

        ## this is to stop the wandering protocol from getting triggered again when person is at the door and the robot already called the caregiver. It will be stopped for 5 mins
        ## wandering protocol eventually stops when the person is no longer cdetected by the door sensor so no other stopping condition needs to be added
        if self.start_stopped_Wandering is not None:
            if time.time() - self.start_stopped_Wandering < 5 * 60:  # 5 minutes
                self.world_state.door_motion_sensor = 0
            else:
                self.start_stopped_Wandering = None

        # DEBUG!!!
        self.world_state.patient_location = 'couch'
        self.world_state.robot_location = 'home'
        self.world_state_pub.publish(self.world_state)


def main(args=None):
    rclpy.init(args=args)

    world_state_node = WorldStateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(world_state_node)

    while True:
        rclpy.spin_once(world_state_node)


if __name__ == '__main__':
    main()
