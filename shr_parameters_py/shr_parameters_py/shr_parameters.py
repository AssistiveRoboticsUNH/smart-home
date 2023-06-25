# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators



class shr_parameters:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        sensor_pills_motion_topic = "smartthings_sensors_motion_pills"
        sensor_eat_motion_topic = "smartthings_sensors_motion_eat"
        sensors_door_motion_topic = "smartthings_sensors_motion_door"
        sensors_door_open_topic = "smartthings_sensors_door"
        update_protocol_topic = "update_protocol"
        world_state_topic = "world_state"
        tf_frames = ["bedroom", "kitchen", "couch", "door"]
        class __Pddl:
            class __Instances:
                Landmarks = ["kitchen", "couch", "home", "door", "outside"]
                Robots = ["jackel"]
                Persons = ["nathan"]
            instances = __Instances()
            class __Foodprotocols:
                instances = ["breakfast", "lunch", "dinner"]
                eat_times = ["7h0m0s/8h0m0s", "14h30m0s/15h30m0s", "18h00m0s/19h0m0s"]
                eat_locations = ["kitchen", "kitchen", "kitchen"]
                check_guide_to_succeeded_times = ["0h1m0s", "0h1m0s", "0h1m0s"]
                remind_automated_food_at_times = ["0h10m0s", "0h10m0s", "0h10m0s"]
                remind_automated_food_at_2_times = ["0h10m0s", "0h1m0s", "0h10m0s"]
            FoodProtocols = __Foodprotocols()
            class __Medicineprotocols:
                instances = ["daily"]
                medicine_location = ["kitchen"]
                take_medication_time = ["8h0m0s/9h0m0s"]
                check_guide_to_succeeded_times = ["0h1m0s"]
                notify_automated_medicine_at_times = ["0h1m0s"]
            MedicineProtocols = __Medicineprotocols()
            class __Wonderingprotocols:
                instances = ["daily"]
                outside_location = ["outside"]
                door_location = ["door"]
                bedroom_location = ["bedroom"]
                too_late_to_leave_time = ["17h0m0s/8h0m0s"]
                check_bed_after_return_wait_times = ["0h1m0s"]
                detect_person_left_house_times = ["0h10m0s"]
                wait_for_person_to_return_times = ["0h0m30s"]
                check_if_person_went_to_bed_times = ["0h0m30s"]
            WonderingProtocols = __Wonderingprotocols()
            class __Fallprotocols:
                instances = ["daily"]
                wait_times = ["0h10m0s"]
            FallProtocols = __Fallprotocols()
        pddl = __Pddl()
        class __TfValues:
            bedroom = [-2.0, -3.69, 0.0, 0.0, 0.0, 0.0, 1.0]
            kitchen = [1.9, -0.27, 0.0, 0.0, 0.0, 0.0, 1.0]
            couch = [0.0, 1.6, -0.0, 0.0, 0.0, 0.0, 1.0]
            door = [-2.89, 0.59, 0.0, 0.0, 0.0, 0.0, 1.0]
        tf_values = __TfValues()
        class __Topics:
            medicine_sensor = "/smartthings_sensors_motion_pills"
            food_sensor = "/smartthings_sensors_motion_food"
            bed_side_sensor = "/smartthings_sensors_bed_side"
        topics = __Topics()



    class ParamListener:
        def __init__(self, node, prefix=""):
            node.declare_parameter('my_parameter', 'world')
            self.prefix_ = prefix
            self.params_ = shr_parameters.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("shr_parameters." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "pddl.instances.Landmarks":
                    updated_params.pddl.instances.Landmarks = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.instances.Robots":
                    updated_params.pddl.instances.Robots = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.instances.Persons":
                    updated_params.pddl.instances.Persons = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.instances":
                    updated_params.pddl.FoodProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.eat_times":
                    updated_params.pddl.FoodProtocols.eat_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.eat_locations":
                    updated_params.pddl.FoodProtocols.eat_locations = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.check_guide_to_succeeded_times":
                    updated_params.pddl.FoodProtocols.check_guide_to_succeeded_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_times":
                    updated_params.pddl.FoodProtocols.remind_automated_food_at_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_2_times":
                    updated_params.pddl.FoodProtocols.remind_automated_food_at_2_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.instances":
                    updated_params.pddl.MedicineProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.medicine_location":
                    updated_params.pddl.MedicineProtocols.medicine_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.take_medication_time":
                    updated_params.pddl.MedicineProtocols.take_medication_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.check_guide_to_succeeded_times":
                    updated_params.pddl.MedicineProtocols.check_guide_to_succeeded_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.notify_automated_medicine_at_times":
                    updated_params.pddl.MedicineProtocols.notify_automated_medicine_at_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.instances":
                    updated_params.pddl.WonderingProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.outside_location":
                    updated_params.pddl.WonderingProtocols.outside_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.door_location":
                    updated_params.pddl.WonderingProtocols.door_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.bedroom_location":
                    updated_params.pddl.WonderingProtocols.bedroom_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.too_late_to_leave_time":
                    updated_params.pddl.WonderingProtocols.too_late_to_leave_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.check_bed_after_return_wait_times":
                    updated_params.pddl.WonderingProtocols.check_bed_after_return_wait_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.detect_person_left_house_times":
                    updated_params.pddl.WonderingProtocols.detect_person_left_house_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.wait_for_person_to_return_times":
                    updated_params.pddl.WonderingProtocols.wait_for_person_to_return_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WonderingProtocols.check_if_person_went_to_bed_times":
                    updated_params.pddl.WonderingProtocols.check_if_person_went_to_bed_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FallProtocols.instances":
                    updated_params.pddl.FallProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FallProtocols.wait_times":
                    updated_params.pddl.FallProtocols.wait_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sensor_pills_motion_topic":
                    updated_params.sensor_pills_motion_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sensor_eat_motion_topic":
                    updated_params.sensor_eat_motion_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sensors_door_motion_topic":
                    updated_params.sensors_door_motion_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "sensors_door_open_topic":
                    updated_params.sensors_door_open_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "update_protocol_topic":
                    updated_params.update_protocol_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "world_state_topic":
                    updated_params.world_state_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_frames":
                    updated_params.tf_frames = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_values.bedroom":
                    updated_params.tf_values.bedroom = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_values.kitchen":
                    updated_params.tf_values.kitchen = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_values.couch":
                    updated_params.tf_values.couch = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "tf_values.door":
                    updated_params.tf_values.door = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.medicine_sensor":
                    updated_params.topics.medicine_sensor = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.food_sensor":
                    updated_params.topics.food_sensor = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.bed_side_sensor":
                    updated_params.topics.bed_side_sensor = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "pddl.instances.Landmarks"):
                descriptor = ParameterDescriptor(description="all landmarks in protocols", read_only = False)
                parameter = updated_params.pddl.instances.Landmarks
                self.node_.declare_parameter(self.prefix_ + "pddl.instances.Landmarks", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.instances.Robots"):
                descriptor = ParameterDescriptor(description="all robots in protocols", read_only = False)
                parameter = updated_params.pddl.instances.Robots
                self.node_.declare_parameter(self.prefix_ + "pddl.instances.Robots", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.instances.Persons"):
                descriptor = ParameterDescriptor(description="all people in protocols", read_only = False)
                parameter = updated_params.pddl.instances.Persons
                self.node_.declare_parameter(self.prefix_ + "pddl.instances.Persons", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FoodProtocols.instances"):
                descriptor = ParameterDescriptor(description="food protocols", read_only = False)
                parameter = updated_params.pddl.FoodProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.FoodProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FoodProtocols.eat_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.FoodProtocols.eat_times
                self.node_.declare_parameter(self.prefix_ + "pddl.FoodProtocols.eat_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FoodProtocols.eat_locations"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.FoodProtocols.eat_locations
                self.node_.declare_parameter(self.prefix_ + "pddl.FoodProtocols.eat_locations", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FoodProtocols.check_guide_to_succeeded_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.FoodProtocols.check_guide_to_succeeded_times
                self.node_.declare_parameter(self.prefix_ + "pddl.FoodProtocols.check_guide_to_succeeded_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.FoodProtocols.remind_automated_food_at_times
                self.node_.declare_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_2_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.FoodProtocols.remind_automated_food_at_2_times
                self.node_.declare_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_2_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.instances"):
                descriptor = ParameterDescriptor(description="medicine protocols", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.medicine_location"):
                descriptor = ParameterDescriptor(description="location of medicine", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.medicine_location
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.medicine_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_time"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.take_medication_time
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.check_guide_to_succeeded_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.check_guide_to_succeeded_times
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.check_guide_to_succeeded_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.notify_automated_medicine_at_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.notify_automated_medicine_at_times
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.notify_automated_medicine_at_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.instances"):
                descriptor = ParameterDescriptor(description="wondering protocols", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.outside_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.outside_location
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.outside_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.door_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.door_location
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.door_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.bedroom_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.bedroom_location
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.bedroom_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.too_late_to_leave_time"):
                descriptor = ParameterDescriptor(description="time that the person must not go outside", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.too_late_to_leave_time
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.too_late_to_leave_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.check_bed_after_return_wait_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.check_bed_after_return_wait_times
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.check_bed_after_return_wait_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.detect_person_left_house_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.detect_person_left_house_times
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.detect_person_left_house_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.wait_for_person_to_return_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.wait_for_person_to_return_times
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.wait_for_person_to_return_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WonderingProtocols.check_if_person_went_to_bed_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.WonderingProtocols.check_if_person_went_to_bed_times
                self.node_.declare_parameter(self.prefix_ + "pddl.WonderingProtocols.check_if_person_went_to_bed_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FallProtocols.instances"):
                descriptor = ParameterDescriptor(description="wondering protocols", read_only = False)
                parameter = updated_params.pddl.FallProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.FallProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FallProtocols.wait_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.FallProtocols.wait_times
                self.node_.declare_parameter(self.prefix_ + "pddl.FallProtocols.wait_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sensor_pills_motion_topic"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.sensor_pills_motion_topic
                self.node_.declare_parameter(self.prefix_ + "sensor_pills_motion_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sensor_eat_motion_topic"):
                descriptor = ParameterDescriptor(description="topic for detecting person is eating", read_only = False)
                parameter = updated_params.sensor_eat_motion_topic
                self.node_.declare_parameter(self.prefix_ + "sensor_eat_motion_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sensors_door_motion_topic"):
                descriptor = ParameterDescriptor(description="topic for person going to door", read_only = False)
                parameter = updated_params.sensors_door_motion_topic
                self.node_.declare_parameter(self.prefix_ + "sensors_door_motion_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "sensors_door_open_topic"):
                descriptor = ParameterDescriptor(description="topic for detecting if door is open", read_only = False)
                parameter = updated_params.sensors_door_open_topic
                self.node_.declare_parameter(self.prefix_ + "sensors_door_open_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "update_protocol_topic"):
                descriptor = ParameterDescriptor(description="topic updating active protocol", read_only = False)
                parameter = updated_params.update_protocol_topic
                self.node_.declare_parameter(self.prefix_ + "update_protocol_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "world_state_topic"):
                descriptor = ParameterDescriptor(description="topic for world state", read_only = False)
                parameter = updated_params.world_state_topic
                self.node_.declare_parameter(self.prefix_ + "world_state_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_frames"):
                descriptor = ParameterDescriptor(description="list of tf frame locations that the robot can visit", read_only = False)
                parameter = updated_params.tf_frames
                self.node_.declare_parameter(self.prefix_ + "tf_frames", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_values.bedroom"):
                descriptor = ParameterDescriptor(description="TF for kitchen", read_only = False)
                parameter = updated_params.tf_values.bedroom
                self.node_.declare_parameter(self.prefix_ + "tf_values.bedroom", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_values.kitchen"):
                descriptor = ParameterDescriptor(description="TF for kitchen", read_only = False)
                parameter = updated_params.tf_values.kitchen
                self.node_.declare_parameter(self.prefix_ + "tf_values.kitchen", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_values.couch"):
                descriptor = ParameterDescriptor(description="TF for couch", read_only = False)
                parameter = updated_params.tf_values.couch
                self.node_.declare_parameter(self.prefix_ + "tf_values.couch", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "tf_values.door"):
                descriptor = ParameterDescriptor(description="TF for couch", read_only = False)
                parameter = updated_params.tf_values.door
                self.node_.declare_parameter(self.prefix_ + "tf_values.door", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.medicine_sensor"):
                descriptor = ParameterDescriptor(description="topic for sensor that detect if medication is taken", read_only = False)
                parameter = updated_params.topics.medicine_sensor
                self.node_.declare_parameter(self.prefix_ + "topics.medicine_sensor", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.food_sensor"):
                descriptor = ParameterDescriptor(description="topic for sensor that detect if patient is eating", read_only = False)
                parameter = updated_params.topics.food_sensor
                self.node_.declare_parameter(self.prefix_ + "topics.food_sensor", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.bed_side_sensor"):
                descriptor = ParameterDescriptor(description="topic for sensor that detect if patient is eating", read_only = False)
                parameter = updated_params.topics.bed_side_sensor
                self.node_.declare_parameter(self.prefix_ + "topics.bed_side_sensor", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.Landmarks")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.Landmarks = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.Robots")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.Robots = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.Persons")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.Persons = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.eat_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.eat_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.eat_locations")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.eat_locations = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.check_guide_to_succeeded_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.check_guide_to_succeeded_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.remind_automated_food_at_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_2_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.remind_automated_food_at_2_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.medicine_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.medicine_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.take_medication_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.check_guide_to_succeeded_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.check_guide_to_succeeded_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.notify_automated_medicine_at_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.notify_automated_medicine_at_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.outside_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.outside_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.door_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.door_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.bedroom_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.bedroom_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.too_late_to_leave_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.too_late_to_leave_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.check_bed_after_return_wait_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.check_bed_after_return_wait_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.detect_person_left_house_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.detect_person_left_house_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.wait_for_person_to_return_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.wait_for_person_to_return_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WonderingProtocols.check_if_person_went_to_bed_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WonderingProtocols.check_if_person_went_to_bed_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FallProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FallProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FallProtocols.wait_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FallProtocols.wait_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "sensor_pills_motion_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sensor_pills_motion_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "sensor_eat_motion_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sensor_eat_motion_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "sensors_door_motion_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sensors_door_motion_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "sensors_door_open_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.sensors_door_open_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "update_protocol_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.update_protocol_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "world_state_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.world_state_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_frames")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_frames = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_values.bedroom")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_values.bedroom = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_values.kitchen")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_values.kitchen = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_values.couch")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_values.couch = param.value
            param = self.node_.get_parameter(self.prefix_ + "tf_values.door")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.tf_values.door = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.medicine_sensor")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.medicine_sensor = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.food_sensor")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.food_sensor = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.bed_side_sensor")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.bed_side_sensor = param.value


            self.update_internal_params(updated_params)