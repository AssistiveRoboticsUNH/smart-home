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

        take_medication_time = "19h00m"
        eat_time = ["7h00m", "14h30m", "18h00m"]
        too_late_to_leave_time = "20h00m"
        sensor_pills_motion_topic = "smartthings_sensors_motion_pills"
        sensor_eat_motion_topic = "smartthings_sensors_motion_eat"
        sensors_door_motion_topic = "smartthings_sensors_motion_door"
        sensors_door_open_topic = "smartthings_sensors_door"
        update_protocol_topic = "update_protocol"
        world_state_topic = "world_state"
        tf_frames = ["bedroom", "kitchen", "couch", "door"]
        class __Locations:
            outside_location = "outside"
            medicine_location = "kitchen"
            door_location = "door"
            eat_location = "kitchen"
            bedroom_location = "bedroom"
        locations = __Locations()
        class __PddlInstances:
            Landmark = ["kitchen", "couch", "home", "door", "outside"]
            Robot = ["jackel"]
            Person = ["nathan"]
            FoodProtocols = ["breakfast", "lunch", "dinner"]
            MedicineProtocols = ["daily"]
            WonderingProtocols = ["daily"]
            FallProtocols = ["daily"]
        pddl_instances = __PddlInstances()
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
                if param.name == self.prefix_ + "locations.outside_location":
                    updated_params.locations.outside_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "locations.medicine_location":
                    updated_params.locations.medicine_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "locations.door_location":
                    updated_params.locations.door_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "locations.eat_location":
                    updated_params.locations.eat_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "locations.bedroom_location":
                    updated_params.locations.bedroom_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.Landmark":
                    updated_params.pddl_instances.Landmark = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.Robot":
                    updated_params.pddl_instances.Robot = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.Person":
                    updated_params.pddl_instances.Person = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.FoodProtocols":
                    updated_params.pddl_instances.FoodProtocols = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.MedicineProtocols":
                    updated_params.pddl_instances.MedicineProtocols = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.WonderingProtocols":
                    updated_params.pddl_instances.WonderingProtocols = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl_instances.FallProtocols":
                    updated_params.pddl_instances.FallProtocols = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "take_medication_time":
                    updated_params.take_medication_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "eat_time":
                    updated_params.eat_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "too_late_to_leave_time":
                    updated_params.too_late_to_leave_time = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "locations.outside_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.locations.outside_location
                self.node_.declare_parameter(self.prefix_ + "locations.outside_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "locations.medicine_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.locations.medicine_location
                self.node_.declare_parameter(self.prefix_ + "locations.medicine_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "locations.door_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.locations.door_location
                self.node_.declare_parameter(self.prefix_ + "locations.door_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "locations.eat_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.locations.eat_location
                self.node_.declare_parameter(self.prefix_ + "locations.eat_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "locations.bedroom_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.locations.bedroom_location
                self.node_.declare_parameter(self.prefix_ + "locations.bedroom_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.Landmark"):
                descriptor = ParameterDescriptor(description="all landmarks in protocols", read_only = False)
                parameter = updated_params.pddl_instances.Landmark
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.Landmark", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.Robot"):
                descriptor = ParameterDescriptor(description="all robots in protocols", read_only = False)
                parameter = updated_params.pddl_instances.Robot
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.Robot", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.Person"):
                descriptor = ParameterDescriptor(description="all people in protocols", read_only = False)
                parameter = updated_params.pddl_instances.Person
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.Person", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.FoodProtocols"):
                descriptor = ParameterDescriptor(description="food protocols", read_only = False)
                parameter = updated_params.pddl_instances.FoodProtocols
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.FoodProtocols", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.MedicineProtocols"):
                descriptor = ParameterDescriptor(description="medicine protocols", read_only = False)
                parameter = updated_params.pddl_instances.MedicineProtocols
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.MedicineProtocols", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.WonderingProtocols"):
                descriptor = ParameterDescriptor(description="wondering protocols", read_only = False)
                parameter = updated_params.pddl_instances.WonderingProtocols
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.WonderingProtocols", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl_instances.FallProtocols"):
                descriptor = ParameterDescriptor(description="wondering protocols", read_only = False)
                parameter = updated_params.pddl_instances.FallProtocols
                self.node_.declare_parameter(self.prefix_ + "pddl_instances.FallProtocols", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "take_medication_time"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.take_medication_time
                self.node_.declare_parameter(self.prefix_ + "take_medication_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "eat_time"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.eat_time
                self.node_.declare_parameter(self.prefix_ + "eat_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "too_late_to_leave_time"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.too_late_to_leave_time
                self.node_.declare_parameter(self.prefix_ + "too_late_to_leave_time", parameter, descriptor)

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
            param = self.node_.get_parameter(self.prefix_ + "locations.outside_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.locations.outside_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "locations.medicine_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.locations.medicine_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "locations.door_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.locations.door_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "locations.eat_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.locations.eat_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "locations.bedroom_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.locations.bedroom_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.Landmark")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.Landmark = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.Robot")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.Robot = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.Person")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.Person = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.FoodProtocols")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.FoodProtocols = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.MedicineProtocols")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.MedicineProtocols = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.WonderingProtocols")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.WonderingProtocols = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl_instances.FallProtocols")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl_instances.FallProtocols = param.value
            param = self.node_.get_parameter(self.prefix_ + "take_medication_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.take_medication_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "eat_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.eat_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "too_late_to_leave_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.too_late_to_leave_time = param.value
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