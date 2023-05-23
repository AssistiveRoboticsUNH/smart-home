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



class parameters:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        patient_name = "nathan"
        locations = ["bedroom_robot_pos", "door_robot_pos", "kitchen_robot_pos", "couch_robot_pos", "outside"]
        outside_location = "outside"
        medicine_location = "kitchen_robot_pos"
        door_location = "door_robot_pos"
        eat_location = "kitchen_robot_pos"
        take_medication_time = "8h15m"
        eat_time = ["9h30m", "14h30m", "16h30m"]
        bedroom_location = "bedroom_robot_pos"



    class ParamListener:
        def __init__(self, node, prefix=""):
            node.declare_parameter('my_parameter', 'world')
            self.prefix_ = prefix
            self.params_ = parameters.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("parameters." + prefix)

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
                if param.name == self.prefix_ + "patient_name":
                    updated_params.patient_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "locations":
                    updated_params.locations = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "outside_location":
                    updated_params.outside_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "medicine_location":
                    updated_params.medicine_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "door_location":
                    updated_params.door_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "eat_location":
                    updated_params.eat_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "take_medication_time":
                    updated_params.take_medication_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "eat_time":
                    updated_params.eat_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "bedroom_location":
                    updated_params.bedroom_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "patient_name"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.patient_name
                self.node_.declare_parameter(self.prefix_ + "patient_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "locations"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.locations
                self.node_.declare_parameter(self.prefix_ + "locations", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "outside_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.outside_location
                self.node_.declare_parameter(self.prefix_ + "outside_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "medicine_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.medicine_location
                self.node_.declare_parameter(self.prefix_ + "medicine_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "door_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.door_location
                self.node_.declare_parameter(self.prefix_ + "door_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "eat_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.eat_location
                self.node_.declare_parameter(self.prefix_ + "eat_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "take_medication_time"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.take_medication_time
                self.node_.declare_parameter(self.prefix_ + "take_medication_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "eat_time"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.eat_time
                self.node_.declare_parameter(self.prefix_ + "eat_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "bedroom_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.bedroom_location
                self.node_.declare_parameter(self.prefix_ + "bedroom_location", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "patient_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.patient_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "locations")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.locations = param.value
            param = self.node_.get_parameter(self.prefix_ + "outside_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.outside_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "medicine_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.medicine_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "door_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.door_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "eat_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.eat_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "take_medication_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.take_medication_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "eat_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.eat_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "bedroom_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.bedroom_location = param.value


            self.update_internal_params(updated_params)