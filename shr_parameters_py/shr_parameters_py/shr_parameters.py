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

        caregiver_phone_number = "7742257735"
        emergency_phone_number = "7742257735"
        person_tf = "nathan"
        robot_tf = "base_link"
        class __Pddl:
            class __Instances:
                Landmarks = ["kitchen", "couch", "home", "door", "outside", "hallway", "bedroom", "dining_room"]
                Robots = ["jackal"]
                Persons = ["nathan"]
            instances = __Instances()
            class __Foodprotocols:
                instances = ["breakfast", "lunch", "dinner"]
                eat_times = ["7h0m0s/8h0m0s", "14h30m0s/16h00m0s", "18h00m0s/19h0m0s"]
                eat_locations = ["kitchen", "kitchen", "kitchen"]
                check_guide_to_succeeded_times = ["0h1m0s", "0h1m0s", "0h1m0s"]
                remind_automated_food_at_times = ["0h10m0s", "0h10m0s", "0h10m0s"]
                remind_automated_food_at_2_times = ["0h10m0s", "0h1m0s", "0h10m0s"]
            FoodProtocols = __Foodprotocols()
            class __Medicineprotocols:
                instances = ["daily_med"]
                medicine_location = ["kitchen"]
                take_medication_time = ["8h0m0s/9h0m0s"]
            MedicineProtocols = __Medicineprotocols()
            class __Wanderingprotocols:
                instances = ["daily_wand"]
                outside_location = ["outside"]
                door_location = ["door"]
                bedroom_location = ["bedroom"]
                too_late_to_leave_time = ["20h0m0s/6h0m0s"]
            WanderingProtocols = __Wanderingprotocols()
            class __Fallprotocols:
                instances = ["daily_fall"]
                wait_times = ["0h10m0s"]
            FallProtocols = __Fallprotocols()
        pddl = __Pddl()
        class __Topics:
            time = "/protocol_time"
            person_taking_medicine = "/person_taking_medicine"
            person_eating = "/person_eating"
            robot_charging = "/charging"
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

                if param.name == self.prefix_ + "pddl.WanderingProtocols.instances":
                    updated_params.pddl.WanderingProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WanderingProtocols.outside_location":
                    updated_params.pddl.WanderingProtocols.outside_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WanderingProtocols.door_location":
                    updated_params.pddl.WanderingProtocols.door_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WanderingProtocols.bedroom_location":
                    updated_params.pddl.WanderingProtocols.bedroom_location = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WanderingProtocols.too_late_to_leave_time":
                    updated_params.pddl.WanderingProtocols.too_late_to_leave_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FallProtocols.instances":
                    updated_params.pddl.FallProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FallProtocols.wait_times":
                    updated_params.pddl.FallProtocols.wait_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "caregiver_phone_number":
                    updated_params.caregiver_phone_number = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "emergency_phone_number":
                    updated_params.emergency_phone_number = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.time":
                    updated_params.topics.time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.person_taking_medicine":
                    updated_params.topics.person_taking_medicine = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.person_eating":
                    updated_params.topics.person_eating = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.robot_charging":
                    updated_params.topics.robot_charging = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "person_tf":
                    updated_params.person_tf = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "robot_tf":
                    updated_params.robot_tf = param.value
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

            if not self.node_.has_parameter(self.prefix_ + "pddl.WanderingProtocols.instances"):
                descriptor = ParameterDescriptor(description="wandering protocols", read_only = False)
                parameter = updated_params.pddl.WanderingProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.WanderingProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WanderingProtocols.outside_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.WanderingProtocols.outside_location
                self.node_.declare_parameter(self.prefix_ + "pddl.WanderingProtocols.outside_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WanderingProtocols.door_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.WanderingProtocols.door_location
                self.node_.declare_parameter(self.prefix_ + "pddl.WanderingProtocols.door_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WanderingProtocols.bedroom_location"):
                descriptor = ParameterDescriptor(description="topic for detecting pills being taken", read_only = False)
                parameter = updated_params.pddl.WanderingProtocols.bedroom_location
                self.node_.declare_parameter(self.prefix_ + "pddl.WanderingProtocols.bedroom_location", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WanderingProtocols.too_late_to_leave_time"):
                descriptor = ParameterDescriptor(description="time that the person must not go outside", read_only = False)
                parameter = updated_params.pddl.WanderingProtocols.too_late_to_leave_time
                self.node_.declare_parameter(self.prefix_ + "pddl.WanderingProtocols.too_late_to_leave_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FallProtocols.instances"):
                descriptor = ParameterDescriptor(description="wandering protocols", read_only = False)
                parameter = updated_params.pddl.FallProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.FallProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.FallProtocols.wait_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.FallProtocols.wait_times
                self.node_.declare_parameter(self.prefix_ + "pddl.FallProtocols.wait_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "caregiver_phone_number"):
                descriptor = ParameterDescriptor(description="caregiver phone number", read_only = False)
                parameter = updated_params.caregiver_phone_number
                self.node_.declare_parameter(self.prefix_ + "caregiver_phone_number", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "emergency_phone_number"):
                descriptor = ParameterDescriptor(description="emergency phone number", read_only = False)
                parameter = updated_params.emergency_phone_number
                self.node_.declare_parameter(self.prefix_ + "emergency_phone_number", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.time"):
                descriptor = ParameterDescriptor(description="topic for protocol clock time", read_only = False)
                parameter = updated_params.topics.time
                self.node_.declare_parameter(self.prefix_ + "topics.time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.person_taking_medicine"):
                descriptor = ParameterDescriptor(description="topic for sensor that detect if medication is taken", read_only = False)
                parameter = updated_params.topics.person_taking_medicine
                self.node_.declare_parameter(self.prefix_ + "topics.person_taking_medicine", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.person_eating"):
                descriptor = ParameterDescriptor(description="topic for sensor that detect if patient is eating", read_only = False)
                parameter = updated_params.topics.person_eating
                self.node_.declare_parameter(self.prefix_ + "topics.person_eating", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.robot_charging"):
                descriptor = ParameterDescriptor(description="topic for smart plug that detect if robot is charging", read_only = False)
                parameter = updated_params.topics.robot_charging
                self.node_.declare_parameter(self.prefix_ + "topics.robot_charging", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "person_tf"):
                descriptor = ParameterDescriptor(description="person tf frame id", read_only = False)
                parameter = updated_params.person_tf
                self.node_.declare_parameter(self.prefix_ + "person_tf", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "robot_tf"):
                descriptor = ParameterDescriptor(description="robot tf frame id", read_only = False)
                parameter = updated_params.robot_tf
                self.node_.declare_parameter(self.prefix_ + "robot_tf", parameter, descriptor)

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
            param = self.node_.get_parameter(self.prefix_ + "pddl.WanderingProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WanderingProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WanderingProtocols.outside_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WanderingProtocols.outside_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WanderingProtocols.door_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WanderingProtocols.door_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WanderingProtocols.bedroom_location")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WanderingProtocols.bedroom_location = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WanderingProtocols.too_late_to_leave_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WanderingProtocols.too_late_to_leave_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FallProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FallProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FallProtocols.wait_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FallProtocols.wait_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "caregiver_phone_number")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.caregiver_phone_number = param.value
            param = self.node_.get_parameter(self.prefix_ + "emergency_phone_number")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.emergency_phone_number = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.time = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.person_taking_medicine")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.person_taking_medicine = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.person_eating")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.person_eating = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.robot_charging")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.robot_charging = param.value
            param = self.node_.get_parameter(self.prefix_ + "person_tf")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.person_tf = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_tf")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.robot_tf = param.value


            self.update_internal_params(updated_params)