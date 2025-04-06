# flake8: noqa

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

        person_tf = "nathan"
        robot_tf = "base_link"
        class __Pddl:
            class __Instances:
                Persons = ["nathan"]
            instances = __Instances()
            class __Medicineprotocols:
                instances = ["am_meds", "pm_meds"]
                take_medication_times = ["Everyday 09h00m0s/09h30m0s", "Everyday 18h00m0s/18h30m0s"]
            MedicineProtocols = __Medicineprotocols()
            class __Gymreminderprotocols:
                instances = ["gym_reminder"]
                gym_reminder_times = ["Monday,Wednesday,Thursday,Friday 06h30m0s/07h00m0s"]
            GymReminderProtocols = __Gymreminderprotocols()
            class __Medicinerefillreminderprotocols:
                instances = ["medicine_refill_reminder"]
                medicine_refill_reminder_times = ["Everyday 17h00m0s/17h30m0s"]
            MedicineRefillReminderProtocols = __Medicinerefillreminderprotocols()
            class __Medicinerefillpharmacyreminderprotocols:
                instances = ["medicine_pharmacy_reminder"]
                medicine_refill_pharmacy_reminder_times = ["Tuesday,Wednesday,Thursday 09h30m0s/10h00m0s"]
            MedicineRefillPharmacyReminderProtocols = __Medicinerefillpharmacyreminderprotocols()
            class __Walkingprotocols:
                instances = ["walking_reminder"]
                walking_reminder_times = ["Everyday 13h00m0s/14h00m0s"]
            WalkingProtocols = __Walkingprotocols()
        pddl = __Pddl()
        class __Topics:
            time = "/protocol_time"
            person_taking_medicine = "/person_taking_medicine"
            person_eating = "/person_eating"
            robot_charging = "/charging"
            good_weather = "/good_weather"
            display = "/display"
        topics = __Topics()



    class ParamListener:
        def __init__(self, node, prefix=""):
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
                if param.name == self.prefix_ + "pddl.instances.Persons":
                    updated_params.pddl.instances.Persons = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.instances":
                    updated_params.pddl.MedicineProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.take_medication_times":
                    updated_params.pddl.MedicineProtocols.take_medication_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.GymReminderProtocols.instances":
                    updated_params.pddl.GymReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.GymReminderProtocols.gym_reminder_times":
                    updated_params.pddl.GymReminderProtocols.gym_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineRefillReminderProtocols.instances":
                    updated_params.pddl.MedicineRefillReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times":
                    updated_params.pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.instances":
                    updated_params.pddl.MedicineRefillPharmacyReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times":
                    updated_params.pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WalkingProtocols.instances":
                    updated_params.pddl.WalkingProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WalkingProtocols.walking_reminder_times":
                    updated_params.pddl.WalkingProtocols.walking_reminder_times = param.value
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

                if param.name == self.prefix_ + "topics.good_weather":
                    updated_params.topics.good_weather = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "topics.display":
                    updated_params.topics.display = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "pddl.instances.Persons"):
                descriptor = ParameterDescriptor(description="all people in protocols", read_only = False)
                parameter = updated_params.pddl.instances.Persons
                self.node_.declare_parameter(self.prefix_ + "pddl.instances.Persons", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.instances"):
                descriptor = ParameterDescriptor(description="medicine protocols", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.take_medication_times
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.GymReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="gym reminder protocols", read_only = False)
                parameter = updated_params.pddl.GymReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.GymReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.GymReminderProtocols.gym_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.GymReminderProtocols.gym_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.GymReminderProtocols.gym_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineRefillReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="medicine refill protocols", read_only = False)
                parameter = updated_params.pddl.MedicineRefillReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineRefillReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="medicine pharmacy refill phramacy protocols", read_only = False)
                parameter = updated_params.pddl.MedicineRefillPharmacyReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WalkingProtocols.instances"):
                descriptor = ParameterDescriptor(description="gym reminder protocols", read_only = False)
                parameter = updated_params.pddl.WalkingProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.WalkingProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WalkingProtocols.walking_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.WalkingProtocols.walking_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.WalkingProtocols.walking_reminder_times", parameter, descriptor)

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

            if not self.node_.has_parameter(self.prefix_ + "topics.good_weather"):
                descriptor = ParameterDescriptor(description="topic for checking good weather", read_only = False)
                parameter = updated_params.topics.good_weather
                self.node_.declare_parameter(self.prefix_ + "topics.good_weather", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "topics.display"):
                descriptor = ParameterDescriptor(description="topic for changing status for display", read_only = False)
                parameter = updated_params.topics.display
                self.node_.declare_parameter(self.prefix_ + "topics.display", parameter, descriptor)

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
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.Persons")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.Persons = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.take_medication_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.GymReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.GymReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.GymReminderProtocols.gym_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.GymReminderProtocols.gym_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineRefillReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineRefillReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineRefillPharmacyReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WalkingProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WalkingProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WalkingProtocols.walking_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WalkingProtocols.walking_reminder_times = param.value
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
            param = self.node_.get_parameter(self.prefix_ + "topics.good_weather")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.good_weather = param.value
            param = self.node_.get_parameter(self.prefix_ + "topics.display")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.topics.display = param.value
            param = self.node_.get_parameter(self.prefix_ + "person_tf")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.person_tf = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_tf")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.robot_tf = param.value


            self.update_internal_params(updated_params)
