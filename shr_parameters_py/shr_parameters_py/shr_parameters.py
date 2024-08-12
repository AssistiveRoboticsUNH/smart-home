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
                LandmarksPerson = ["bedroom", "inside_not_bedroom", "outside"]
                LandmarksRobot = ["home", "designated_space"]
                Persons = ["nathan"]
            instances = __Instances()
            class __Foodprotocols:
                instances = ["breakfast", "lunch"]
                eat_times = ["09h30m0s/10h00m0s", "12h30m00s/13h30m00s"]
                remind_automated_food_at_times = ["0h10m0s", "0h10m0s", "0h10m0s"]
                remind_automated_food_at_2_times = ["0h10m0s", "0h1m0s", "0h10m0s"]
            FoodProtocols = __Foodprotocols()
            class __Medicineprotocols:
                instances = ["am_meds", "pm_meds"]
                take_medication_time = ["8h00m0s/9h00m0s", "20h00m0s/21h00m0s"]
            MedicineProtocols = __Medicineprotocols()
            class __Movereminderprotocol:
                instances = ["move_reminder"]
                move_reminder_time = ["17h00m0s/18h00m0s"]
            MoveReminderProtocol = __Movereminderprotocol()
            class __Internalcheckreminderprotocol:
                instances = ["internal_check_reminder"]
                internal_check_reminder_time = ["20h00m0s/21h00m0s"]
            InternalCheckReminderProtocol = __Internalcheckreminderprotocol()
            class __Practicereminderprotocol:
                instances = ["noon"]
                practice_reminder_time = ["20h00m0s/21h00m0s"]
            PracticeReminderProtocol = __Practicereminderprotocol()
            class __Sleepreminderprotocols:
                instances = ["sleep_reminder"]
                sleep_reminder_times = ["16h45m0s/18h00m0s"]
            SleepReminderProtocols = __Sleepreminderprotocols()
            class __Gymprotocols:
                instances = ["gym_reminder"]
                gym_reminder_times = ["4h30m0s/06h00m0s"]
                wait_times = ["0h10m0s"]
            GymProtocols = __Gymprotocols()
            class __Walkingprotocols:
                instances = ["walking_reminder"]
                walk_reminder_times = ["06h00m0s/07h00m0s"]
            WalkingProtocols = __Walkingprotocols()
            class __Movereminderprotocols:
                instances = ["move_reminder"]
                move_reminder_times = ["015h00m0s/015h30m0s"]
            MoveReminderProtocols = __Movereminderprotocols()
            class __Internalcheckreminderprotocols:
                instances = ["internalcheck_reminder"]
                internalcheck_reminder_times = ["08h45m0s/09h30m0s"]
            InternalCheckReminderProtocols = __Internalcheckreminderprotocols()
            class __Practicereminderprotocols:
                instances = ["practice_reminder"]
                walk_reminder_times = ["07h30m0s/08h30m0s"]
            PracticeReminderProtocols = __Practicereminderprotocols()
            class __Alertprotocols:
                instances = ["night_alert"]
                alert_reminder_times = ["01h00m0s/04h30m0s"]
            AlertProtocols = __Alertprotocols()
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
                if param.name == self.prefix_ + "pddl.instances.LandmarksPerson":
                    updated_params.pddl.instances.LandmarksPerson = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.instances.LandmarksRobot":
                    updated_params.pddl.instances.LandmarksRobot = param.value
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

                if param.name == self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_times":
                    updated_params.pddl.FoodProtocols.remind_automated_food_at_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_2_times":
                    updated_params.pddl.FoodProtocols.remind_automated_food_at_2_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.instances":
                    updated_params.pddl.MedicineProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MedicineProtocols.take_medication_time":
                    updated_params.pddl.MedicineProtocols.take_medication_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MoveReminderProtocol.instances":
                    updated_params.pddl.MoveReminderProtocol.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MoveReminderProtocol.move_reminder_time":
                    updated_params.pddl.MoveReminderProtocol.move_reminder_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.InternalCheckReminderProtocol.instances":
                    updated_params.pddl.InternalCheckReminderProtocol.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.InternalCheckReminderProtocol.internal_check_reminder_time":
                    updated_params.pddl.InternalCheckReminderProtocol.internal_check_reminder_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.PracticeReminderProtocol.instances":
                    updated_params.pddl.PracticeReminderProtocol.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.PracticeReminderProtocol.practice_reminder_time":
                    updated_params.pddl.PracticeReminderProtocol.practice_reminder_time = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.SleepReminderProtocols.instances":
                    updated_params.pddl.SleepReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.SleepReminderProtocols.sleep_reminder_times":
                    updated_params.pddl.SleepReminderProtocols.sleep_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.GymProtocols.instances":
                    updated_params.pddl.GymProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.GymProtocols.gym_reminder_times":
                    updated_params.pddl.GymProtocols.gym_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.GymProtocols.wait_times":
                    updated_params.pddl.GymProtocols.wait_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WalkingProtocols.instances":
                    updated_params.pddl.WalkingProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.WalkingProtocols.walk_reminder_times":
                    updated_params.pddl.WalkingProtocols.walk_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MoveReminderProtocols.instances":
                    updated_params.pddl.MoveReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.MoveReminderProtocols.move_reminder_times":
                    updated_params.pddl.MoveReminderProtocols.move_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.InternalCheckReminderProtocols.instances":
                    updated_params.pddl.InternalCheckReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.InternalCheckReminderProtocols.internalcheck_reminder_times":
                    updated_params.pddl.InternalCheckReminderProtocols.internalcheck_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.PracticeReminderProtocols.instances":
                    updated_params.pddl.PracticeReminderProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.PracticeReminderProtocols.walk_reminder_times":
                    updated_params.pddl.PracticeReminderProtocols.walk_reminder_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.AlertProtocols.instances":
                    updated_params.pddl.AlertProtocols.instances = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pddl.AlertProtocols.alert_reminder_times":
                    updated_params.pddl.AlertProtocols.alert_reminder_times = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "pddl.instances.LandmarksPerson"):
                descriptor = ParameterDescriptor(description="all landmarks in protocols", read_only = False)
                parameter = updated_params.pddl.instances.LandmarksPerson
                self.node_.declare_parameter(self.prefix_ + "pddl.instances.LandmarksPerson", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.instances.LandmarksRobot"):
                descriptor = ParameterDescriptor(description="all landmarks in protocols", read_only = False)
                parameter = updated_params.pddl.instances.LandmarksRobot
                self.node_.declare_parameter(self.prefix_ + "pddl.instances.LandmarksRobot", parameter, descriptor)

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

            if not self.node_.has_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_time"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MedicineProtocols.take_medication_time
                self.node_.declare_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MoveReminderProtocol.instances"):
                descriptor = ParameterDescriptor(description="medicine protocols", read_only = False)
                parameter = updated_params.pddl.MoveReminderProtocol.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.MoveReminderProtocol.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MoveReminderProtocol.move_reminder_time"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MoveReminderProtocol.move_reminder_time
                self.node_.declare_parameter(self.prefix_ + "pddl.MoveReminderProtocol.move_reminder_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocol.instances"):
                descriptor = ParameterDescriptor(description="internal check reminder protocols", read_only = False)
                parameter = updated_params.pddl.InternalCheckReminderProtocol.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocol.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocol.internal_check_reminder_time"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.InternalCheckReminderProtocol.internal_check_reminder_time
                self.node_.declare_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocol.internal_check_reminder_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.PracticeReminderProtocol.instances"):
                descriptor = ParameterDescriptor(description="medicine protocols", read_only = False)
                parameter = updated_params.pddl.PracticeReminderProtocol.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.PracticeReminderProtocol.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.PracticeReminderProtocol.practice_reminder_time"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.PracticeReminderProtocol.practice_reminder_time
                self.node_.declare_parameter(self.prefix_ + "pddl.PracticeReminderProtocol.practice_reminder_time", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.SleepReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="sleep protocols", read_only = False)
                parameter = updated_params.pddl.SleepReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.SleepReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.SleepReminderProtocols.sleep_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.SleepReminderProtocols.sleep_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.SleepReminderProtocols.sleep_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.GymProtocols.instances"):
                descriptor = ParameterDescriptor(description="gym protocols", read_only = False)
                parameter = updated_params.pddl.GymProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.GymProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.GymProtocols.gym_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.GymProtocols.gym_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.GymProtocols.gym_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.GymProtocols.wait_times"):
                descriptor = ParameterDescriptor(description="time to wait for observation", read_only = False)
                parameter = updated_params.pddl.GymProtocols.wait_times
                self.node_.declare_parameter(self.prefix_ + "pddl.GymProtocols.wait_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WalkingProtocols.instances"):
                descriptor = ParameterDescriptor(description="walking protocols", read_only = False)
                parameter = updated_params.pddl.WalkingProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.WalkingProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.WalkingProtocols.walk_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.WalkingProtocols.walk_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.WalkingProtocols.walk_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MoveReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="walking protocols", read_only = False)
                parameter = updated_params.pddl.MoveReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.MoveReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.MoveReminderProtocols.move_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.MoveReminderProtocols.move_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.MoveReminderProtocols.move_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="internalcheck protocols", read_only = False)
                parameter = updated_params.pddl.InternalCheckReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocols.internalcheck_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.InternalCheckReminderProtocols.internalcheck_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocols.internalcheck_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.PracticeReminderProtocols.instances"):
                descriptor = ParameterDescriptor(description="practice protocols", read_only = False)
                parameter = updated_params.pddl.PracticeReminderProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.PracticeReminderProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.PracticeReminderProtocols.walk_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.PracticeReminderProtocols.walk_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.PracticeReminderProtocols.walk_reminder_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.AlertProtocols.instances"):
                descriptor = ParameterDescriptor(description="alert protocols", read_only = False)
                parameter = updated_params.pddl.AlertProtocols.instances
                self.node_.declare_parameter(self.prefix_ + "pddl.AlertProtocols.instances", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pddl.AlertProtocols.alert_reminder_times"):
                descriptor = ParameterDescriptor(description="time that each protocol is triggered", read_only = False)
                parameter = updated_params.pddl.AlertProtocols.alert_reminder_times
                self.node_.declare_parameter(self.prefix_ + "pddl.AlertProtocols.alert_reminder_times", parameter, descriptor)

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
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.LandmarksPerson")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.LandmarksPerson = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.LandmarksRobot")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.LandmarksRobot = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.instances.Persons")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.instances.Persons = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.eat_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.eat_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.remind_automated_food_at_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.FoodProtocols.remind_automated_food_at_2_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.FoodProtocols.remind_automated_food_at_2_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MedicineProtocols.take_medication_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MedicineProtocols.take_medication_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MoveReminderProtocol.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MoveReminderProtocol.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MoveReminderProtocol.move_reminder_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MoveReminderProtocol.move_reminder_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocol.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.InternalCheckReminderProtocol.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocol.internal_check_reminder_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.InternalCheckReminderProtocol.internal_check_reminder_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.PracticeReminderProtocol.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.PracticeReminderProtocol.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.PracticeReminderProtocol.practice_reminder_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.PracticeReminderProtocol.practice_reminder_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.SleepReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.SleepReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.SleepReminderProtocols.sleep_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.SleepReminderProtocols.sleep_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.GymProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.GymProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.GymProtocols.gym_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.GymProtocols.gym_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.GymProtocols.wait_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.GymProtocols.wait_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WalkingProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WalkingProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.WalkingProtocols.walk_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.WalkingProtocols.walk_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MoveReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MoveReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.MoveReminderProtocols.move_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.MoveReminderProtocols.move_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.InternalCheckReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.InternalCheckReminderProtocols.internalcheck_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.InternalCheckReminderProtocols.internalcheck_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.PracticeReminderProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.PracticeReminderProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.PracticeReminderProtocols.walk_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.PracticeReminderProtocols.walk_reminder_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.AlertProtocols.instances")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.AlertProtocols.instances = param.value
            param = self.node_.get_parameter(self.prefix_ + "pddl.AlertProtocols.alert_reminder_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pddl.AlertProtocols.alert_reminder_times = param.value
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