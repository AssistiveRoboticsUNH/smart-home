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

        patient_name = "nathan"
        locations = ["bedroom_robot_pos", "door_robot_pos", "kitchen_robot_pos", "couch_robot_pos", "outside"]
        outside_location = "outside"
        medicine_location = "kitchen_robot_pos"
        door_location = "door_robot_pos"
        eat_location = "kitchen_robot_pos"
        take_medication_time = "19h00m"
        eat_time = ["7h00m", "14h30m", "18h00m"]
        bedroom_location = "bedroom_robot_pos"
        too_late_to_leave_time = "20h00m"
        sensor_pills_motion_topic = "smartthings_sensors_motion_pills"
        sensor_eat_motion_topic = "smartthings_sensors_motion_eat"
        sensors_door_motion_topic = "smartthings_sensors_motion_door"
        sensors_door_open_topic = "smartthings_sensors_door"
        update_protocol_topic = "update_protocol"
        world_state_topic = "world_state"
        tf_frames = ["bedroom", "kitchen", "couch", "door"]
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
        class __CallActions:
            actions = ["askcaregiverhelpmedicine1", "askcaregiverhelpmedicine2", "askcaregiverhelpfood1", "askcaregiverhelpfood2", "callcaregiverasktogotobed", "callcaregiverwondering", "callemergency", "callcaregiverasktogotobedafterreturn2", "callcaregiverasktogotobedafterreturn1"]
            phone_numbers = ["7742257735", "7742257735", "7742257735", "7742257735", "7742257735", "7742257735", "7742257735", "7742257735", "7742257735"]
            script_names = ["call_msg_medical.xml", "call_msg_medical.xml", "call_msg_food.xml", "call_msg_food.xml", "call_msg_get_back_to_bed.xml", "call_msg_leaving_house.xml", "call_msg_911.xml", "call_msg_get_back_to_bed.xml", "call_msg_get_back_to_bed.xml"]
        call_actions = __CallActions()
        class __NotifyRecordedActions:
            actions = ["notifyrecordedmedicineat", "remindautomatedfoodat", "notifyrecordedmidnightat"]
            file_names = ["medicine_reminder.mp3", "food_reminder.mp3", "midnight_reminder.mp3"]
            wait_times = [500.0, 500.0, 10.0]
            topics = ["/observe/pill_detection", "/observe/eat_detection", "/decided_to_go_back_to_sleep"]
        notify_recorded_actions = __NotifyRecordedActions()
        class __NotifyAutomatedDeepFakeActions:
            actions = ["notifyautomatedmedicineat", "remindautomatedfoodat2", "notifyautomatedmidnightat"]
            script_names = ["medicine_reminder", "food_reminder", "midnight_reminder"]
            wait_times = [500.0, 500.0, 10.0]
            topics = ["/observe/pill_detection", "/observe/eat_detection", "/decided_to_go_back_to_sleep"]
            voice_names = ["natasha", "natasha", "natasha"]
        notify_automated_deep_fake_actions = __NotifyAutomatedDeepFakeActions()
        class __GuideToActions:
            actions = ["guidepersontolandmarkattempt1", "guidepersontolandmarkattempt2"]
        guide_to_actions = __GuideToActions()
        class __DetectPersonActions:
            actions = ["checkguidetosucceeded1", "checkguidetosucceeded2", "detectperson"]
            timeouts = [60.0, 60.0, 60.0]
        detect_person_actions = __DetectPersonActions()
        class __NoneActions:
            actions = ["updatesuccess1", "updatesuccess2", "updatesuccess3", "updatepersonloc1", "updatepersonloc2", "initguidepersontolandmarkattempt", "initmovetolandmark", "initdetectpersonlefthouse1", "initdetectpersonlefthouse2", "persongooutside1", "persongooutside2", "finishdetectperson1", "finishdetectperson2", "updatepersonlocation1", "updatepersonlocation2", "updatesuccess0", "updatesuccess5", "initcheckbedafterreturn1", "initcheckbedafterreturn2"]
        none_actions = __NoneActions()
        class __DetectPersonLeftHouse:
            actions = ["detectpersonlefthouse1", "detectpersonlefthouse2"]
            timeouts = [30.0, 30.0]
        detect_person_left_house = __DetectPersonLeftHouse()
        class __CheckIfPersonWentToBed:
            actions = ["checkifpersonwenttobed1", "checkifpersonwenttobed2", "checkbedafterreturn1", "checkbedafterreturn2"]
            timeouts = [120.0, 120.0, 120.0, 120.0]
        check_if_person_went_to_bed = __CheckIfPersonWentToBed()
        class __WaitForPersonToReturn:
            actions = ["waitforpersontoreturn1", "waitforpersontoreturn2"]
            timeouts = [300.0, 300.0]
        wait_for_person_to_return = __WaitForPersonToReturn()
        class __MoveRobotActions:
            actions = ["movetolandmark"]
        move_robot_actions = __MoveRobotActions()
        class __PddlInstances:
            Landmark = ["kitchen", "couch", "home", "door"]
            Robot = ["jackel"]
            Person = ["nathan"]
            FoodProtocols = ["breakfast", "lunch", "dinner"]
            MedicineProtocols = ["daily"]
            WonderingProtocols = ["daily"]
            FallProtocols = ["daily"]
        pddl_instances = __PddlInstances()



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

                if param.name == self.prefix_ + "call_actions.actions":
                    updated_params.call_actions.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "call_actions.phone_numbers":
                    updated_params.call_actions.phone_numbers = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "call_actions.script_names":
                    updated_params.call_actions.script_names = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_recorded_actions.actions":
                    updated_params.notify_recorded_actions.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_recorded_actions.file_names":
                    updated_params.notify_recorded_actions.file_names = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_recorded_actions.wait_times":
                    updated_params.notify_recorded_actions.wait_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_recorded_actions.topics":
                    updated_params.notify_recorded_actions.topics = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_automated_deep_fake_actions.actions":
                    updated_params.notify_automated_deep_fake_actions.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_automated_deep_fake_actions.script_names":
                    updated_params.notify_automated_deep_fake_actions.script_names = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_automated_deep_fake_actions.wait_times":
                    updated_params.notify_automated_deep_fake_actions.wait_times = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_automated_deep_fake_actions.topics":
                    updated_params.notify_automated_deep_fake_actions.topics = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "notify_automated_deep_fake_actions.voice_names":
                    updated_params.notify_automated_deep_fake_actions.voice_names = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "guide_to_actions.actions":
                    updated_params.guide_to_actions.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detect_person_actions.actions":
                    updated_params.detect_person_actions.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detect_person_actions.timeouts":
                    updated_params.detect_person_actions.timeouts = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "none_actions.actions":
                    updated_params.none_actions.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detect_person_left_house.actions":
                    updated_params.detect_person_left_house.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detect_person_left_house.timeouts":
                    updated_params.detect_person_left_house.timeouts = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "check_if_person_went_to_bed.actions":
                    updated_params.check_if_person_went_to_bed.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "check_if_person_went_to_bed.timeouts":
                    updated_params.check_if_person_went_to_bed.timeouts = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "wait_for_person_to_return.actions":
                    updated_params.wait_for_person_to_return.actions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "wait_for_person_to_return.timeouts":
                    updated_params.wait_for_person_to_return.timeouts = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "move_robot_actions.actions":
                    updated_params.move_robot_actions.actions = param.value
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

            if not self.node_.has_parameter(self.prefix_ + "call_actions.actions"):
                descriptor = ParameterDescriptor(description="list of pddl call actions", read_only = False)
                parameter = updated_params.call_actions.actions
                self.node_.declare_parameter(self.prefix_ + "call_actions.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "call_actions.phone_numbers"):
                descriptor = ParameterDescriptor(description="list of phone numbers for each call action", read_only = False)
                parameter = updated_params.call_actions.phone_numbers
                self.node_.declare_parameter(self.prefix_ + "call_actions.phone_numbers", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "call_actions.script_names"):
                descriptor = ParameterDescriptor(description="list of pddl call actions", read_only = False)
                parameter = updated_params.call_actions.script_names
                self.node_.declare_parameter(self.prefix_ + "call_actions.script_names", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_recorded_actions.actions"):
                descriptor = ParameterDescriptor(description="list of pddl notify recorded actions", read_only = False)
                parameter = updated_params.notify_recorded_actions.actions
                self.node_.declare_parameter(self.prefix_ + "notify_recorded_actions.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_recorded_actions.file_names"):
                descriptor = ParameterDescriptor(description="list of file names", read_only = False)
                parameter = updated_params.notify_recorded_actions.file_names
                self.node_.declare_parameter(self.prefix_ + "notify_recorded_actions.file_names", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_recorded_actions.wait_times"):
                descriptor = ParameterDescriptor(description="list wait times after prompting", read_only = False)
                parameter = updated_params.notify_recorded_actions.wait_times
                self.node_.declare_parameter(self.prefix_ + "notify_recorded_actions.wait_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_recorded_actions.topics"):
                descriptor = ParameterDescriptor(description="topic for sensor ", read_only = False)
                parameter = updated_params.notify_recorded_actions.topics
                self.node_.declare_parameter(self.prefix_ + "notify_recorded_actions.topics", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_automated_deep_fake_actions.actions"):
                descriptor = ParameterDescriptor(description="list of pddl notify recorded actions", read_only = False)
                parameter = updated_params.notify_automated_deep_fake_actions.actions
                self.node_.declare_parameter(self.prefix_ + "notify_automated_deep_fake_actions.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_automated_deep_fake_actions.script_names"):
                descriptor = ParameterDescriptor(description="list of script names", read_only = False)
                parameter = updated_params.notify_automated_deep_fake_actions.script_names
                self.node_.declare_parameter(self.prefix_ + "notify_automated_deep_fake_actions.script_names", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_automated_deep_fake_actions.wait_times"):
                descriptor = ParameterDescriptor(description="list wait times after prompting", read_only = False)
                parameter = updated_params.notify_automated_deep_fake_actions.wait_times
                self.node_.declare_parameter(self.prefix_ + "notify_automated_deep_fake_actions.wait_times", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_automated_deep_fake_actions.topics"):
                descriptor = ParameterDescriptor(description="topic for sensor ", read_only = False)
                parameter = updated_params.notify_automated_deep_fake_actions.topics
                self.node_.declare_parameter(self.prefix_ + "notify_automated_deep_fake_actions.topics", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "notify_automated_deep_fake_actions.voice_names"):
                descriptor = ParameterDescriptor(description="list of script names", read_only = False)
                parameter = updated_params.notify_automated_deep_fake_actions.voice_names
                self.node_.declare_parameter(self.prefix_ + "notify_automated_deep_fake_actions.voice_names", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "guide_to_actions.actions"):
                descriptor = ParameterDescriptor(description="list of pddl guide to actions", read_only = False)
                parameter = updated_params.guide_to_actions.actions
                self.node_.declare_parameter(self.prefix_ + "guide_to_actions.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detect_person_actions.actions"):
                descriptor = ParameterDescriptor(description="list of pddl detect person actions", read_only = False)
                parameter = updated_params.detect_person_actions.actions
                self.node_.declare_parameter(self.prefix_ + "detect_person_actions.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detect_person_actions.timeouts"):
                descriptor = ParameterDescriptor(description="list of detect person timeout values", read_only = False)
                parameter = updated_params.detect_person_actions.timeouts
                self.node_.declare_parameter(self.prefix_ + "detect_person_actions.timeouts", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "none_actions.actions"):
                descriptor = ParameterDescriptor(description="list of pddl actions that do not have a corresponding action", read_only = False)
                parameter = updated_params.none_actions.actions
                self.node_.declare_parameter(self.prefix_ + "none_actions.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detect_person_left_house.actions"):
                descriptor = ParameterDescriptor(description="list of pddl actions that do not have a corresponding action", read_only = False)
                parameter = updated_params.detect_person_left_house.actions
                self.node_.declare_parameter(self.prefix_ + "detect_person_left_house.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detect_person_left_house.timeouts"):
                descriptor = ParameterDescriptor(description="list of detect person timeout values", read_only = False)
                parameter = updated_params.detect_person_left_house.timeouts
                self.node_.declare_parameter(self.prefix_ + "detect_person_left_house.timeouts", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "check_if_person_went_to_bed.actions"):
                descriptor = ParameterDescriptor(description="list of pddl actions that do not have a corresponding action", read_only = False)
                parameter = updated_params.check_if_person_went_to_bed.actions
                self.node_.declare_parameter(self.prefix_ + "check_if_person_went_to_bed.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "check_if_person_went_to_bed.timeouts"):
                descriptor = ParameterDescriptor(description="list of detect person timeout values", read_only = False)
                parameter = updated_params.check_if_person_went_to_bed.timeouts
                self.node_.declare_parameter(self.prefix_ + "check_if_person_went_to_bed.timeouts", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "wait_for_person_to_return.actions"):
                descriptor = ParameterDescriptor(description="list of pddl actions that do not have a corresponding action", read_only = False)
                parameter = updated_params.wait_for_person_to_return.actions
                self.node_.declare_parameter(self.prefix_ + "wait_for_person_to_return.actions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "wait_for_person_to_return.timeouts"):
                descriptor = ParameterDescriptor(description="list of detect person timeout values", read_only = False)
                parameter = updated_params.wait_for_person_to_return.timeouts
                self.node_.declare_parameter(self.prefix_ + "wait_for_person_to_return.timeouts", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "move_robot_actions.actions"):
                descriptor = ParameterDescriptor(description="pddl action to move robot", read_only = False)
                parameter = updated_params.move_robot_actions.actions
                self.node_.declare_parameter(self.prefix_ + "move_robot_actions.actions", parameter, descriptor)

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
            param = self.node_.get_parameter(self.prefix_ + "call_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.call_actions.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "call_actions.phone_numbers")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.call_actions.phone_numbers = param.value
            param = self.node_.get_parameter(self.prefix_ + "call_actions.script_names")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.call_actions.script_names = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_recorded_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_recorded_actions.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_recorded_actions.file_names")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_recorded_actions.file_names = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_recorded_actions.wait_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_recorded_actions.wait_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_recorded_actions.topics")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_recorded_actions.topics = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_automated_deep_fake_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_automated_deep_fake_actions.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_automated_deep_fake_actions.script_names")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_automated_deep_fake_actions.script_names = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_automated_deep_fake_actions.wait_times")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_automated_deep_fake_actions.wait_times = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_automated_deep_fake_actions.topics")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_automated_deep_fake_actions.topics = param.value
            param = self.node_.get_parameter(self.prefix_ + "notify_automated_deep_fake_actions.voice_names")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.notify_automated_deep_fake_actions.voice_names = param.value
            param = self.node_.get_parameter(self.prefix_ + "guide_to_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.guide_to_actions.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "detect_person_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detect_person_actions.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "detect_person_actions.timeouts")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detect_person_actions.timeouts = param.value
            param = self.node_.get_parameter(self.prefix_ + "none_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.none_actions.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "detect_person_left_house.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detect_person_left_house.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "detect_person_left_house.timeouts")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detect_person_left_house.timeouts = param.value
            param = self.node_.get_parameter(self.prefix_ + "check_if_person_went_to_bed.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.check_if_person_went_to_bed.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "check_if_person_went_to_bed.timeouts")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.check_if_person_went_to_bed.timeouts = param.value
            param = self.node_.get_parameter(self.prefix_ + "wait_for_person_to_return.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.wait_for_person_to_return.actions = param.value
            param = self.node_.get_parameter(self.prefix_ + "wait_for_person_to_return.timeouts")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.wait_for_person_to_return.timeouts = param.value
            param = self.node_.get_parameter(self.prefix_ + "move_robot_actions.actions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.move_robot_actions.actions = param.value
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


            self.update_internal_params(updated_params)