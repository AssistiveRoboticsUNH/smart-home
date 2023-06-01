import rclpy
from rclpy.node import Node
from plansys2_msgs.srv import SetDomain, AffectParam, GetProblem
from plansys2_types import plansys2
import os

class PlanningController(Node):
    def __init__(self):
        super().__init__("planning_controller_py")

    def add_instance(self, instance):
        while not self.add_instance_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')

        request = AffectParam.Request()
        request.param = instance

        future = self.add_instance_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Instance added successfully')
                return True
            else:
                self.get_logger().error(f'Failed to add instance: {response.error_info}')
        else:
            self.get_logger().error('Service call failed')

        return False

    def init_plan(self):

        # # Create a client for the SetDomain service
        # domain_client = self.create_client(SetDomain, "/domain_expert/set_domain")
        # # Wait for the service to be available
        # while not domain_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Service not available, waiting...")
        #
        # # Create the request message
        # request = SetDomain.Request()
        # pkg_path = os.environ["HOME"] + "/smart-home/src/smart-home/shr_plan/pddl/"
        # request.domain = pkg_path + "midnight_domain.pddl"
        # # Send the request and wait for the response
        # domain_future = domain_client.call_async(request)
        # rclpy.spin_until_future_complete(self, domain_future)
        #
        # # Process the response
        # if domain_future.result() is not None:
        #     response = domain_future.result()
        #     if response.success:
        #         self.get_logger().info("Domain set successfully")
        #     else:
        #         self.get_logger().info("Failed to set domain: %s", response.error_info)
        # else:
        #     self.get_logger().info("Service call failed")
        #
        problem_client = self.create_client(SetDomain, "/problem_expert/set_domain")

        while not problem_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Create the request message
        request = SetDomain.Request()
        pkg_path = os.environ["HOME"] + "/smart-home/src/smart-home/shr_plan/pddl/"
        request.domain = pkg_path + "midnight_domain.pddl"
        # Send the request and wait for the response
        problem_future = problem_client.call_async(request)
        rclpy.spin_until_future_complete(self, problem_future)
        # Process the response
        if problem_future.result() is not None:
            response = problem_future.result()
            if response.success:
                self.get_logger().info("Problem set successfully")
            else:
                self.get_logger().info("Failed to set problem: %s", response.error_info)
        else:
            self.get_logger().info("Service call failed")

        # # Create the service client NOT TO BE USED YET
        # add_instance_client = self.create_client(AffectParam, '/add_problem_instance')
        #
        # request_problem = AffectParam.Request()
        # add_instance_client.call_async(request.param)

        search_locations = ["bedroom_robot_pos", "kitchen_robot_pos", "couch_robot_pos", "door_robot_pos"]
        get_problem_client = self.create_client(GetProblem, "/problem_expert/get_problem")
        print(get_problem_client(""))

        for loc in search_locations:
            self.add_instance(plansys2.Instance(loc, "landmark"))

        print(get_problem_client(""))
        # problem_expert_.addInstance(plansys2.Instance("home", "landmark"))
        # problem_expert_.addInstance(plansys2.Instance("pioneer", "robot"))
        #
        # problem_expert_.addInstance(plansys2.Instance(world_state_.patient_name, "person"))
        #
        # problem_expert_.addPredicate(plansys2.Predicate("(robot_at pioneer home)"))
        # problem_expert_.addPredicate(plansys2.Predicate(
        #     "(person_at " + world_state_.patient_name + " " + world_state_.door_location + ")"))



def main(args=None):
    rclpy.init(args=args)
    planning_controller = PlanningController()
    planning_controller.init_plan()
    rclpy.spin(planning_controller)
    planning_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()