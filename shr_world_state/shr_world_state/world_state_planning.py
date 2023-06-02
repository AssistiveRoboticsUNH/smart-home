#!/usr/bin/env python

import rclpy
from plansys2.domain_parser import DomainParser
from plansys2.problem_parser import ProblemParser

def reset_plansys2():
    # Initialize the ROS node
    rclpy.init()

    # Load the domain file
    domain_parser = DomainParser()
    domain_parser.parse_domain('path/to/pddl_domain_file.pddl')

    # Load the problem file
    problem_parser = ProblemParser()
    problem_parser.parse_problem('path/to/pddl_problem_file.pddl')

    # Get the parsed domain and problem
    domain = domain_parser.get_parsed_domain()
    problem = problem_parser.get_parsed_problem()

    # Perform any additional operations or resets as needed
    print(domain)
    print(problem)
    # Shutdown the ROS node
    rclpy.shutdown()

if __name__ == '__main__':
    reset_plansys2()