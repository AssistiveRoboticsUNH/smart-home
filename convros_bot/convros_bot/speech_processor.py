import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import random
from datetime import datetime
import requests
from bs4 import BeautifulSoup
from rclpy.action import ActionClient
from convros_interfaces.action import ConversationRequest

class CentralNode(Node):
    def __init__(self):
        super().__init__('speech_processor')

        self.declare_parameter('wake_words', ["Sarah"])
        self.declare_parameter('wake_responses', ["Hello! How can I assist you today?"])
        # self.declare_parameter('trigger_keywords', ["weather"])
        # self.declare_parameter('incapability_responses', ["I'm sorry."])
        

        self.wake_words = self.get_parameter('wake_words').get_parameter_value().string_array_value
        self.wake_responses = self.get_parameter('wake_responses').get_parameter_value().string_array_value
        # self.trigger_keywords = self.get_parameter('trigger_keywords').get_parameter_value().string_array_value
        # self.incapability_responses = self.get_parameter('incapability_responses').get_parameter_value().string_array_value
        #         
        self.isAwake = False

        # Subscribe to the speech_text topic (from STT node)
        self.subscription = self.create_subscription(String, 'speech_text', self.speech_callback, 10)  

        # Publisher for tts_text topic (to TTS node)
        self.publisher_ = self.create_publisher(String, 'tts_text', 10)

        self.isCalled_publisher = self.create_publisher(Bool, 'iscalled', 10) # Will publish True if robot is called by its name

        self.conversation_action_client = ActionClient(self, ConversationRequest, 'conversation_action')

    def speak(self, text):
        self.get_logger().info(f'Response:  {text}')
        response_msg = String()
        response_msg.data = text
        self.publisher_.publish(response_msg)

    def speech_callback(self, msg):
        # self.get_logger().info(f'Received: {msg.data}')
        speech_text = msg.data.lower()  # Convert to lowercase for easier matching

        if not self.isAwake:
            for word in self.wake_words:
                if word in speech_text:
                    self.isAwake = True
                    # true_msg = Bool()
                    # true_msg.data = True
                    self.isCalled_publisher.publish(Bool(data=True))
                    self.speak(random.choice(self.wake_responses))
                    self.start_conversation_action()
                    return
        

    # def start_conversation_action(self, command="start"):
    #     #Either 'start' or 'stop' path recording
    #     goal_msg = ConversationRequest.Goal()
    #     goal_msg.command = command

    #     self.conversation_action_client.wait_for_server()
    #     self.get_logger().info(f'Conversation: {command}')
    #     return self.conversation_action_client.send_goal_async(goal_msg)

    def start_conversation_action(self, command="start"):
        """
        Sends a goal to the conversation action server and tracks feedback.

        Args:
            command (str): Command to send to the server, 'start'
        """
        goal_msg = ConversationRequest.Goal()
        goal_msg.command = command

        self.conversation_action_client.wait_for_server()
        self.get_logger().debug(f'Sending conversation command: {command}')

        send_goal_future = self.conversation_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_conversation_action(self):
        """
        Sends a cancel request to the action server for the current goal.
        """
        if self.current_goal_handle is None:
            self.get_logger().info("No active goal to cancel.")
            return

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def feedback_callback(self, feedback_msg):
        """Updates and prints feedback received from the action server."""
        feedback = feedback_msg.feedback
        self.feedback_seconds_running = feedback.seconds_running
        # self.get_logger().info(f"Feedback: Conversation running for {self.feedback_seconds_running:.2f} seconds")

    def goal_response_callback(self, future):
        """Handles the response to the goal request."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Conversation request rejected')
            self.isAwake = False
            return

        # self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle  # Save the goal handle for cancellation
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def cancel_done_callback(self, future):
        """Handles the result of a cancel request."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            # self.get_logger().info("Goal successfully canceled.")
            pass
        else:
            self.get_logger().info("Conversation cancel request was rejected.")
        self.isAwake = False

    def result_callback(self, future):
        """Handles the result of the action."""
        result = future.result().result
        if result.success:
            self.get_logger().info('Conversation completed successfully!')
        else:
            self.get_logger().info('Conversation Ended!')
        self.isAwake = False
        self.current_goal_handle = None  # Clear the current goal handle

    
def main(args=None):
    rclpy.init(args=args)

    central_node = CentralNode()

    try:
        rclpy.spin(central_node)
    except KeyboardInterrupt:
        central_node.get_logger().info('Central node stopped by user.')
    finally:
        central_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
