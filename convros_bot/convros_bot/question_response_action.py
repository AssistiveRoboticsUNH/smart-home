'''
# Run the following commnad to receive a yes/no response by asking question
ros2 action send_goal --feedback question_response_action convros_interfaces/action/QuestionResponseRequest "{question: 'Do you want me to show how to operate the Microwave oven?'}"

# feedback is runtime of the conversation

Author: Akash
'''
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String, Empty
import time, os
from action_msgs.msg import GoalStatus
from shr_msgs.action import QuestionResponseRequest
from rclpy.executors import MultiThreadedExecutor
import random
from datetime import datetime
import requests
from bs4 import BeautifulSoup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from convros_bot.speechRecognizer_main import speechRecognizer
from convros_bot.tts_main import TTS
import re
from ament_index_python.packages import get_package_share_directory

share_directory = get_package_share_directory('convros_bot')
workspace_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_directory))))


class QuestionResponseActionServer(Node):
    def __init__(self):
        super().__init__('question_response_action_node')
        self._action_server = ActionServer(self, QuestionResponseRequest, 'question_response_action',
                                           execute_callback = self.execute_callback,
                                           goal_callback =self.goal_callback,
                                           cancel_callback = self.cancel_callback)
        
        self.timeElapsed = 0 # How long conversation is running

        self.response = "" # Will store response here
        self.isAwake = False #if speech recognizer is active

        # Setup speech recognizer
        self.recognizer = speechRecognizer()
        self.timer = self.create_timer(0.05, self.speech_manager) # start a timer

        # Setup Text-to-speech Model
        self.declare_parameter('tts_model', "gTTS")
        self.declare_parameter('api_credential_location', "")
        self.tts_model = self.get_parameter('tts_model').get_parameter_value().string_value

        if self.tts_model == "googleAPI":
            api_credential_fileName = self.get_parameter('api_credential_file').get_parameter_value().string_value
            
            # Define text-to-speech object
            self.tts = TTS(self.tts_model, api_credential_fileName)

            self.get_logger().warn(f'Using Google Text-to-Speech API')
        elif self.tts_model == "gTTS":
            # Define text-to-speech object
            self.tts = TTS(self.tts_model)
            self.get_logger().warn(f'Using gTTS for speech to text service')


    async def execute_callback(self, goal_handle):
        question = goal_handle.request.question

        if question == "":
            goal_handle.abort()
            self.isAwake = False # deactivate the speech recognizer
            return QuestionResponseRequest.Result(response="aborted")


        # self.get_logger().info('Asking question')
        feedback_msg = QuestionResponseRequest.Feedback()
        rate = self.create_rate(10)
        self.start_time = time.time()

        self.tts.speak(question) # Ask the question
        self.speech_text_queue = [] # Make speech queue empty

        self.isAwake = True # Activate the speech recognizer
        # print("Asked, will start loop")
        count = 0 
        while self.isAwake:
            print(goal_handle.is_cancel_requested)
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Conversation cancelled')
                goal_handle.canceled()
                self.isAwake = False # deactivate the speech recognizer
                return QuestionResponseRequest.Result(response="canceled")
            
            while True:
                if count < 3:   
                    if len(self.speech_text_queue):
                        speech_text = self.speech_text_queue.pop(0)
                        # print(speech_text)
                        # if "yes" in speech_text:
                        if bool(re.search(r'\byes\b', speech_text, re.IGNORECASE)):
                            self.response = "yes"
                            self.tts.speak("I got a yes")
                            self.isAwake = False
                            break
                        
                        # elif "no" in speech_text:
                        # Check if 'no' appears as a standalone word
                        elif bool(re.search(r'\bno\b', speech_text, re.IGNORECASE)):
                            self.response = "no"
                            self.tts.speak("I got a no")
                            self.isAwake = False
                            break

                    # Calculate how long conversation is active
                    self.timeElapsed = round(time.time()-self.start_time, 2)
                    # print("self.timeElapsed", self.timeElapsed)
                    if self.timeElapsed > 10:
                        count +=1
                        if count < 3: 
                            print("I didnt get that. I will try again")
                            self.tts.speak("I didnt get that. I will try again") # Ask the question again
                            self.tts.speak(question)
                            self.start_time = time.time()
                        
                else:
                    self.tts.speak("Okay. I consider that as a no")
                    self.response = "no"
                    self.isAwake = False
                    break
                
            # Optional feedback about progress (e.g., time elasped)
            # feedback_msg.seconds_running = self.timeElapsed
            # goal_handle.publish_feedback(feedback_msg)
            
            rate.sleep()  # Control recording frequency

        self.isAwake = False # Deactivate the speech recognizer
        self.recognizer.stop() # Stop the recognizer running in separate thread

        # Finally Finish the action with success
        goal_handle.succeed()
        result = QuestionResponseRequest.Result(response = self.response)
        
        return result

    def speech_registrar(self, text):
        self.get_logger().info(f'Received: {text}')
        self.speech_text_queue.append(text.lower())  # Convert to lowercase for easier matching
        print("speech_registrar")
        
    def speech_manager(self):
        if self.isAwake:
            # self.get_logger().info(f'speak......')
            try:
                self.recognizer.recorder.text(self.speech_registrar)
                    
            except Exception as e:
                self.get_logger().error(f'An Error occurred: {e}')        
    

    def cancel_callback(self, goal_handle):
        if not self.isAwake:
            self.get_logger().warn('Conversation was not started')

        else:
            self.isAwake = False
            self.get_logger().info('Conversation Stopped')

            self.get_logger().info(f'Conversation was running for {self.timeElapsed} seconds.)')
            
        self.recognizer.stop() # Stop the recognizer running in separate thread
        return CancelResponse.ACCEPT
    
    def goal_callback(self, goal_request):
        self.get_logger().info('Question response action request accepted')
        return GoalResponse.ACCEPT



def main(args=None):
    rclpy.init(args=args)

    action_server = QuestionResponseActionServer()

    # try:
    #     rclpy.spin(action_server)
    # except KeyboardInterrupt:
    #     action_server.get_logger().info('Central node stopped by user.')
    # finally:
    #     action_server.destroy_node()
    #     rclpy.shutdown()


    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    executor.spin()


if __name__ == '__main__':
    main()
