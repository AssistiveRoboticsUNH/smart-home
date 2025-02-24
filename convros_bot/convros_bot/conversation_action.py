import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String, Empty
import time, os
from action_msgs.msg import GoalStatus
from convros_interfaces.action import ConversationRequest
from rclpy.executors import MultiThreadedExecutor
import random
from datetime import datetime
import requests
from bs4 import BeautifulSoup
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from ament_index_python.packages import get_package_share_directory

share_directory = get_package_share_directory('convros_bot')
workspace_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_directory))))

#ros2 action send_goal --feedback conversation_action convros_interfaces/action/ConversationRequest "{command: 'start'}"

class ConversationActionServer(Node):
    def __init__(self):
        super().__init__('conversation_action_node')
        self._action_server = ActionServer(self, ConversationRequest, 'conversation_action',
                                           execute_callback = self.execute_callback,
                                           goal_callback =self.goal_callback,
                                           cancel_callback = self.cancel_callback)
        
        self.declare_parameter('incapability_responses', ["I'm sorry."])
        self.declare_parameter('wake_words', ["Sarah"])
        self.declare_parameter('wake_responses', ["Hello! How can I assist you today?"])
        self.declare_parameter('trigger_keywords', ["weather"])
        

        self.incapability_responses = self.get_parameter('incapability_responses').get_parameter_value().string_array_value
        self.wake_words = self.get_parameter('wake_words').get_parameter_value().string_array_value
        self.wake_responses = self.get_parameter('wake_responses').get_parameter_value().string_array_value
        self.trigger_keywords = self.get_parameter('trigger_keywords').get_parameter_value().string_array_value
        
        self.isAwake = False # To chcek if conversation is active
        self.speech_text_queue = [] # Will store all unprocessed speech in queue
        self.timeElapsed = 0 # How long conversation is running

        self.goal_cancel = False 

        # Subscribe to the speech_text topic (from STT node)
        self.subscription = self.create_subscription(String, 'speech_text', self.speech_callback, 10)  

        # Publisher for tts_text topic (to TTS node)
        self.publisher_ = self.create_publisher(String, 'tts_text', 10)


    async def execute_callback(self, goal_handle):
        command = goal_handle.request.command
        # self.get_logger().info(f'Received command: {command}')

        if command != 'start':
            goal_handle.abort()
            return ConversationRequest.Result(success=False)

        self.isAwake = True
        self.get_logger().info('Conversation started')
        feedback_msg = ConversationRequest.Feedback()
        rate = self.create_rate(5)
        self.start_time = time.time()

        self.speak(random.choice(self.wake_responses))
        self.speech_text_queue = [] # Make speech queue empty

        while self.isAwake:
            if goal_handle.is_cancel_requested or self.goal_cancel:
                self.get_logger().info('Conversation cancelled')
                goal_handle.canceled()
                return ConversationRequest.Result(success=False)
            
            if len(self.speech_text_queue):
                speech_text = self.speech_text_queue.pop(0)
                self.process_speech(speech_text)

            # Optional feedback about progress (e.g., time for how long conversation is active)
            self.timeElapsed = round(time.time()-self.start_time, 2)
            feedback_msg.seconds_running = self.timeElapsed

            goal_handle.publish_feedback(feedback_msg)
            rate.sleep()  # Control recording frequency


        # Finally Finish the action with success
        goal_handle.succeed()
        result = ConversationRequest.Result(success = True)
        return result

    def speech_callback(self, msg):
        # self.get_logger().info(f'Received: {msg.data}')
        self.speech_text_queue.append(msg.data.lower())  # Convert to lowercase for easier matching

    def speak(self, text):
        self.get_logger().info(f'Response:  {text}')
        response_msg = String()
        response_msg.data = text
        self.publisher_.publish(response_msg)


    def process_speech(self, text):                    
        # Check for keywords in the speech text
        for keyword in self.trigger_keywords:
            if keyword in text:
                self.execute_action(keyword)
                return
        # Default response if no keyword matches
        self.speak( self.select_random_incapability_msg())
        
    def execute_action(self, keyword):
        if keyword == "time":
            # Action to announce the current time
            current_time_msg = self.get_current_time()
            self.speak(current_time_msg)
        
        elif keyword in ["weather", "way there", "temperature"]:
            # Action to greet the user
            self.speak('Hmmm... Let me check')
            weather_msg = self.get_weather()
            self.speak(weather_msg)
        
        elif keyword == "come":
            # Action to say goodbye
            self.speak("Okay let me come")

        elif keyword == "go home":
            self.speak("Ok, Thank you. I am going home.")
            self.isAwake = False
        
        elif keyword == "bye":
            # Action to say goodbye
            self.speak("Ok Bye. I am going home.")
            self.isAwake = False
        
        elif keyword in ["thanks", "thank"]:
            self.speak("You are welcome")
            self.isAwake = False
        
        elif keyword == "hear me":
            # Action to say goodbye
            self.speak("Yes I can hear you")

        else:
            # Handle unrecognized keywords
            self.speak("I'm sorry, I don't recognize that command.")

    def select_random_incapability_msg(self):
        return random.choice(self.incapability_responses)
    
    def get_current_time(self):
        current_time = datetime.now().strftime("%I:%M %p")
        message = f"The current time is {current_time}"
        return message


    def get_weather(self, location='USNH0079:1:US'):
        temperature = None
        response_txt = f'Owh Sorry, I could not fetch the weather data.'

        url = f"https://www.weather.com/weather/today/l/{location}"
        # Send GET request to the website
        response = requests.get(url)
        if response.status_code == 200:
            # Parse the HTML content
            soup = BeautifulSoup(response.text, 'html.parser')
            
            # Find temperature element (HTML structure may vary)
            temp_element = soup.find('span', class_='CurrentConditions--tempValue--zUBSz')
            desc_element = soup.find('div', class_='CurrentConditions--phraseValue---VS-k')
            
            if temp_element and desc_element:
                temperature = temp_element.text
                description = desc_element.text
                # print(f"Current temperature: {temperature}")
                # print(f"Weather description: {description}")
                response_txt = f'Current temperature is {temperature}. And its {description}.'
            else:
                print("Could not find weather data. The page structure might have changed.")
        else:
            print("Failed to retrieve data. Check the URL or location code.")
        
        
        print(response_txt)
        return response_txt
    

    def cancel_callback(self, goal_handle):
        if not self.isAwake:
            self.get_logger().warn('Conversation was not started')

        self.isAwake = False
        self.get_logger().info('Conversation Stopped')

        self.get_logger().info(f'Conversation was running for {self.timeElapsed} seconds.)')
        self.goal_cancel = True
        return CancelResponse.ACCEPT
    
    def goal_callback(self, goal_request):
        self.get_logger().info('Conversation start request accepted')
        return GoalResponse.ACCEPT



def main(args=None):
    rclpy.init(args=args)

    action_server = ConversationActionServer()

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