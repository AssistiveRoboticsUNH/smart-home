import zmq
import rclpy
from shr_actions_py.play_audio_action import PlayAudioActionServer
from shr_actions_py.read_script_action import ReadScriptActionServer
from convros_bot.question_test_action import SpeechRecognitionActionServer  # ✅ Import the updated class
from smartthings_ros.display_node import DisplayStatusSubscriber
def main():
    # Initialize ZeroMQ context and shared socket
    zmq_context = zmq.Context()
    zmq_socket = zmq_context.socket(zmq.PUB)
    zmq_socket.bind("tcp://192.168.50.57:5556")  # Shared address and port

    # Initialize ROS 2
    rclpy.init()

    # Create action servers with shared ZeroMQ socket
    play_audio_action_server = PlayAudioActionServer(zmq_socket)
    read_script_action_server = ReadScriptActionServer(zmq_socket)
    question_response_action_server = SpeechRecognitionActionServer(zmq_socket)  # ✅ Added Question Response Server
    display_status_subscriber = DisplayStatusSubscriber(zmq_socket)

    # Spin the action servers
    while True:
        rclpy.spin_once(play_audio_action_server, timeout_sec=0.1)
        rclpy.spin_once(read_script_action_server, timeout_sec=0.1)
        rclpy.spin_once(question_response_action_server, timeout_sec=0.1)  # ✅ Added to the loop
        rclpy.spin_once(display_status_subscriber, timeout_sec=0.1)
