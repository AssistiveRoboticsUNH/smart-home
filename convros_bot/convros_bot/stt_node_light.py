import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os
os.environ["PYTHONWARNINGS"] = "ignore"

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node_light')
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=0, sample_rate=44100)
        self.timer = self.create_timer(0.1, self.listen_and_publish)

    def listen_and_publish(self):
        try:
            with self.microphone as source:
                msg = String()
                audio = self.recognizer.listen(source, timeout=5)
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Recognized: {text}')
                msg.data = text
                self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().warn('Could not understand')
        except sr.RequestError as e:
            self.get_logger().error(f'Error with Google Speech Recognition service: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down STT Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
