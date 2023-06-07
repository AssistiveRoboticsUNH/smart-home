import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import Log
from datetime import datetime
import os 
 
path = "~/.log_rosout"
 
isExist = os.path.exists(path)
if not isExist: 
   os.makedirs(path)
   print(f" {path} is created!")
else:
    print(f'{path} exist')

class LogSubscriber(Node):

    def __init__(self):
        super().__init__('log_rosout')
        self.subscription = self.create_subscription(
            Log,
            'rosout',
            self.listener_callback,
            10)
        self.subscription   



    def listener_callback(self, msg):
        # print('info:', msg)
         
        stamp=msg.stamp
        name=msg.name
        file=msg.file 
        data=msg.msg 
        
        td=datetime.fromtimestamp(stamp.sec).strftime("%m/%d/%Y : %H:%M:%S")

        print(f'\ntime={td}')
        print(f'name={name}')
        print(f'file={file}')
        print(f'data={data}')



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LogSubscriber() 
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
