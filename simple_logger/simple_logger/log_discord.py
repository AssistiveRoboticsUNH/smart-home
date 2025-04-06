import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rcl_interfaces.msg import Log
from datetime import datetime
import os, re
from std_msgs.msg import Float32, Int64, Int32, Bool
from simple_logger.discord_bot import DiscordNotifier
import time, json
import asyncio
import subprocess

#log file: ~/.log_rosout
 
path = f"{os.path.expanduser('~')}/.log_rosout"

isExist = os.path.exists(path)
if not isExist: 
   os.makedirs(path)

discord_secrect_file = f"{path}/discord_secret.json"

# Load credentials from JSON file
with open(discord_secrect_file, "r") as file:
    config = json.load(file)

TOKEN = config["TOKEN"]
CHANNEL_ID = config["CHANNEL_ID_test"]

class LogSubscriber(Node):

    def __init__(self):
        super().__init__('log_rosout')
        self.subscription = self.create_subscription(
            Log,
            'rosout',
            self.listener_callback,
            10)
        self.subscription   
        self.exclude_list = ["ZeroMQ", "/home", "high_level_domain_Idle", "starting undocking"]
        # self.simple_log_file=path+'/'+'simplelog.txt'
        
        self.bump_subscriber = self.create_subscription(Int64, 'bump', self.bump_callback, 10)
        self.voltage_subscriber = self.create_subscription(Float32, 'charging_voltage', self.voltage_callback, 10)
        # self.ir_sensor_subscriber = self.create_subscription(Float32, 'docking/ir_weight', self.ir_sensor_callback, 10)
        self.charger_subscriber = self.create_subscription(Int32, 'charging', self.iot_charger_callback, 10)
        self.current_subscriber = self.create_subscription(Float32, 'charging_current', self.current_callback, 10)
        
        self.charger_status = None
        self.bump = None
        self.voltage = None

        self.notified_start = False
        self.prev_info = ""

        try:
            self.notifier = DiscordNotifier(token=TOKEN, channel_id=CHANNEL_ID, on_message_callback=self.on_message_callback)
            self.isDiscord_connected = True
            self.get_logger().info(f'Discord connection established')
        except Exception as e:
            self.get_logger().warn(f'Error in discord connection:{e}')
            self.get_logger().warn(f'Will log offline')
            self.isDiscord_connected = False
        
    def iot_charger_callback(self, msg):
        self.charger_status = msg.data

    def bump_callback(self, msg):
        self.bump = msg.data

    def voltage_callback(self, msg):
        self.voltage = msg.data

    def current_callback(self, msg):
        self.current = msg.data

    def log_offline(self, info):
        '''
        append log text in loacl file
        '''
        # Generate the filename based on the current date
        date_str = datetime.now().strftime("Y%y_M%m_D%d")
        self.simple_log_file = f"{path}/log_{date_str}.txt"

        
        with open(self.simple_log_file, 'a+') as f:
            f.write(info)


    async  def on_message_callback(self, msg):
        # print(f"Received :{msg}")
        self.get_logger().info(f'Received :{msg}')
        time_str = datetime.now().strftime('%m/%d/%Y  %H:%M:%S')
        if msg == "help":
            available_commands = "**Available Commands:**\nstatus \nrunstop\nrun\n"
            await self.notifier.send_message(f'{available_commands}')
        elif msg == "status":
            status = f"Charging: {self.charger_status},\nBump: {self.bump},\nVoltage: {self.voltage}V, \nCurrent: {self.current}Amps"
            self.get_logger().info(f'Robot Status :{status}\n')
            await self.notifier.send_message(f'{time_str} >> Robot Status: \n{status}')
            # asyncio.create_task(notifier.send_message(status))
        
        elif msg == "runstop":
            command = "ros2 service call /runstop std_srvs/srv/SetBool \"{data: true}\""
            result = subprocess.run(command, shell=True, text=True, capture_output=True)
            await self.notifier.send_message(f'{result}')

        elif msg == "run":
            command = "ros2 service call /runstop std_srvs/srv/SetBool \"{data: false}\""
            result = subprocess.run(command, shell=True, text=True, capture_output=True)
            await self.notifier.send_message(f'{result}')

        elif msg == "stop_lidar":
            command = "ros2 service call /stop_motor std_srvs/srv/Empty"
            result = subprocess.run(command, shell=True, text=True, capture_output=True)
            await self.notifier.send_message(f'{result}')
            
        elif msg == "start_lidar":
            command = "ros2 service call /start_motor std_srvs/srv/Empty"
            result = subprocess.run(command, shell=True, text=True, capture_output=True)
            await self.notifier.send_message(f'{result}')




    async def listener_callback(self, msg):
        
        # print('info:', msg) 
        stamp=msg.stamp
        name=msg.name
        file=msg.file 
        data=msg.msg 
        td=datetime.fromtimestamp(stamp.sec).strftime("%m/%d/%Y  %H:%M:%S")


        if not self.notified_start:
            await self.notifier.send_message(f'{td} >> **Robot Started**\n')
            self.log_offline(f'\n{td} >> **Robot Started**\n')
            self.notified_start = True

        
        # print(f'\ntime={td}')
        # print(f'name={name}')
        # print(f'file={file}')
        # print(f'data={data}')

        magic_key='weblog='
        if data.startswith(magic_key):
            data=data.replace(magic_key,'')

            # info_full =f'time={td}\nname={name}\nfile={file}\nmsg={data}\n-----'
            # self.log_offline(info_full)
            # self.get_logger().info(f'>> {info_full}')
            # print(info)
            
            for word in self.exclude_list:
                if word in data:
                    return

            # Remove timestamp if present at the beginning
            cleaned_text = re.sub(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}', '', data)

            # if 'action' in file:
                
            info=f'{td} >> {cleaned_text}\n'
            
            # Dont send same message consecutively
            if self.prev_info == cleaned_text:
                return

            self.get_logger().info(f'>> {info}')

            if self.isDiscord_connected:
                await self.notifier.send_message(info)

            # Log offline as well
            self.log_offline(info)

            
            self.prev_info = cleaned_text
            



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
