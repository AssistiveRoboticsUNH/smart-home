import aiohttp
import asyncio
import pysmartthings
import threading
import time
import os

import rclpy
from rclpy.node import Node
from pioneer_shr_msg.msg import SmartSensor


class SmartthingsPublisher(Node):

    def __init__(self, smartthings_response, update_period):
        super().__init__('smartthings_publisher')
        self.publisher_ = self.create_publisher(SmartSensor, 'smartthings_sensors', 10)
        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartthings_response = smartthings_response


    def timer_callback(self):
        if self.smartthings_response.updated:
            msg = SmartSensor()
            msg.door_is_open = not self.smartthings_response.closed
            self.publisher_.publish(msg)


class SmartthingsResponse:
    def __init__(self, update_period):
        self.updated = False
        self.closed = None
        self.update_period = update_period

    async def print_devices(self):
        token = os.getenv("SMARTTHINGS_TOKEN")
        device_names = ['multi-sensor']
        async with aiohttp.ClientSession() as session:
            api = pysmartthings.SmartThings(session, token)
            devices = await api.devices()
            while True:
                start = float(time.time_ns() // 1_000_000_000)
                for device in devices:
                    if device.name in device_names:
                        await device.status.refresh()
                        self.closed = device.status.values['contact'] == 'closed'
                end = float(time.time_ns() // 1_000_000_000)
                await asyncio.sleep(self.update_period - (end - start))
                self.updated = True


def main(args=None):
    update_period = 1.5  # 1 sec
    smartthings_response = SmartthingsResponse(update_period)
    x = threading.Thread(target=asyncio.run, args=(smartthings_response.print_devices(),))
    x.start()

    rclpy.init(args=args)

    minimal_publisher = SmartthingsPublisher(smartthings_response, update_period)

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
