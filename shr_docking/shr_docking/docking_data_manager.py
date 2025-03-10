'''
This code is receiving the following data from esp32 and publishing it to topic accordingly.

Limit switch data(Int64)            > /bump 
Battery current voltage(Float32)    > /charging_voltage
IR ring sensor weight(Float32)      > /docking/ir_weight

Author: Akash
'''
import rclpy
from rclpy.node import Node
import asyncio
import serial_asyncio
import serial
from serial.tools import list_ports
import time
from std_msgs.msg import Int32, Float32, Int64  # ROS2 message types


# SERIAL_PORT = "/dev/hello-esp"  # Change this if necessary
BAUD_RATE = 115200
VID = 0x10c4
PID = 0xea60
SERIAL = "0001"

def find_esp_port(baudrate, vid=0x10c4, pid=0xea60, serial_number="0001"):
    # List all available serial ports
    ports = list_ports.comports()
    for port in ports:
        if (port.vid == vid) and (port.pid == pid):
            # Check for the specific serial number if provided
            if serial_number and port.serial_number != serial_number:
                continue
            try:
                # Try opening the port to confirm it's available
                ser = serial.Serial(port.device, baudrate, timeout=0.1)
                ser.close()
                return port.device
            except (serial.SerialException, OSError):
                continue
    print("No Arduino Nano found with the specified VID, PID, and serial number.")
    return None
    # raise RuntimeError("No Arduino Nano found with the specified VID, PID, and serial number.")



class SerialReader(Node):
    def __init__(self):
        super().__init__("serial_reader")

        # Create publishers for the three topics
        self.bump_pub = self.create_publisher(Int64, "/bump", 10)
        self.voltage_pub = self.create_publisher(Float32, "/charging_voltage", 10)
        self.direction_pub = self.create_publisher(Float32, "/docking/ir_weight", 10)
        
        self.serial_active = False
        self.SERIAL_PORT = find_esp_port(baudrate=BAUD_RATE, vid=0x10c4, pid=0xea60, serial_number=SERIAL)
        if self.SERIAL_PORT != None:
            self.get_logger().info(f"Found Docking Serial Port: {self.SERIAL_PORT}.")
        else:
            self.get_logger().warn(f"No Docking Serial Device found!")

        # self.get_logger().info(f"Started Docking SerialReader node. Listening on {SERIAL_PORT}.")
        # Start the asyncio serial reader
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.start_serial_reader())

    async def start_serial_reader(self):
        """Handles serial communication asynchronously."""
        while rclpy.ok():
            # if SERIAL_PORT == None:
            #     self.get_logger().warn("Searching for ESP32 serial port...")
            #     SERIAL_PORT = find_esp_port(baudrate=BAUD_RATE, vid=VID, pid=PID, serial_number="0001")
            # else:
                try:
                    transport, protocol = await serial_asyncio.create_serial_connection(
                        asyncio.get_event_loop(), 
                        lambda: SerialProtocol(self), 
                        self.SERIAL_PORT, BAUD_RATE
                    )
                    await asyncio.sleep(3600)  # Keep connection alive
                except Exception as e:
                    self.get_logger().error(f"Serial connection error: {e}. Retrying in 2 seconds...")
                    await asyncio.sleep(2)
    async def stop_serial_reader(self):
        """Stops serial communication cleanly before shutting down."""
        self.get_logger().info("Closing serial connection...")
        if self.transport:
            self.transport.close()  # Properly close the serial port
            self.transport = None
        if self.serial_task:
            self.serial_task.cancel()  # Cancel the async task
            try:
                await self.serial_task
            except asyncio.CancelledError:
                pass  # Expected when canceling the task

class SerialProtocol(asyncio.Protocol):
    """Handles incoming serial data."""
    def __init__(self, node):
        self.node = node
        self.buffer = b""
        self.voltage_msg = Float32()
        self.bump_msg = Int64()
        self.direction_msg = Float32()
        
        self.bump_state = 0
        self.voltage = 0.0
        self.direction = -1.0


    def data_received(self, data):
        """Reads and processes serial data in real-time."""
        self.buffer += data
        if b"\n" in self.buffer:
            lines = self.buffer.split(b"\n")
            for line in lines[:-1]:
                self.process_line(line.decode("utf-8", errors="ignore").strip())
            self.buffer = lines[-1]  # Keep the remaining incomplete part

    def process_line(self, line):
        """Parses and publishes sensor data, handling missing values."""
        try:
            # Example: "B1,W3.50,V12.43" (Can be incomplete)
            data_parts = line.split(",")

            # Parse only available values
            for part in data_parts:
                # key, value = part.split(":")
                if part[0] == "B":
                    value = part.replace('B', '')
                    try:
                        self.bump_state = int(value)
                        self.bump_msg.data = self.bump_state

                        if self.bump_state in [0,1]: # Bump should be either 1 or 0
                            self.node.bump_pub.publish(self.bump_msg)
                    except:
                        pass

                elif part[0] == "V":
                    value = part.replace("V", "")
                    try:
                        self.voltage = round(float(value), 2)
                        self.voltage_msg.data = self.voltage

                        if self.voltage > 9: ## Voltage should not drop bellow 9V
                            self.node.voltage_pub.publish(self.voltage_msg)
                    except:
                        pass

                elif part[0] == "W":
                    value = part.replace("W", "")
                    try:
                        self.direction = float(value)
                        self.direction_msg.data = self.direction

                        if self.direction < 9: # IR weight should be less than 9
                            self.node.direction_pub.publish(self.direction_msg)
                    except:
                        pass

            
            # Log received values
            
            # curr_time = int(time.time())
            # if curr_time % 5 == 0:
            #     self.node.get_logger().info(f'{line}')
                # self.node.get_logger().info(f"Bump: {self.bump_state}, Voltage: {self.voltage:.2f}V, IR Weight: {self.direction:.2f}")

            

        except Exception as e:
            self.node.get_logger().error(f"Error processing docking microcontroller data: {line}, {e}")

            

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    try:
        asyncio.run(node.start_serial_reader())
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down serial reader node.")
        # asyncio.run(node.stop_serial_reader())  # Gracefully close serial before exit
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
