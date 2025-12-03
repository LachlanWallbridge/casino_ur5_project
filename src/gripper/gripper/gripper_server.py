#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interface.srv import GripperCmd
import requests
import serial
from serial.tools import list_ports
import time


class GripperServer(Node):

    def __init__(self):
        super().__init__('gripper_server')
        # --- Parameters for serial connection ---
        # If 'auto_detect' is true, the node will scan available serial ports
        # and try to open one that looks like the Teensy/USB-serial device.
        # Otherwise it will attempt to open the provided 'serial_port'.
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('auto_detect', True)
        self.declare_parameter('connect_timeout', 30.0)  # seconds, 0 for infinite
        self.declare_parameter('retry_interval', 2.0)    # seconds between scans

        # Create the service after we attempt to open the serial port so we fail fast
        try:
            self.ser = self.open_serial_port()
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.srv = self.create_service(GripperCmd, 'gripper_cmd', self.gripper_callback)
        self.get_logger().info('Gripper Server ready to receive commands...')
    
        
    def gripper_callback(self, request, response):
        # Validate width
        if not (0 <= request.width <= 180):
            response.success = False
            response.message = f'Width must be between 0-180 (received {request.width})'
            self.get_logger().info(response.message)
            return response
            
        try:
            # Send command to gripper
            self.ser.write(f"{int(request.width)}\n".encode('ascii'))
            self.ser.flush()
            # wait for response
            res = self.ser.readline().decode('ascii', errors='ignore').strip()


            if res == "OK":
                response.success = True
                response.message = f'Gripper set to width {request.width}'
            else:
                response.success = False
                response.message = f'Gripper command failed: {res}'
                
        except Exception as e:
            response.success = False
            response.message = f'Error communicating with gripper: {str(e)}'
            
        return response

    # ----------------------------------------------------------
    def open_serial_port(self):
        """Try to open and verify a serial port. Returns an open serial.Serial instance.

        Behavior:
        - If auto_detect is False, tries the configured serial_port once.
        - If auto_detect is True, scans available ports (via list_ports) and attempts
          to open likely candidates (ttyACM, ttyUSB or ports with 'Teensy' in description).
        - Performs a small write/read handshake to validate the device when possible.
        - Retries until connect_timeout seconds elapse (0 means wait forever).
        """
        port_param = self.get_parameter('serial_port').get_parameter_value().string_value
        auto_detect = self.get_parameter('auto_detect').get_parameter_value().bool_value
        connect_timeout = self.get_parameter('connect_timeout').get_parameter_value().double_value
        retry_interval = self.get_parameter('retry_interval').get_parameter_value().double_value

        deadline = None
        if connect_timeout and connect_timeout > 0:
            deadline = time.time() + connect_timeout

        self.get_logger().info(f"Opening serial port (auto_detect={auto_detect}, port='{port_param}')")

        while True:
            candidates = []
            if not auto_detect and port_param:
                candidates = [port_param]
            else:
                # scan available ports
                for p in list_ports.comports():
                    # prefer explicit Teensy description if available, otherwise any ACM/USB device
                    if 'Teensy' in (p.description or ''):
                        candidates.insert(0, p.device)
                    elif 'ttyACM' in p.device or 'ttyUSB' in p.device:
                        candidates.append(p.device)

            # Remove duplicates while preserving order
            seen = set()
            filtered = []
            for c in candidates:
                if c not in seen:
                    filtered.append(c)
                    seen.add(c)

            for dev in filtered:
                try:
                    self.get_logger().info(f"Attempting to open serial port '{dev}'...")
                    ser = serial.Serial(port=dev, baudrate=9600, timeout=1)
                    # Try a minimal handshake: write newline and read a line.
                    # The gripper firmware typically replies with 'OK' after commands;
                    # if no reply, still accept the port as long as opening succeeded.
                    try:
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                        ser.write(b"\n")
                        ser.flush()
                        resp = ser.readline().decode('ascii', errors='ignore').strip()
                        self.get_logger().debug(f"Serial test response from {dev!s}: '{resp}'")
                    except Exception:
                        # handshake failed but port opened; continue and accept it
                        self.get_logger().debug(f"Handshake on {dev} failed/returned nothing.")

                    self.get_logger().info(f"Connected to gripper on {dev}")
                    return ser
                except Exception as e:
                    self.get_logger().warn(f"Failed to open {dev}: {e}")

            if deadline and time.time() > deadline:
                raise TimeoutError(f"Could not open any serial port before timeout ({connect_timeout}s)")

            self.get_logger().info(f"No suitable serial port found; retrying in {retry_interval}s...")
            time.sleep(retry_interval)



def main(args=None):
    rclpy.init(args=args)
    gripper_server = GripperServer()
    rclpy.spin(gripper_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()