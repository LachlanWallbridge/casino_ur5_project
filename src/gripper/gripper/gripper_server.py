#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interface.srv import GripperCmd
import requests
import serial


class GripperServer(Node):

    def __init__(self):
        super().__init__('gripper_server')
        self.srv = self.create_service(GripperCmd, 'gripper_cmd', self.gripper_callback)
        self.get_logger().info('Gripper Server ready to receive commands...')
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
    
        
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



def main(args=None):
    rclpy.init(args=args)
    gripper_server = GripperServer()
    rclpy.spin(gripper_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()