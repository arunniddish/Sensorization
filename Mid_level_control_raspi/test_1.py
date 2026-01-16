# Camera
from picamera2 import Picamera2
import cv2
import smbus

# Flex Sensors
import busio
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# IMUs
import adafruit_bno055


# Raspberry to Computer
import socket
import json
import struct  # <--- Added missing import

# General
import numpy as np
import RPi.GPIO as gp
import time
import board
import os
import io

# [1] TCP/IP protocol

# Create socket - Sensors
client_sensors = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ip_addr = '10.0.0.8'
client_sensors.connect(('10.0.0.8', 5000))  # Windows PC IP and port
#client.connect((ip_addr, 5000))

# Create socket - Sensors
client_image = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ip_addr = '10.0.0.8'
client_image.connect(('10.0.0.8', 5001))  # Windows PC IP and port
#client.connect((ip_addr, 5000))

# [2] Flex Sensor data

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan = AnalogIn(mcp, MCP.P0)


# [3] IMU data

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Sensor Mode
sensor_mode = sensor.mode

# [4] Camera
cam = Picamera2()
#config = cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}, buffer_count=2)
config = cam.create_still_configuration(main={"size": (640, 480)})
cam.configure(config)
cam.start()
time.sleep(3)

# [] DAQ
try:
	while True:
		#Flex sensor
		flex_volt = chan.voltage

		#IMU
		euler_angles = sensor.euler
		#euler_angles_has_none = None in euler_angles
	
		if euler_angles is not None and all(val is not None for val in euler_angles):
			#Display data
			data_list = [flex_volt] + list(euler_angles)
			print(f"Sending: {data_list}")
			# 'ffff' means 4 floating point numbers
			binary_data = struct.pack('ffff', flex_volt, euler_angles[0], euler_angles[1], euler_angles[2])
			client_sensors.sendall(binary_data)
			
		#Camera
		frame = cam.capture_array()
		
		# Convert to JPEG for compression
		#_, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
		_, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
		img_bytes = buffer.tobytes()
        
        # Send image size first (4 bytes)
		img_size = len(img_bytes)
		client_image.sendall(struct.pack('<I', img_size))
            
        # Then send the image data
		client_image.sendall(img_bytes)
        #print(f"Sent image: {img_size} bytes")
	
		time.sleep(0.2)

except KeyboardInterrupt:
    client_image.close()
    client_sensors.close()
    
