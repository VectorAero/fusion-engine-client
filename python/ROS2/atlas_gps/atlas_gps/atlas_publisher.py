import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from atlas_msgs.msg import GPSFix
from sensor_msgs.msg import Imu

import socket
import threading
import time
import queue
import os
import sys

from std_msgs.msg import String

from atlas_gps.fusion_engine_client.messages.defs import MessageType, MessageHeader
# from atlas_gps.fusion_engine_client.messages.ros import *
from atlas_gps.fusion_engine_client.messages.ros import ROS_PoseMessage, ROS_GPSFixMessage, ROS_IMUMessage

import atlas_gps.message_decoder
import atlas_gps.message_decode


class AtlasGPS(Node):

    def __init__(self):
        super().__init__('atlas_gps')
        
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.pose_publisher_ = self.create_publisher(Pose, 'atlas/pose', 10)
        self.gpsfix_publisher_ = self.create_publisher(GPSFix, 'atlas/gpsfix', 10) 
        self.imu_publisher_ = self.create_publisher(Imu, 'atlas/imu', 10) 

        # Initialize a dictionary which maps the message type from the atlas to
        # a class and publisher. Dictionary within dictionary is done for clarity
        # to casual observer
        self.message_type_map = {
            MessageType.ROS_POSE : { 'message_class' : ROS_PoseMessage, 'publisher' : self.pose_publisher_ },
            MessageType.ROS_GPS_FIX : { 'message_class' : ROS_GPSFixMessage, 'publisher' : self.gpsfix_publisher_ },
            MessageType.ROS_IMU : { 'message_class' : ROS_IMUMessage, 'publisher' : self.imu_publisher_ }
        }
        
        self.atlas_driver = AtlasDriver()
        self.atlas_driver.node = self

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        print('timer_callback')
        return


'''
AtlasDriver is the low level connection to the NavPointOne Atlas GPS

There is a queue which holds raw data coming from the Atlas
There are two associated threads.
1. The first thread reads the Atlas, makes a copy of the data, and places it in the queue
2. The second thread removes the data, converts it to a ROS2 message, and publishes it.
'''


class AtlasDriver():

    def __init__(self):
        super().__init__
        self.data = None
        # This is to get to the publisher
        self.node = None
        # Setup thread that reads the Atlas from a UDP socket
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.thread_running = False
        self.message_decoder = atlas_gps.message_decoder.Message_Decoder()

        #  Open the socket to read the Atlas
        self.atlas_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.atlas_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # FIX ME - Should be from a configuration
        # FIX ME - Error out correctly
        self.atlas_socket.bind(('', 12345))
        self.initialize_capture_queue()

    def initialize_capture_queue(self):
        self.queue_lock = threading.Lock()

        self.capture_queue = queue.Queue()

        # thread to capture Atlas stream and place in Queue
        self.capture_event = threading.Event()
        self.capture_thread = threading.Thread(
            target=self.capture_atlas, name='capture')

        # thread to read queue, convert to message and publish
        self.publisher_event = threading.Event()
        self.publish_thread = threading.Thread(
            target=self.publish_events, name='publish')

        self.capture_thread.start()
        self.publish_thread.start()

    def stop_capturing(self):
        # if workers are initialized and running, tell them to stop and wait until stopped
        if hasattr(self, 'capture_event') and self.capture_event != None:
            self.capture_event.set()
        if hasattr(self, 'publisher_event') and self.publisher_event != None:
            self.publisher_event.set()
        if hasattr(self, 'publish_thread') and self.publish_thread.is_alive():
            self.publish_thread.join()
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join()

    def start(self):
        if self.thread_running:
            # FIX ME - Should be ROS2 error message
            print('Already capturing Atlas')
            return None
        # create a thread to read the Atlas
        self.thread_running = True
        self.read_thread = threading.Thread(target=self.read_atlas)
        self.read_thread.start()
        return self

    def stop(self):
        self.thread_running = False
        self.read_thread.join()

    def release(self):
        self.stop()

    # There are three types of messages we process
    # 1. ROS_Pose
    # 2. ROS_GPSFIX
    # 3. ROS_IMU

    def display_atlas_message(self):
        try:
            header = atlas_gps.fusion_engine_client.messages.defs.MessageHeader()
            offset = header.unpack(buffer=self.data)
        except Exception as e:
            print('Decode error: %s' % str(e))

        atlas_gps.message_decode.decode_message(header, self.data, offset)

    def read_atlas(self):
        while self.thread_running:
            try:
                with self.read_lock:
                    self.data = self.atlas_socket.recv(1024)
                    # self.display_atlas_message()

            except RuntimeError:
                # Fix Me
                print('Unable to read atlas')

    def capture_atlas(self):
        while hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            try:
                with self.queue_lock:
                    self.data = self.atlas_socket.recv(1024)
                    # Now place the data in the queue
                    to_add = bytearray(1024)
                    to_add[:] = self.data
                    self.capture_queue.put(to_add)
            except RuntimeError:
                # Fix Me
                print('Capture Error')
            time.sleep(.005)

    def publish_events(self):
        # Loop reading from capture queue and send to ROS topic
        print('publishing events')
        while True:
            if self.publisher_event.is_set():
                break
            try:
                msg_data = self.capture_queue.get(block=True, timeout=2)
            except queue.Empty:
                msg_data = None
            if self.publisher_event.is_set():
                break
            if msg_data is not None:
                # Convert the data into the proper ROS2 message, based on the 
                # message type in the header
                header = MessageHeader() 
                offset = header.unpack(buffer=msg_data)
                # Check for a valid message
                if len(msg_data) != header.calcsize() + header.payload_size_bytes:
                    break
                else:
                    header.validate_crc(msg_data)

                message_info = self.node.message_type_map.get(header.message_type,None) 
                if message_info is not None:
                    # Convert the message to a ROS2 Message
                    ros_class = message_info.get('message_class')
                    # This assumes that the message class and the publisher always are returned correctly
                    ros_class_converter = ros_class()
                    # We pass the ROS2 node here so that if the message has a standard Header, it is easy to fill
                    self.ros_msg = ros_class_converter.unpack_to_msg(buffer=msg_data, offset=offset, node=self.node)
                    node_publisher = message_info.get('publisher')
                    node_publisher.publish(self.ros_msg)
                else:
                    # This is not one of the ROS messages
                    # Could be a correct message format, or something is wrong
                    self.display_atlas_message()
 

            time.sleep(.005)

    def data_as_ROS2_message(self):
        try:
            header = atlas_gps.fusion_engine_client.messages.defs.MessageHeader()
            offset = header.unpack(buffer=self.data)
        except Exception as e:
            print('Decode error: %s' % str(e))
            return None
        ros_msg = self.message_decoder.decode_message(
            header, self.data, offset)
        return ros_msg

    def data_as_message(self, atlas_data):
        try:
            header = atlas_gps.fusion_engine_client.messages.defs.MessageHeader()
            offset = header.unpack(buffer=atlas_data)
        except Exception as e:
            print('Decode error: %s' % str(e))
            return None
        ros_msg = self.message_decoder.decode_message(
            header, atlas_data, offset)
        return ros_msg


def main(args=None):
    rclpy.init(args=args)

    atlas_gps = AtlasGPS()
    rclpy.spin(atlas_gps)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    atlas_gps.atlas_driver.stop_capturing()
    atlas_gps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
