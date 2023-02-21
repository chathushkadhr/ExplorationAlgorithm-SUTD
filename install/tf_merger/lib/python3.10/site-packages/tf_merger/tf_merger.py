#!/usr/bin/env python3
"""
Author : Achala Athukorala
"""

import rclpy
from rclpy import qos
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import traceback
import time
import yaml
import traceback

from tf2_ros import TFMessage, TransformStamped

class TFPrefixer(Node):

    def __init__(self):

        # Init node
        super().__init__('tf_merger_node')

        # Get node name
        self.node_name = self.get_name()
        # Set tf publish rate (default)
        self.rate = 1.0 # Hz

        # Get ros params
        if not (self.get_params()):
            # Exit program if parameters not set properly
            return
        
        # Check namespace validity
        if (self.namespace==""):
            self.get_logger().warning("No namespace provided for tf_merger." +
                                    " TF Merger is only used for multi robot (namespaced tf topics ) scenarios." +
                                    " Exiting!")
            return
        
        # Create TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create QOS profile
        qos_tf = qos.QoSProfile(depth=100,  
                                     reliability = qos.ReliabilityPolicy.RELIABLE,
                                     history = qos.HistoryPolicy.KEEP_LAST,
                                     durability=qos.QoSDurabilityPolicy.VOLATILE,
                                     liveliness = qos.LivelinessPolicy.AUTOMATIC)
        
        qos_tf_static = qos.QoSProfile(depth=100,  
                                     reliability = qos.ReliabilityPolicy.RELIABLE,
                                     history = qos.HistoryPolicy.KEEP_LAST,
                                     durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                     liveliness = qos.LivelinessPolicy.AUTOMATIC)
        
        # Create topics        
        self.local_tf_sub = self.create_subscription(TFMessage, 'tf', self.local_tf_callback, qos_tf)
        self.local_tf_static_sub = self.create_subscription(TFMessage, 'tf_static', self.local_tf_static_callback, qos_tf_static)
        self.local_tf_pub = self.create_publisher(TFMessage, 'tf', qos_tf)
        self.local_tf_static_pub = self.create_publisher(TFMessage, 'tf_static', qos_tf_static)
        
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, qos_tf)
        self.tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, qos_tf_static)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', qos_tf)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', qos_tf_static)

        # Create tf publish timer
        # self.timer = self.create_timer((1.0 / self.rate), self.publish_tf)
        

        # Print node status
        self.get_logger().info(self.node_name + " ready! Namespace: " + self.namespace)

    def get_params(self):
        self.declare_parameter('namespace', 'robot')
        self.declare_parameter('config_file', 'params.yaml')

        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        config_file_name = self.get_parameter('config_file').get_parameter_value().string_value

        # Parse config file
        try:
            pkg_dir = get_package_share_directory("tf_merger")
            config_file_path = os.path.join(pkg_dir, 'config', config_file_name)
            with open(config_file_path) as file:
                params = yaml.load(file, Loader=yaml.FullLoader)

            self.tf_ignore_list = {}
            if not (params['ignore_transforms'] is None):
                for transform in params['ignore_transforms']:
                    self.tf_ignore_list[transform] = (params['ignore_transforms'][transform]['parent_frame'], 
                                                        params['ignore_transforms'][transform]['child_frame'])
                    self.get_logger().info("Ignoring transform : " + transform + "  {" + 
                                        self.tf_ignore_list[transform][0] + " : " + self.tf_ignore_list[transform][1] + "}")
            else:
                self.get_logger().warning("No transforms are ignored! Sharing all transforms")
    
        except Exception as ex:
            self.get_logger().error(self.node_name + " : Error parsing configuration file : " + 
                                        config_file_path + " . Cause: " + str(ex) + " : " + traceback.format_exc())
            return False
        return True


    def tf_callback(self, msg):
        # Add frames to blacklist (To prevent republishing into shared TF topic)
        self.blacklist_frames(msg)
        self.local_tf_pub.publish(msg)
            
    def tf_static_callback(self, msg):
        # Add frames to blacklist (To prevent republishing into shared TF topic)
        self.blacklist_frames(msg)
        self.local_tf_static_pub.publish(msg)

    def local_tf_callback(self, msg):
        filtered_msg = self.filter_blacklisted_transforms(msg)
        if (len(filtered_msg.transforms) > 0):
            prefixed_msg = self.get_prefixed_tf_message(filtered_msg)
            self.tf_pub.publish(prefixed_msg)

    def local_tf_static_callback(self, msg):
        filtered_msg = self.filter_blacklisted_transforms(msg)
        if (len(filtered_msg.transforms) > 0):
            prefixed_msg = self.get_prefixed_tf_message(filtered_msg)
            self.tf_static_pub.publish(prefixed_msg)

    def blacklist_frames(self, tf_message):
        '''
            Add all transforms in a TFMessage to 'ignore list'
        '''
        for stamped_transform in tf_message.transforms:
            parent_child_frame = stamped_transform.header.frame_id + "_to_" + stamped_transform.child_frame_id
            self.tf_ignore_list[parent_child_frame] = (stamped_transform.header.frame_id, stamped_transform.child_frame_id)

    def filter_blacklisted_transforms(self, tf_message):
        filtered_msg = TFMessage()
        # Filter blacklisted transforms
        for stamped_transform in tf_message.transforms:
            parent_child_frames = (stamped_transform.header.frame_id, stamped_transform.child_frame_id)
            if not(parent_child_frames in self.tf_ignore_list.values()):
                filtered_msg._transforms.append(stamped_transform)
        return filtered_msg
        
    def get_prefixed_tf_message(self, tf_message):
        for stamped_transform in tf_message.transforms:
            stamped_transform.header.frame_id = self.add_prefix(stamped_transform.header.frame_id, self.namespace)
            stamped_transform.child_frame_id = self.add_prefix(stamped_transform.child_frame_id, self.namespace)
        return tf_message
    
    def publish_tf(transformStamped, prefix):
        pass
        
    def add_prefix(self, field, prefix):
        if field[0] == '/':
            return prefix + field
        else:
            return prefix + '/' + field
                
def main(args = None):
    rclpy.init(args=args)
    
    try:
        tf_prefixer = TFPrefixer()
        rclpy.spin(tf_prefixer)
    except Exception as ex:
        print("Exception: %s"%str(ex))
        print(traceback.format_exc())
        
        # Time delay before exiting program (For potential respawn actions)
        time.sleep(3)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

