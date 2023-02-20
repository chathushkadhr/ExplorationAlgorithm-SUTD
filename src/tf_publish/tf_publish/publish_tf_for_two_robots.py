
import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose, Transform,TransformStamped


class TestPublishTF(Node):

    def __init__(self):
        super().__init__('test_publish_tf')
                
        self.robot1_pub = self.create_publisher(Float32MultiArray,'robot1/robot_pos', 10)
        self.robot2_pub = self.create_publisher(Float32MultiArray,'robot2/robot_pos', 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.rate = self.create_rate(10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    
    def timer_callback(self):
        try:
            
            #now = rospy.Time.now()
            #tf_listener.waitForTransform('robot1/map','robot1/odom', rospy.Time.now(), rospy.Duration(10.0))
            robot1_map_to_odom = self.tf_buffer.lookup_transform('robot1/map', 'robot1/odom',rclpy.time.Time())
            robot2_odom_to_map = self.tf_buffer.lookup_transform('robot2/odom', 'robot2/map',rclpy.time.Time())
        except:
            print('no tf')
            return
        robot1_odom_to_robot2_odom_Rigidtrans = TransformStamped()
        # Set the frame IDs
        robot1_odom_to_robot2_odom_Rigidtrans.header.frame_id = 'robot2/odom'
        robot1_odom_to_robot2_odom_Rigidtrans.child_frame_id = 'robot1/odom'

        # Set the translation and rotation
        robot1_odom_to_robot2_odom_Rigidtrans.transform.translation.x = 0
        robot1_odom_to_robot2_odom_Rigidtrans.transform.translation.y = 0
        robot1_odom_to_robot2_odom_Rigidtrans.transform.translation.z = 0
        robot1_odom_to_robot2_odom_Rigidtrans.transform.rotation.x = 0
        robot1_odom_to_robot2_odom_Rigidtrans.transform.rotation.y = 0
        robot1_odom_to_robot2_odom_Rigidtrans.transform.rotation.z = 0
        robot1_odom_to_robot2_odom_Rigidtrans.transform.rotation.w = 1
        
        mytf = robot1_map_to_odom * robot2_odom_to_map * robot1_odom_to_robot2_odom_Rigidtrans
        
        
        
        
        
        # robot1_map_to_odom_Rigidtrans = self.tf2Rigidtrans(robot1_map_to_odom, 'robot1/odom', 'robot1/map')
        # robot2_map_to_odom_Rigidtrans = self.tf2Rigidtrans(robot2_map_to_odom, 'robot2/odom', 'robot2/map')
        # robot1_odom_to_robot2_odom_Rigidtrans = self.tf2Rigidtrans(([0,0,0],[0,0,0,1]), 'robot2/odom', 'robot1/odom')
        
        
        # robot1_map_to_robot2_map_Rigidtrans = robot1_map_to_odom_Rigidtrans*robot1_odom_to_robot2_odom_Rigidtrans*robot2_map_to_odom_Rigidtrans.inverse()   

        tf =  self.Rigidtrans2transstamped(robot1_map_to_robot2_map_Rigidtrans)

        self.br.sendTransform(tf)
        # send robot pos
        try:
            #now = rospy.Time.now()
            #tf_listener.waitForTransform('robot1/map','robot1/odom', rospy.Time.now(), rospy.Duration(10.0))
            robot1_map_to_base = self.tf_buffer.lookup_transform('robot1/map', 'robot1/base_footprint', rclpy.time.Time())
            robot2_map_to_base = self.tf_buffer.lookup_transform('robot2/map', 'robot2/base_footprint', rclpy.time.Time())
        except:
            print('no tf')
            return
        
        robot1_pos_msg = Float32MultiArray()
        robot1_pos_msg.data.append(robot1_map_to_base[0][0])
        robot1_pos_msg.data.append(robot1_map_to_base[0][1])
        robot2_pos_msg = Float32MultiArray()
        robot2_pos_msg.data.append(robot2_map_to_base[0][0])
        robot2_pos_msg.data.append(robot2_map_to_base[0][1])
        self.robot1_pub.publish(robot1_pos_msg)
        self.robot2_pub.publish(robot2_pos_msg)
 
    
    
    
    def tf2Rigidtrans(self,trans, from_frame, to_frame):
        rotation_quaternion = np.asarray([trans[1][3], trans[1][0], trans[1][1], trans[1][2]])
        T_trans = np.asarray([trans[0][0], trans[0][1], trans[0][2]])
        T_qua2rota = RigidTransform(rotation_quaternion, T_trans, from_frame = from_frame, to_frame=to_frame)
        return T_qua2rota

    def Rigidtrans2transstamped(self,Rigidtrans):
        trans = tf2_ros.TransformStamped()
        trans.header.stamp = self.get_clock().now()
        trans.header.frame_id = Rigidtrans.to_frame
        trans.child_frame_id = Rigidtrans.from_frame
        trans.transform = self.pose2trans(Rigidtrans.pose_msg)
        return trans

    def pose2trans(pose):
        trans = Transform()
        trans.rotation = pose.orientation
        trans.translation =pose.position
        return trans





def main(args=None):
    rclpy.init(args=args)
    node = TestPublishTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
