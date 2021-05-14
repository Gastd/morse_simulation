import roslibpy
import rclpy
from std_msgs.msg import Float32, Int32, String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Bridge:
    def __init__(self, topic_name, topic_type):
        self.callbacks = {'std_msgs/Float32': self.FloatCallback, 
                'std_msgs/Int32': self.IntCallback, 
                'std_msgs/String': self.StringCallback, 
                'nav_msgs/Odometry': self.OdometryCallback, 
                'geometry_msgs/PoseStamped': self.PoseCallback, 
                'sensor_msgs/LaserScan': self.LaserCallback}
        self.ros2_pub = None
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.topic_bridge = roslibpy.Topic(client, topic_name, topic_type)
        
        self.topic_bridge.subscribe(self.callbacks[self.topic_type])

    def pub(self, ros_type, msg):
        if self.ros2_pub is None:
            self.ros2_pub = node.create_publisher(ros_type, self.topic_name, 10)
        self.ros2_pub.publish(msg)


    def FloatCallback(self, bridge):
        msg = Float32()
        msg.data = bridge['data']
        self.pub(Float32, msg)

    def IntCallback(self, bridge):
        msg = Int32()
        msg.data = bridge['data']
        self.pub(Int32, msg)

    def StringCallback(self,bridge):
        msg = String()
        msg.data = bridge['data']
        self.pub(String, msg)
        
    def PoseCallback(self, bridge):
        msg = Pose()
        msg.position.x = bridge['pose']['position']['x']
        msg.position.y = bridge['pose']['position']['y']
        msg.position.z = bridge['pose']['position']['z']
        msg.orientation.x = bridge['pose']['orientation']['x']
        msg.orientation.y = bridge['pose']['orientation']['y']
        msg.orientation.z = bridge['pose']['orientation']['z']
        self.pub(Pose, msg)

    def OdometryCallback(self, bridge):
        msg = Odometry()
        msg.pose.pose.position.x = bridge['pose']['pose']['position']['x']
        msg.pose.pose.position.y = bridge['pose']['pose']['position']['y']
        msg.pose.pose.position.z = bridge['pose']['pose']['position']['z']
        msg.pose.pose.orientation.x = bridge['pose']['pose']['orientation']['x']
        msg.pose.pose.orientation.y = bridge['pose']['pose']['orientation']['y']
        msg.pose.pose.orientation.z = bridge['pose']['pose']['orientation']['z']
        msg.pose.pose.orientation.w = bridge['pose']['pose']['orientation']['w']
        msg.pose.covariance = bridge['pose']['covariance']

        msg.twist.twist.linear.x = bridge['twist']['twist']['linear']['x']
        msg.twist.twist.linear.y = bridge['twist']['twist']['linear']['y']
        msg.twist.twist.linear.z = bridge['twist']['twist']['linear']['z']
        msg.twist.twist.angular.x = bridge['twist']['twist']['angular']['x']
        msg.twist.twist.angular.y = bridge['twist']['twist']['angular']['y']
        msg.twist.twist.angular.z = bridge['twist']['twist']['angular']['z']
        msg.twist.covariance = bridge['twist']['covariance']

        msg.child_frame_id = bridge['child_frame_id']
        self.pub(Odometry, msg)

    def LaserCallback(self, bridge):
        msg = LaserScan()
        msg.angle_min = bridge['angle_min']
        msg.angle_max = bridge['angle_max']
        msg.angle_increment = bridge['angle_increment']
        msg.time_increment = bridge['time_increment']
        msg.scan_time = bridge['scan_time']
        msg.range_min = bridge['range_min']
        msg.range_max = bridge['range_max']
        msg.ranges = bridge['ranges']
        msg.intensities = bridge['intensities']
        self.pub(LaserScan, msg)


client = roslibpy.Ros(host='10.6.0.2', port=9090)
client.run()

rclpy.init()
node = rclpy.create_node('bridge')

float_sub = [Bridge(topic_name,'std_msgs/Float32')  for topic_name in client.get_topics_for_type('std_msgs/Float32')]
int_sub = [Bridge(topic_name,'std_msgs/Int32')  for topic_name in client.get_topics_for_type('std_msgs/Int32')]
odom_sub = [Bridge(topic_name,'nav_msgs/Odometry')  for topic_name in client.get_topics_for_type('nav_msgs/Odometry')]
str_sub = [Bridge(topic_name,'std_msgs/String')  for topic_name in client.get_topics_for_type('std_msgs/String')]
pose_sub = [Bridge(topic_name,'geometry_msgs/PoseStamped')  for topic_name in client.get_topics_for_type('geometry_msgs/PoseStamped')]
laser_sub = [Bridge(topic_name,'sensor_msgs/LaserScan')  for topic_name in client.get_topics_for_type('sensor_msgs/LaserScan')]

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    client.terminate()

