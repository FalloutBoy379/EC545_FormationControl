# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String
# #from apriltag_msgs.msg import AprilTagDetectionArray
# from tf2_msgs.msg import TFMessage
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovariance

# previous_poses = {}

# nodeName = "tagToOdo"

# class tagToOdo(Node):
# 	def __init__(self):
# 		super().__init__('tag_to_odo')
# 		self.subscription = self.create_subscription(TFMessage,'/tf', self.listener_callback, 3)
# 		self.subscription
# 		self.bot1 = self.create_publisher(Odometry, 'robot1/odometry', 3)
# 		self.bot2 = self.create_publisher(Odometry, 'robot2/odometry', 3)
# 		self.bot3 = self.create_publisher(Odometry, 'robot3/odometry', 3)
# 		self.bot4 = self.create_publisher(Odometry, 'robot4/odometry', 3)
# 		self.bot5 = self.create_publisher(Odometry, 'robot5/odometry', 3)
# 		self.bot6 = self.create_publisher(Odometry, 'robot6/odometry', 3)
            
#     def publish_call(self, tag_id, odo):
#         #crafts and publishes odometry message for tag_id
#         odo.header.stamp = rclpy.Time.now()
#         if int(tag_id) == 94:
#             self.robot1.publish(odo)
#         elif int(tag_id) == 104:
#             self.robot2.publish(odo)
#         elif int(tag_id) == 109:
#             self.robot3.publish(odo)
#         elif int(tag_id) == 127:
#             self.robot4.publish(odo)
#         elif int(tag_id) == 78:
#             self.robot5.publish(odo)
#         elif int(tag_id) == 86:
#             self.robot6.publish(odo)
#         else:
#             pass

#         #do_publisher.publish(odo)
#         #grabs detection array, and publishes odom per tag
    
#     def listener_callback(self, data):
#         for detection in data.transforms:
#             #print()
#             tag_id = detection.child_frame_id.split(':')
#             tag_id = tag_id[1]
#             print(tag_id)

#             pose_w_cov = detection.transform
#             if tag_id in previous_poses:
#                 previous_pose = previous_poses[tag_id]
#                 odo = toOdometry(previous_pose, pose_w_cov)
#                 self.publish_call(tag_id, odo)
#             previous_poses[tag_id] = pose_w_cov
#             self.get_logger().info('I heard: "%s"' % tag_id)

# #find differences in poses to get twist
# def toOdometry(previous, current):
#     odo = Odometry()

#     #linear velocity
#     #odo.twist.twist.linear.x = current.position.x - previous.position.x
#     #odo.twist.twist.linear.y = current.position.y - previous.position.y
#     #odo.twist.twist.linear.z = current.position.z - previous.position.z
#     #odo.twist.twist.linear.z = 0 #force 0
#     #odo.twist.covariance = 

# #	#angular velocity
# #	odo.twist.twist.angular.x= current.orientation.x - previous.orientation.x
# #	odo.twist.twist.angular.y= current.orientation.y - previous.orientation.y
# #	#odo.twist.twist.angular.z= current.orientation.z - previous.orientation.z
# #	odo.twist.twist.angular.z = 0 #force 0

#     #pose
#     odo.pose.pose = current
#     return odo



# def main(args=None):
# 	rclpy.init(args=args)
# 	tag_to_odo = tagToOdo()
# 	rclpy.spin(tag_to_odo)

# 	# Destroy the node explicitly
# 	# (optional - otherwise it will be done automatically
# 	# when the garbage collector destroys the node object)
# 	# tag_to_odo.destroy_node()
# 	rclpy.shutdown()

# if __name__ == '__main__':
# 	main()
