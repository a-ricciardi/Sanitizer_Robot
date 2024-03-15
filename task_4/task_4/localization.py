import rclpy
import sys
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
import tf_transformations
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Empty
from std_msgs.msg import Bool

# Seed for random variables
# np.random.seed(0)
# Initial estimated positions
x = 0.0
y = 0.0

class Localization(Node):

    def __init__(self):
        super().__init__('Localization')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            1
        )
        self.get_logger().info("Initialized")

        # Amcl initialization variables
        self.amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.localization_ended = False
        self.estim_amcl_pose = PoseStamped()
        self.estim_amcl_orientation = PoseStamped()
        self.estim_amcl_cov = np.empty((6, 6))
        self.cov_des = np.ones(6) * 0.5
        self.cov_des[5] = 0.1

        self.amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amclPoseCallback,
            self.amcl_pose_qos
        )
        self.get_logger().info("Subscription to amcl_pose topic done")

        # Setting timer
        self.call_timer = self.create_timer(20, self.timer_change_goal)
        self.counter = 0

        # Initialization action servers
        self.goal = NavigateToPose.Goal()
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()

        # Initialization services for cleaning the visualization on Rviz
        self.client_costmap = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        while not self.client_costmap.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service client_costmap not available')
        self.clear_costmap = ClearEntireCostmap.Request()

        # Create a publisher on a topic called LocalizationCompleted
        self.localization_publisher = self.create_publisher(
            Bool,
            'LocalizationCompleted',
            10
        )

    def publish_initial_pose(self, x_, y_):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x_
        msg.pose.pose.position.y = y_
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q_x
        msg.pose.pose.orientation.y = q_y
        msg.pose.pose.orientation.z = q_z
        msg.pose.pose.orientation.w = q_w
        # Set Covariance
        cov = np.zeros(36, dtype=np.float64)
        cov[0] = 200
        cov[7] = 200
        cov[35] = 1
        msg.pose.covariance = cov
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing initial estimated position")

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        self.estim_amcl_pose = msg.pose.pose.position
        self.estim_amcl_orientation = msg.pose.pose.orientation
        self.estim_amcl_cov = np.reshape(msg.pose.covariance, (6, 6))
        self.get_logger().info('Check on Eigenvalues: "%s"' % np.linalg.eigvals(self.estim_amcl_cov))
        print(np.linalg.eigvals(self.estim_amcl_cov))
        if np.all(np.linalg.eigvals(self.estim_amcl_cov) <= self.cov_des):
            print("localization_DONE_TRUE")
            self.localization_ended = True
            self.localization_done()

    def localization_done(self):
        msg = Bool()
        msg.data = True
        self.get_logger().info("Covariance matrix reached an acceptable value")
        self.destroy_timer(self.call_timer)
        self.future_clear_costmap = self.client_costmap.call_async(self.clear_costmap)

        self.localization_publisher.publish(msg)
        raise SystemExit

    def timer_change_goal(self):
        self.counter += 1
        self.future_clear_costmap = self.client_costmap.call_async(self.clear_costmap)

        if self.counter >= 5:
            self.get_logger().info("Reinitialize global localization'")
            self.publish_initial_pose(0.0, 0.0)  # Setting the initial pose to the origin
            self.counter = 0
        else:
            # Implement wall following behavior
            q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, -np.pi/2)

            self.goal.pose.header.stamp = self.get_clock().now().to_msg()
            self.goal.pose.header.frame_id = "map"
            self.goal.pose.pose.position = self.estim_amcl_pose
            self.goal.pose.pose.orientation = Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

            self._send_goal_future = self.client.send_goal_async(self.goal)
            self.get_logger().info("New goal sent: %s" % (self.goal))
        
            # Don't assume localization is done here, wait for action to complete
            self._send_goal_future.add_done_callback(self.on_goal_completed)
    
    def on_goal_completed(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("Navigation goal accepted, waiting for result")
            goal_handle.get_result_async().add_done_callback(self.on_goal_result)
        else:
            self.get_logger().warning("Navigation goal rejected")
    
    def on_goal_result(self, future):
        result = future.result().result

        try:
            # Assuming that if the result is received without an exception, navigation is successful
            result_content = str(result)
            self.get_logger().info(f"Navigation result received")

            # Call the _amclPoseCallback method to check the covariance matrix eigenvalues
            #self._amclPoseCallback(PoseWithCovarianceStamped(pose=PoseStamped(pose=result.goal.pose.pose)))

            # Check the covariance matrix eigenvalues to ensure localization is accurate
            if np.all(np.linalg.eigvals(self.estim_amcl_cov) <= self.cov_des):
                self.localization_ended = True
                self.localization_done()

        except Exception:
            self.get_logger().warning(f"Navigation failed")

def main():
    rclpy.init()
    try:
        publisher = Localization()
        publisher.publish_initial_pose(x, y)
        rclpy.spin(publisher)
    except SystemExit:  # <--- process the exception
        rclpy.logging.get_logger("Quitting").info('Localization Done')
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in route_manager:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()