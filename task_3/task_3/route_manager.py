import itertools
import random
import yaml
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose,FollowPath
from std_msgs.msg import Bool

class RouteManager(Node):
    """
    Send goals to the navigation2 stack for the specified route. Routes forever.

   Loads the route from yaml.
   Use RViz to record 2D nav goals.
   Echo the input goal on topic /move_base_simple/goal

   Format:

        mode: inorder
        poses:
            - pose:
                  position:
                    x: -5.41667556763
                    y: -3.14395284653
                    z: 0.0
                  orientation:
                    x: 0.0
                    y: 0.0
                    z: 0.785181432231
                    w: 0.619265789851

    """

    # Return an iterator over the goals
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random': lambda goals: (random.choice(goals) for i in itertools.count()),
    }

    def __init__(self):
        super().__init__('route_manager', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.route = []
        self.current_goal = NavigateToPose.Goal()

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()
       	#time.sleep(10)
        
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.follow_path_client.wait_for_server()
       
        route_file_path = os.path.join(get_package_share_directory('task_3'), 'config', 'routes.yaml')
        with open(route_file_path, 'r') as f:
            route_file_contents = f.read()
        route_yaml = yaml.safe_load(route_file_contents)

        self.route_mode = route_yaml['mode']
        if self.route_mode not in RouteManager.route_modes:
            self.get_logger().error(
                "Route mode '%s' unknown, exiting route manager" % (self.route_mode))
            return

        poses = route_yaml['poses']
        if not poses:
            self.get_logger().info("Route manager initialized no goals, unable to route")

        self.goals = RouteManager.route_modes[self.route_mode](poses)
        self.get_logger().info(
            "Route manager initialized with %s goals in %s mode" % (len(poses), self.route_mode))
        self.allgoal = len(poses)
        self.countgoal = 0


        # subscriber to receive info about localization
        self.localization_subscriber = self.create_subscription(
                    Bool,
                    'LocalizationCompleted',
                    self.localization_callback,
                    10
                    )

    def to_move_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position = Point(**pose['pose']['position'])
        goal.pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def route_forever(self):
        try:
            self.countgoal += 1
            if (self.countgoal > self.allgoal):
                raise StopIteration
            self.get_logger().info("Route mode is '%s', getting next goal" % (self.route_mode))
            current_goal = self.to_move_goal(next(self.goals))
            self.get_logger().info("Sending target goal: %s" % (current_goal))
            self.client.wait_for_server()
            #time.sleep(10)
            self._send_goal_future = self.client.send_goal_async(
                current_goal)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        except StopIteration:
            self.get_logger().info("No goals, stopping route manager.")
            raise SystemExit

    def get_result_callback(self, future: NavigateToPose.Result):
        result = future.result().result
        # Expecting empty result (std_msgs::Empty) for NavigateToPose
        self.get_logger().info('Result: {}'.format(result.result))
        self.route_forever()

    def localization_callback(self, msg):
        
        # Call the function
        self.route_forever()

def main():
    rclpy.init()
    try:
        route_manager = RouteManager()
        rclpy.spin(route_manager)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')

    except KeyboardInterrupt:
        pass
    except BaseException:
        print('Exception in route_manager:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()