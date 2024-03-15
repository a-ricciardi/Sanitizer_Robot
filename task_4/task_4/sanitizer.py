import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from task_4.manager import *
from rclpy.qos import *
import cv2
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from collections import deque
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from time import *
from rclpy.action import ActionClient
from rclpy.task import Future


class SanitizerNode(Node):
    def __init__(self,
                 mapManager: MapManager,
                 gridManager: GridManager,
                 routePlanner: RoutePlanner,
                 neighborhood_size: int,
                 scale_factor: int,
                 sanitization_goal: int,
                 sanitization_threshold: int,
                 initial_sanitization_pos,
                 grid_top_left,
                 grid_bottom_right
                 ):
        
        super().__init__('sanitizer_node')

        self.map_manager = mapManager
        self.grid_manager = gridManager
        self.route_planner = routePlanner
        self.neighborhood_size = neighborhood_size
        self.scale_factor = scale_factor
        self.sanitization_goal = sanitization_goal
        self.sanitization_threshold = sanitization_threshold
        self.initial_sanitization_pos = initial_sanitization_pos
        self.grid_top_left = grid_top_left
        self.grid_bottom_right = grid_bottom_right

        self.amcl_robot_position = PoseWithCovarianceStamped()
        self.amcl_robot_orientation = PoseWithCovarianceStamped()

        self.grid_robot_position = None
        self.goal_reached_future = None

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()

        self.amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
    
        # subscriber to read robot position
        self.amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amclPoseCallback,
            self.amcl_pose_qos
            )
        
        self.loc_subscriber = self.create_subscription(
            Bool,
            'LocalizationCompleted',
            self.localization_callback,
            10
            )
        
        
    def localization_callback(self, msg):
        self.move_to_starting_position(self.initial_sanitization_pos)
         
    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        self.amcl_robot_position = msg.pose.pose.position
        self.amcl_robot_orientation = msg.pose.pose.orientation

        # conversion to grid cordinates
        self.grid_robot_position =self.map_manager.convert_to_grid_coordinates(
            (self.amcl_robot_position.x,
            self.amcl_robot_position.y)
        )

    def convert_to_gazebo_coords(self, py_coords):
        gazebo_coords = self.map_manager.convert_to_pixel_coordinates(py_coords)
        return gazebo_coords
    
    def to_move_goal(self, goal_position):
        # Set the goal target pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position = Point(x=goal_position[0], y=goal_position[1], z=0.0)
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.70, w=0.70)
        goal_msg.pose.header.frame_id = "map"
        return goal_msg
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future: NavigateToPose.Result):
        result = future.result().result
        # Expecting empty result (std_msgs::Empty) for NavigateToPose
        self.get_logger().info('Result: {}'.format(result.result))
        self.sanitize_room()

    def move_to_starting_position(self, starting_position_gazebo):
        goal_msg = self.to_move_goal(starting_position_gazebo)
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def sanitize_room(self):
        robot_position = self.grid_robot_position

        print('current robot position = ', robot_position)
        
        try:

            neigborhood_sanitized = False
            while True:
                neigborhood_sanitized = self.grid_manager.is_neighborhood_sanitized(robot_position, self.neighborhood_size)
                if neigborhood_sanitized:
                    break
                self.grid_manager.update_energy_grid(robot_position)

            print('neighborood sanitized: ', neigborhood_sanitized)
            sanitized_percentage = self.grid_manager.calculate_sanitized_percentage()

            if sanitized_percentage >= self.sanitization_goal:
                raise StopIteration

            # opencv visualization of sanitization updates on occupancy grid
            grid_image = self.grid_manager.create_grid_image(self.scale_factor, robot_position)
            subgrid_image = self.grid_manager.create_subgrid_image(grid_image, self.grid_top_left, self.grid_bottom_right, self.scale_factor)

            # entire house and partial house visualization
            cv2.imshow("Occupancy Grid", grid_image) # comment line if needed
            # cv2.imshow("Room Occupancy Grid", subgrid_image) # comment line if needed
            cv2.waitKey(1)

            print('robot posizion in function: ', robot_position)
            print('robot position from map: ', self.grid_robot_position)

            next_position = self.route_planner.find_closest_target(robot_position)
            if next_position is None or next_position == robot_position:
                raise StopIteration

            robot_position = next_position
            print('next grid robot position:', robot_position)

            # Convert and publish the new robot position to ROS
            next_goal_gazebo = self.convert_to_gazebo_coords(robot_position)
            print('next gazebo robot position:', next_goal_gazebo)

            goal_msg = self.to_move_goal(next_goal_gazebo)
            self.client.wait_for_server()
            self._send_goal_future = self.client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        except StopIteration:
            self.get_logger().info("Sanitization completed or no goals found, stopping sanitization process.")
            raise SystemExit


def main():

    ######################
    # # INITIALIZATION # #
    ######################

    # sanitization parameters
    sanitization_threshold = 0.01 # has to be < then obstacle value in occupancy grid (1)
    pl = 1e-3  # light power
    delta_t = 10  # time delta
    neighborhood_size = 5
    sanitization_goal = 98 # percentage of map to sanitize
    scale_factor = 20 # to print map visualization with cv2
    position_buffer = 50000 # scale factor for image visualization

    # initial sanitization position in gazebo coordinates
    initial_sanitization_pos = (1, 2)


    # map from rViz2 and Gazebo
    # path_to_map =  '/media/psf/Home/Desktop/FEDE/UNIBO/Magistrale/AMR/AMR_Project/src/task_1/config/map_2.pgm'
    path_to_map = '/home/agatino/Project_AMR/src/task_4/config/map.pgm'
    # path_to_map = '/Users/federicofabbri/Desktop/FEDE/UNIBO/Magistrale/AMR/AMR_Project/src/task_1/config/map_2.pgm'

    # map parameters
    origin = (-7.48, -7.97) # found in .yaml map file
    original_resolution = 0.05
    scaled_resolution = 0.2



    # Manager initialization
    map_manager = MapManager(path_to_map, original_resolution, scaled_resolution, origin)
    grid_manager = GridManager(map_manager.scaled_grid, pl, delta_t, sanitization_threshold)
    route_planner = RoutePlanner(map_manager.scaled_grid, sanitization_threshold, position_buffer)


    # initialization to visualize sub_region of map during sanification, gazebo coordinates
    room_top_left_gazebo = (-0.2, 5.2)
    room_bottom_right_gazebo = (-7.3, 0)

    room_top_left_grid = map_manager.convert_to_grid_coordinates(room_top_left_gazebo)
    room_bottom_right_grid = map_manager.convert_to_grid_coordinates(room_bottom_right_gazebo)
    


    rclpy.init()
    sanitizer = SanitizerNode(
        map_manager,
        grid_manager,
        route_planner,
        neighborhood_size,
        scale_factor, sanitization_goal,
        sanitization_threshold,
        initial_sanitization_pos,
        room_top_left_grid,
        room_bottom_right_grid
        )
    
    rclpy.spin(sanitizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()