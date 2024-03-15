from manager import *
from collections import deque
import cv2

# Initialization and setup...
sanitization_threshold = 0.01 # has to be < then obstacle value in occupancy grid (1)
pl = 1e-3  # light power
delta_t = 10  # time delta
neighborhood_size = 10
sanitization_goal = 98 # percentage of map to sanitize
scale_factor = 20 # to print map visualization with cv2
position_buffer = 50 # scale factor for image visualization
sanitized_percentage = 0 # initial value

# initial sanitization position in gazebo coordinates
initial_sanitization_pos = (-6.7, -2.8)


# map from rViz2 and Gazebo
# path_to_map =  '/media/psf/Home/Desktop/FEDE/UNIBO/Magistrale/AMR/AMR_Project/src/task_1/config/map_2.pgm'
# path_to_map = '/Users/federicofabbri/Desktop/FEDE/UNIBO/Magistrale/AMR/AMR_Project/src/task_1/config/map_2.pgm'
path_to_map = '/home/agatino/Project_AMR/src/task_4/config/map.pgm'

# map parameters
origin = (-7.48, -7.97) # found in .yaml map file
original_resolution = 0.05
scaled_resolution = 0.2

# Manager initialization
map_manager = MapManager(path_to_map, original_resolution, scaled_resolution, origin)
grid_manager = GridManager(map_manager.scaled_grid, pl, delta_t, sanitization_threshold)
route_planner = RoutePlanner(map_manager.scaled_grid, sanitization_threshold, position_buffer)


# initialization to visualize sub_region of map during sanification, gazebo coordinates
room_top_left_gazebo = (-7.46, 5.2)
room_bottom_right_gazebo = (-4.94, -4.24)

room_top_left_grid = map_manager.convert_to_grid_coordinates(room_top_left_gazebo)
room_bottom_right_grid = map_manager.convert_to_grid_coordinates(room_bottom_right_gazebo)


# while loop for sanitization simulation 

robot_position = map_manager.convert_to_grid_coordinates(initial_sanitization_pos)


while sanitized_percentage < sanitization_goal:
    grid_manager.update_energy_grid(robot_position)
    neighborhood_sanitized = grid_manager.is_neighborhood_sanitized(robot_position, neighborhood_size)
    sanitized_percentage = grid_manager.calculate_sanitized_percentage()



    if neighborhood_sanitized:
        next_position = route_planner.find_closest_target(robot_position)
        if next_position is None or next_position == robot_position:
            break

        robot_position = next_position
        print(f'New robot position: {robot_position}')

    grid_image = grid_manager.create_grid_image(scale_factor, robot_position)
    subgrid_image = grid_manager.create_subgrid_image(grid_image, room_top_left_grid, room_bottom_right_grid, scale_factor)


    # entire house and partial house visualization
    cv2.imshow("Occupancy Grid", grid_image) # comment line if needed
    # cv2.imshow("Room", subgrid_image) # comment line if needed

    cv2.waitKey(1)




cv2.waitKey(0)
cv2.destroyAllWindows()