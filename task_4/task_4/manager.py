from PIL import Image
import numpy as np
import cv2
from collections import deque
import math

class GridManager:
    def __init__(self, occupancy_grid, pl, delta_t, sanitization_threshold=10):
        self.occupancy_grid = occupancy_grid
        self.pl = pl  # light power
        self.delta_t = delta_t
        self.sanitization_threshold = sanitization_threshold

    def calculate_energy_at_point(self, x, y, robot_position):
        px, py = robot_position
        distance_squared = (x - px) ** 2 + (y - py) ** 2
        if distance_squared != 0:  # Avoid division by zero
            return (self.pl * self.delta_t) / distance_squared
        return 0
    

    def is_line_of_sight_clear(self, start, end):
        # Bresenham's Line Algorithm for a grid
        x0, y0 = start
        x1, y1 = end

        x0, y0, x1, y1 = map(int, [x0, y0, x1, y1])  # Convert to integers

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            if (x0, y0) == (x1, y1):
                break
            if self.occupancy_grid[y0][x0] == 1:  # Check for an obstacle
                return False
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

        return True


    def update_energy_grid(self, robot_position):
        for y in range(self.occupancy_grid.shape[0]):
            for x in range(self.occupancy_grid.shape[1]):
                if self.occupancy_grid[y, x] == 1:  # Skip obstacle cells
                    continue
                if self.is_line_of_sight_clear(robot_position, (x, y)):
                    if self.occupancy_grid[y, x] < self.sanitization_threshold:  # Update unsanitized, accessible cells
                        additional_energy = self.calculate_energy_at_point(x, y, robot_position)
                        self.occupancy_grid[y, x] += additional_energy
                        if self.occupancy_grid[y, x] > self.sanitization_threshold:
                            self.occupancy_grid[y, x] = self.sanitization_threshold
        return self.occupancy_grid


    def create_grid_image(self, scale_factor=1, robot_position=None):
        # Create an empty image with 3 channels (RGB)
        grid_image = np.zeros((self.occupancy_grid.shape[0], self.occupancy_grid.shape[1], 3), dtype=np.uint8)

        for y in range(self.occupancy_grid.shape[0]):
            for x in range(self.occupancy_grid.shape[1]):
                if robot_position and (x, y) == robot_position:
                    color = (255, 0, 0)  # Blue color for the robot's position
                elif self.occupancy_grid[y, x] == 1.0:  # Obstacle cells - Black
                    color = (0, 0, 0)
                elif self.occupancy_grid[y, x] >= self.sanitization_threshold:  # Sanitized cells - Green
                    color = (0, 255, 0)
                elif self.occupancy_grid[y, x] >= self.sanitization_threshold / 2:  # Partially sanitized - Yellow
                    color = (0, 255, 255)
                else:  # Not sanitized - Red
                    color = (0, 0, 255)
                grid_image[y, x] = color

        # Upscale the image if needed
        if scale_factor != 1:
            grid_image = self.upscale_grid_image(grid_image, scale_factor)

        return grid_image
    
    def create_subgrid_image(self, grid_image, top_left, bottom_right, scale_factor):
        x1, y1 = top_left[0]*scale_factor, top_left[1]*scale_factor
        x2, y2 = bottom_right[0]*scale_factor, bottom_right[1]*scale_factor
        roi = grid_image[y1:y2, x1:x2]
        return roi


    def upscale_grid_image(self, grid_image, scale_factor):
        """
        Upscales the grid image by a given scale factor.
        """
        # Assuming create_grid_image is a method that creates the grid image
        upscaled_image = cv2.resize(grid_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)
        return upscaled_image
    
    def downscale_grid_image(self, grid_image, scale_factor):
        new_width = int(grid_image.shape[1] / scale_factor)
        new_height = int(grid_image.shape[0] / scale_factor)

        # Resize (downscale) the image
        downscaled_image = cv2.resize(grid_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
        return downscaled_image
    
    def display_grid(self, grid_image):
        # Display the image
        cv2.imshow("Occupancy Grid", grid_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def extract_grid_roi(self, image, top_left, bottom_right, scale_factor):
        # Read the image

        self.downscale_grid_image(image, scale_factor)

        # Extract ROI
        x1, y1 = top_left
        x2, y2 = bottom_right
        roi = image[y1:y2, x1:x2]

        image_roi = self.upscale_grid_image(roi, scale_factor)
        
        return image_roi

    def calculate_sanitized_percentage(self):
        total_cells = 0
        sanitized_cells = 0

        for row in self.occupancy_grid:
            for cell in row:
                if cell != 1:  # Assuming 1 represents an obstacle
                    total_cells += 1
                    if cell >= self.sanitization_threshold:
                        sanitized_cells += 1

        if total_cells == 0:
            return 0  # Avoid division by zero if the grid has only obstacles

        sanitized_percentage = (sanitized_cells / total_cells) * 100
        return sanitized_percentage
    


    def is_neighborhood_sanitized(self, position, neighborhood_size=3):
        half_size = neighborhood_size // 2

        # print('-------------', position)    
        for dy in range(-half_size, half_size + 1):
            for dx in range(-half_size, half_size + 1):
                # Skip the robot's current cell
                if dx == 0 and dy == 0:
                    continue

                ny, nx = position[1] + dy, position[0] + dx
                # Check bounds to ensure we don't go outside the grid
                if 0 <= ny < self.occupancy_grid.shape[0] and 0 <= nx < self.occupancy_grid.shape[1]:
                    if self.is_line_of_sight_clear(position, (nx,ny)):
                        cell_value = self.occupancy_grid[ny, nx]
                        # Consider obstacle cells (black cells) as sanitized
                        if cell_value != 1 and cell_value < self.sanitization_threshold:
                            return False
        
        print('Neighborhood OK')
        return True  # Neighborhood is considered sanitized if all non-obstacle cells are sanitized
    

class MapManager:
    def __init__(self, pgm_file_path, original_resolution, desired_resolution, map_origin):
        self.pgm_file_path = pgm_file_path
        self.original_resolution = original_resolution
        self.desired_resolution = desired_resolution
        self.map_origin = map_origin
        self.scaled_grid = self.scale_grid(self.load_pgm(pgm_file_path))

    def load_pgm(self, pgm_file_path):
        img = Image.open(pgm_file_path)
        return np.array(img)


    def scale_grid(self, img_array):
        scale_factor = int(self.desired_resolution / self.original_resolution)
        scaled_height = img_array.shape[0] // scale_factor
        scaled_width = img_array.shape[1] // scale_factor
        scaled_grid = np.zeros((scaled_height, scaled_width))

        for i in range(scaled_height):
            for j in range(scaled_width):
                block = img_array[i*scale_factor:(i+1)*scale_factor, j*scale_factor:(j+1)*scale_factor]
                # If any value in the block is less than 50, classify as occupied
                if np.any(block < 50):
                    scaled_grid[i, j] = 1
                # Otherwise, classify based on the majority of the block's values
                else:
                    values, counts = np.unique(block, return_counts=True)
                    most_common = values[np.argmax(counts)]
                    if most_common > 200:  # Free
                        scaled_grid[i, j] = 0
                    else:  # Unknown
                        scaled_grid[i, j] = -1

        return scaled_grid


    def convert_to_grid_coordinates(self, real_coords):
        """
        Convert real-world coordinates to grid coordinates.
        """
        grid_x = int((real_coords[0] - self.map_origin[0]) / self.desired_resolution)
        # Adjust for y-coordinate inversion
        grid_y = int(math.floor(((self.map_origin[1] - real_coords[1]) / self.desired_resolution) + self.scaled_grid.shape[0]))

        # Ensure coordinates are within grid bounds
        grid_x = max(0, min(grid_x, self.scaled_grid.shape[1] - 1))
        grid_y = max(0, min(grid_y, self.scaled_grid.shape[0] - 1))

        return [grid_x, grid_y]

    def convert_to_pixel_coordinates(self, grid_coords):
        """
        Convert grid coordinates back to real-world (pixel) coordinates.
        """
        pixel_x = (grid_coords[0] * self.desired_resolution) + self.map_origin[0]
        
        y_max_pixel = 5.2 #approx from RViz2 map
        pixel_y = y_max_pixel - (grid_coords[1] * self.desired_resolution)

        # formatting results to 2 decimals

        pixel_x_final = np.round(pixel_x, 2)
        pixel_y_final = np.round(pixel_y, 2)


        return (pixel_x_final, pixel_y_final)


    def extract_subgrid(self, room_corners):
        """
        Extract the sub-grid corresponding to the room.
        room_corners should be a tuple of (top_left, bottom_right) grid coordinates.
        """
        top_left, bottom_right = room_corners

        # Ensure that we extract the grid correctly by taking the higher Y value as the starting point
        # and the lower Y value as the ending point since we are decreasing in Y as we go down.
        y_start = max(top_left[1], bottom_right[1])
        y_end = min(top_left[1], bottom_right[1])

        # Similarly, ensure that X coordinates are in the correct order
        x_start = min(top_left[0], bottom_right[0])
        x_end = max(top_left[0], bottom_right[0])

        # Extract the subgrid using the determined start and end points
        # return self.scaled_grid[y_end:y_start, x_start:x_end]
        return self.scaled_grid[y_end:y_start, x_start:x_end]

    def visualize_grid(self, grid, window_title="Map Visualization"):
        color_map = {
            1: [0, 0, 0],      # Black for occupied cells
            0: [255, 255, 255],# White for free cells
            -1: [127, 127, 127]# Gray for unknown cells
        }

        image = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                image[i, j] = color_map[grid[i, j]]

        cv2.imshow(window_title, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

class RoutePlanner:
    def __init__(self, occupancy_grid, sanitization_threshold, maxlen):
        self.occupancy_grid = occupancy_grid
        self.sanitization_threshold = sanitization_threshold
        self.position_buffer = deque(maxlen = maxlen)


    # YELLOW FIRST, THEN RED

    def find_closest_target(self, current_position):
        min_distance = float('inf')
        target_position = None
        self.position_buffer.append(current_position)

        # Function to check if the cell is a valid target
        def is_valid_target(x, y):
            return (((x, y) not in self.position_buffer) and ((x, y) != current_position) and (self.occupancy_grid[y, x] < self.sanitization_threshold))

        # First, look for the closest half-sanitized (yellow) cells
        for y in range(self.occupancy_grid.shape[0]):
            for x in range(self.occupancy_grid.shape[1]):
                if is_valid_target(x, y) and self.sanitization_threshold / 2 <= self.occupancy_grid[y, x]:
                    distance = np.sqrt((x - current_position[0])**2 + (y - current_position[1])**2)
                    if distance < min_distance:
                        min_distance = distance
                        target_position = (x, y)

        # If no half-sanitized cells found, look for non-sanitized (red) cells
        if target_position is None:
            for y in range(self.occupancy_grid.shape[0]):
                for x in range(self.occupancy_grid.shape[1]):
                    if is_valid_target(x, y):
                        distance = np.sqrt((x - current_position[0])**2 + (y - current_position[1])**2)
                        if distance < min_distance:
                            target_position = (x, y)

        return target_position