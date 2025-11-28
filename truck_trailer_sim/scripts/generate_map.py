import numpy as np
from PIL import Image
import yaml
import os

def generate_t_junction(
    filename_prefix="t_junction",
    map_width_m=50.0,
    map_height_m=50.0,
    resolution=0.05,
    road_width_horiz_m=8.0,
    road_width_vert_m=8.0,
    center_offset_x=0.0, # 0.0 is center. -0.5 is left edge, 0.5 is right edge
    center_offset_y=0.0  # 0.0 is center. -0.5 is bottom edge, 0.5 is top edge
):
    """
    Generates a T-junction occupancy grid map (PGM) and configuration (YAML).
    
    Args:
        filename_prefix: Name for output files (e.g., 'map' -> 'map.pgm', 'map.yaml')
        map_width_m: Physical width of the map in meters
        map_height_m: Physical height of the map in meters
        resolution: Map resolution in meters per pixel
        road_width_horiz_m: Width of the horizontal road in meters
        road_width_vert_m: Width of the vertical road in meters
        center_offset_x: Shift vertical road left/right (ratio of map width, -0.5 to 0.5)
        center_offset_y: Shift horizontal road up/down (ratio of map height, -0.5 to 0.5)
    """
    
    # 1. Calculate dimensions in pixels
    width_px = int(np.ceil(map_width_m / resolution))
    height_px = int(np.ceil(map_height_m / resolution))
    
    print(f"Generating map: {width_px}x{height_px} pixels ({map_width_m}x{map_height_m}m)")

    # 2. Initialize Grid (0 = Black/Occupied, 255 = White/Free)
    # in ROS map_server: 
    #   Pixel 0 (Black)   => Occupancy 100 (Occupied)
    #   Pixel 255 (White) => Occupancy 0   (Free)
    #   Pixel 205 (Gray)  => Unknown
    grid = np.zeros((height_px, width_px), dtype=np.uint8)

    # 3. Calculate Pixel coordinates for roads
    
    # Center points in pixels (considering offsets)
    center_x_px = int((width_px / 2) + (center_offset_x * width_px))
    center_y_px = int((height_px / 2) - (center_offset_y * height_px)) # Y is inverted in image coords usually, but we handle it simply here
    
    # Road half-widths in pixels
    rw_h_half_px = int((road_width_horiz_m / resolution) / 2)
    rw_v_half_px = int((road_width_vert_m / resolution) / 2)

    # 4. Draw Roads (Set pixels to 255/White)
    
    # Horizontal Road (Full width)
    # Y bounds: center_y +/- half width
    y_start = max(0, center_y_px - rw_h_half_px)
    y_end = min(height_px, center_y_px + rw_h_half_px)
    grid[y_start:y_end, :] = 255

    # Vertical Road (From bottom/top to horizontal road center)
    # For a T-junction, usually the vertical leg comes from the bottom and stops at the center
    # X bounds
    x_start = max(0, center_x_px - rw_v_half_px)
    x_end = min(width_px, center_x_px + rw_v_half_px)
    
    # Y bounds (From bottom of image up to the top of the horizontal road)
    # If you want a 4-way intersection, change 'center_y_px' to 0 or height_px
    # Here we draw from bottom (height_px) up to the top edge of the horizontal road
    # Note: numpy image origin (0,0) is Top-Left. 
    # So "Bottom" of map is index `height_px`.
    
    # Let's make the "Stem" of the T come from the "South" (Bottom of image)
    # It should connect to the horizontal road.
    # We define the vertical road valid range from row `y_end` (bottom of horiz road) down to `height_px`
    # Wait, in numpy 0 is top. 
    # Horizontal road is at y_start to y_end.
    # To make a "T" where the stem is at the bottom:
    # We draw from y_end (bottom of horiz road) to height_px (bottom of image).
    
    grid[center_y_px:height_px, x_start:x_end] = 255

    # Optional: Fill the intersection cleanly (already covered by overlap, but ensures connectivity)
    grid[y_start:y_end, x_start:x_end] = 255

    # 5. Save Image
    image_filename = f"{filename_prefix}.pgm"
    img = Image.fromarray(grid)
    img.save(image_filename)
    print(f"Saved image: {image_filename}")

    # 6. Create YAML file
    # The origin is [x, y, yaw]. 
    # Usually we want (0,0) to be the exact center of the image map in world coordinates.
    # The YAML origin defines the world position of the BOTTOM-LEFT pixel of the image.
    origin_x = -(map_width_m / 2)
    origin_y = -(map_height_m / 2)
    
    yaml_data = {
        'image': image_filename,
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'mode': 'trinary' # Standard for ROS 2
    }
    
    yaml_filename = f"{filename_prefix}.yaml"
    with open(yaml_filename, 'w') as f:
        yaml.dump(yaml_data, f, sort_keys=False)
    print(f"Saved config: {yaml_filename}")

if __name__ == "__main__":
    # --- CONFIGURATION ---
    generate_t_junction(
        filename_prefix="t_junction",
        map_width_m=6.0,       # 6 meters wide
        map_height_m=3.0,      # 3 meters tall
        resolution=0.05,        # 5cm per pixel (high res for parking)
        road_width_horiz_m=1.0, # 1m wide road
        road_width_vert_m=1.0,  # 1m wide road
        center_offset_x=0.0,    # Centered left-right
        center_offset_y=0.3     # Centered up-down
    )