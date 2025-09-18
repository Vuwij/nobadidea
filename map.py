import os
from PIL import Image

import pybullet as pb
from collections import defaultdict
from tile import load_tiles, TileType

storefront_dir = "storefronts"
atlas_dir = "storefronts_atlas"
os.makedirs(atlas_dir, exist_ok=True)

storefront_files = sorted(
    [os.path.join(storefront_dir, f) for f in os.listdir(storefront_dir)
     if f.lower().endswith((".png", ".jpg", ".jpeg"))]
)


def make_4x4_atlas_texture(input_path, output_path, shrink_factor=4, filler_color=(30, 30, 30)):
    """
    Create a square texture with 4x4 grid of the storefront image.
    The image will be repeated 4 times horizontally and 4 times vertically.
    Each image is flipped left and right (mirrored horizontally).
    Specific tiles can be rotated using rotation_map.

    rotation_map format: {(row, col): degrees}
    Example: {(3, 1): 90, (2, 2): 180, (0, 3): 270}
    """
    img = Image.open(input_path).convert("RGB")
    w, h = img.size

    # Calculate the size for each tile (shrunk)
    tile_width = w // shrink_factor

    # Create a square canvas (4x4 grid)
    canvas = Image.new("RGB", (tile_width * 4, tile_width * 4), filler_color)

    # Default rotation map if none provided
    rotation_map = {(2, 0): 90, (2, 1): 180, (3, 0): 0, (3, 2): 0}
    solid_color_set = {(0, 0), (0, 1), (0, 2), (0, 3),
                       (1, 0), (1, 1), (1, 2), (1, 3),
                       (2, 2), (2, 3),
                       (3, 1), (3, 3)}

    # Resize the original image to tile size and flip horizontally
    base_tile = img.resize((tile_width, tile_width), Image.LANCZOS)
    base_tile = base_tile.transpose(Image.FLIP_LEFT_RIGHT)  # Flip left-right

    # Paste the tile in a 4x4 grid pattern with specified rotations
    for row in range(4):
        for col in range(4):
            x = col * tile_width
            y = row * tile_width

            if (row, col) in solid_color_set:
                continue

            # Check if this tile needs rotation
            if (row, col) in rotation_map:
                rotation_degrees = rotation_map[(row, col)]
                rotated_tile = base_tile.rotate(rotation_degrees, expand=False)
                canvas.paste(rotated_tile, (x, y))
            else:
                canvas.paste(base_tile, (x, y))

    canvas.save(output_path)


# Build 4x4 atlas textures only if missing
atlas_files = []
for file in storefront_files:
    base, _ = os.path.splitext(os.path.basename(file))
    atlas_file = os.path.join(atlas_dir, base + "_4x4_atlas.png")
    if not os.path.exists(atlas_file):
        make_4x4_atlas_texture(file, atlas_file)
    atlas_files.append(atlas_file)

print(f"Created {len(atlas_files)} 4x4 atlas textures in {atlas_dir}")

# Replace storefront_files with atlas_files for usage later
storefront_files = atlas_files

# Keep track of which texture to assign next
building_texture_index = 0


def create_building(pos, height, size=1):
    """
    Create a building at pos (x, y), with given height (number of floors).
    Each floor gets its own texture, cycling through storefront_files.
    """
    global building_texture_index
    floor_ids = []

    for floor in range(height):
        z = (floor + 0.5) * size  # center of this floor

        # Create a visual + collision shape for this floor
        visual_shape_id = pb.createVisualShape(
            shapeType=pb.GEOM_BOX,
            halfExtents=[size / 2, size / 2, size / 2]
        )
        collision_shape_id = pb.createCollisionShape(
            shapeType=pb.GEOM_BOX,
            halfExtents=[size / 2, size / 2, size / 2]
        )
        body_id = pb.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[pos[0], pos[1], z]
        )


        # Assign texture for this floor
        if storefront_files:
            texture_file = storefront_files[building_texture_index % len(storefront_files)]
            tex_id = pb.loadTexture(texture_file)
            pb.changeVisualShape(body_id, -1, textureUniqueId=tex_id)

            building_texture_index += 1

        floor_ids.append(body_id)

    return floor_ids


def create_road(pos, tile_size=1):
    """Create a road tile at pos (x, y) with reduced size (road_scale)."""
    half_size = tile_size * 0.8 / 2
    visual_shape_id = pb.createVisualShape(
        shapeType=pb.GEOM_BOX,
        halfExtents=[half_size, half_size, 0.01],
        rgbaColor=[0.5, 0.5, 0.5, 1]
    )
    collision_shape_id = pb.createCollisionShape(
        shapeType=pb.GEOM_BOX,
        halfExtents=[half_size, half_size, 0.01]
    )
    body_id = pb.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[pos[0], pos[1], 0.01]
    )
    return body_id

def draw_map():
    map_file = "map.txt"
    objects, map_width, map_height = load_tiles(map_file)
    tile_size = 1
    # Ground size matches map exactly
    ground_width = map_width * tile_size
    ground_height = map_height * tile_size
    ground_visual = pb.createVisualShape(pb.GEOM_BOX,
                                         halfExtents=[ground_width / 2, ground_height / 2, 0.05],
                                         rgbaColor=[0.5, 0.8, 0.5, 1])
    ground_collision = pb.createCollisionShape(pb.GEOM_BOX,
                                               halfExtents=[ground_width / 2, ground_height / 2, 0.05])
    pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=ground_collision, baseVisualShapeIndex=ground_visual,
                       basePosition=[ground_width / 2 - tile_size / 2, ground_height / 2 - tile_size / 2, -0.05])
    # Create map objects
    # Define allowed directions per road style
    tile_directions = {
        "R": [(0, 1), (0, -1), (-1, 0), (1, 0)],  # all four directions
        "|": [(0, 1), (0, -1)],  # up and down
        "/": [(0, 1), (-1, 0)],  # up and left
    }
    # Build edges dictionary
    edges = defaultdict(list)
    road_positions = {tile.pos for tile in objects if tile.type == TileType.ROAD}
    for tile in objects:
        if tile.type == TileType.ROAD:
            x, y = tile.pos
            directions = tile_directions.get(tile.road_style, [])  # get directions based on style
            for dx, dy in directions:
                neighbor = (x + dx, y + dy)
                if neighbor in road_positions:
                    edges[tile.pos].append(neighbor)
    # Compute edge midpoints
    edge_midpoints = set()
    for from_tile, neighbors in edges.items():
        x1, y1 = from_tile
        for nx, ny in neighbors:
            mx = (x1 + nx) / 2
            my = (y1 + ny) / 2
            edge_midpoints.add((mx, my))
    # Create PyBullet objects
    for tile in objects:
        if tile.type == TileType.ROAD:
            create_road(tile.pos, tile_size=tile_size)
        elif tile.type == TileType.BUILDING:
            create_building(tile.pos, tile.height, size=tile_size)
    # Create road segments for midpoints
    for edge_midpoint in edge_midpoints:
        create_road(edge_midpoint, tile_size=1)
    return objects, ground_height, ground_width, map_height, map_width
