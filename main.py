import math
from collections import deque

import pybullet as pb
import pybullet_data
import time
import threading
import tkinter as tk

from map import draw_map
from robot import Robot
from store import Store
from tile import TileType


# Example store locations (replace with your real store coords)
STORE_LIST = [
    Store("다이소 (Daiso)", (1, 3, 0.5), floor=1, direction="LEFT"),
    Store("맘스터치 (Mom's Touch)", (2, 2, 0.1), floor=2, direction="RIGHT"),
    Store("메가커피 (Mega Coffee)", (2, 2, 0.1), floor=1, direction="RIGHT"),
    Store("GS25 (GS25)", (2, 2, 0.1), floor=0, direction="RIGHT"),
    Store("CGV (CGV)", (1, 3, 0.1), floor=0, direction="LEFT"),
    Store("삼성 (Samsung)", (0, 1, 0.1), floor=1, direction="DOWN"),
    Store("올리브영 (Olive Young)", (0, 1, 0.1), floor=0, direction="DOWN"),
    Store("뚜레쥬르 (Tous les Jours)", (0, 1, 0.1), floor=2, direction="DOWN"),
    Store("조선대학교 (Chosun University)", (2, 0, 0.1), floor=0, direction="RIGHT"),
]

tile_directions = {
    "R": [(0, 1), (0, -1), (-1, 0), (1, 0)],  # all four
    "|": [(0, 1), (0, -1)],                   # vertical only
    "/": [(0, 1), (-1, 0)],                   # up and left (example)
}

def can_move(tile, direction):
    """Check if a tile allows movement in given direction"""
    if tile.type != TileType.ROAD:
        return False
    if tile.road_style is None:
        return tile_directions.get("R")
    return direction in tile_directions.get(tile.road_style, [(0, 1), (0, -1), (-1, 0), (1, 0)])


def find_path(start_pos, goal_pos, objects):
    """BFS pathfinding with road_style constraints"""
    # Convert objects to lookup dict
    tile_map = {tile.pos: tile for tile in objects}

    # BFS
    queue = deque([(start_pos, [start_pos])])
    visited = set([start_pos])

    while queue:
        current, path = queue.popleft()
        if current == goal_pos:
            return path

        if current not in tile_map:
            continue
        curr_tile = tile_map[current]

        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt not in tile_map or nxt in visited:
                continue

            nxt_tile = tile_map[nxt]

            # Goal tile is always valid (even if it's not a ROAD)
            if nxt == goal_pos:
                queue.append((nxt, path+[nxt]))
                visited.add(nxt)
                continue

            # Both tiles must allow travel in this direction
            if can_move(curr_tile, (dx, dy)) and can_move(nxt_tile, (-dx, -dy)):
                queue.append((nxt, path+[nxt]))
                visited.add(nxt)

    return None

def run_pybullet():
    global objects
    # Connect
    pb.connect(pb.GUI, options="--width=1480 --height=900")
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

    # Load map
    objects, ground_height, ground_width, map_height, map_width = draw_map()

    # Adjust camera to fit the map
    pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
    pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
    pb.resetDebugVisualizerCamera(
        cameraDistance=max(map_width, map_height) * 0.7,
        cameraYaw=0,
        cameraPitch=-45,
        cameraTargetPosition=[ground_width / 2 - 1 / 2,
                              ground_height / 2 - 1 / 2, 0]
    )

    # Spawn robot
    start_tile = next((tile for tile in objects if tile.type == TileType.START), None)
    if start_tile is not None:
        start_pos = (start_tile.pos[0], start_tile.pos[1], 0.1)
    else:
        start_pos = (0, 0, 0.1)
    robot = Robot(start_pos=start_pos)

    # Expose robot so Tkinter callbacks can access it
    global shared_robot
    shared_robot = robot

    try:
        while True:
            pb.stepSimulation()
            time.sleep(1. / 240.)
    except KeyboardInterrupt:
        pass
    finally:
        pb.disconnect()


def goto_store(store: Store):
    global objects

    """Move robot along the BFS path using spin/move_forward"""
    if shared_robot is None:
        return

    # Get start and goal grid
    start_pos, _ = pb.getBasePositionAndOrientation(shared_robot.body_id)
    start_grid = (round(start_pos[0]), round(start_pos[1]))
    goal_grid = (round(store.location[0]), round(store.location[1]))

    path = find_path(start_grid, goal_grid, objects)
    if not path:
        print(f"No path found to {store.name}")
        return

    print(f"Path to {store.name}: {path}")

    for i in range(1, len(path)):
        current = path[i-1]
        next_tile = path[i]

        # Compute required yaw to face next tile
        dx = next_tile[0] - current[0]
        dy = next_tile[1] - current[1]
        target_yaw = math.atan2(dy, dx)

        # Rotate until facing the direction
        while True:
            pos, orn = pb.getBasePositionAndOrientation(shared_robot.body_id)
            euler = pb.getEulerFromQuaternion(orn)
            yaw = euler[2]
            angle_diff = (target_yaw - yaw + math.pi) % (2*math.pi) - math.pi  # normalize to [-pi, pi]

            if abs(angle_diff) < 0.02:  # threshold
                break
            spin_speed = 3 if angle_diff > 0 else -3
            shared_robot.spin(speed=spin_speed)
            pb.stepSimulation()
            time.sleep(1./240.)

        # Move forward until reaching next tile
        while True:
            pos, _ = pb.getBasePositionAndOrientation(shared_robot.body_id)
            if abs(pos[0] - next_tile[0]) < 0.1 and abs(pos[1] - next_tile[1]) < 0.1:
                break
            shared_robot.move_forward(speed=3.0)
            pb.stepSimulation()
            time.sleep(1./240.)

    DIRECTION_TO_YAW = {
        "RIGHT": 0,  # +X
        "UP": math.pi / 2,  # +Y
        "LEFT": math.pi,  # -X
        "DOWN": -math.pi / 2  # -Y
    }

    # Rotate until facing the store front
    target_yaw = DIRECTION_TO_YAW.get(store.direction, 0)
    while True:
        pos, orn = pb.getBasePositionAndOrientation(shared_robot.body_id)
        euler = pb.getEulerFromQuaternion(orn)
        yaw = euler[2]
        angle_diff = (target_yaw - yaw + math.pi) % (2*math.pi) - math.pi  # normalize to [-pi, pi]

        if abs(angle_diff) < 0.02:
            break
        spin_speed = 3 if angle_diff > 0 else -3
        shared_robot.spin(speed=spin_speed)
        pb.stepSimulation()
        time.sleep(1./240.)

    # Point arm to the correct floor at destination
    shared_robot.point_to_floor(store.floor)
    print(f"Arrived at {store.name}, floor {store.floor}, facing {store.direction}")

def run_gui():
    root = tk.Tk()
    root.title("Store Selector")

    # Create 9 buttons in a grid
    row, col = 0, 0
    for store in STORE_LIST:
        btn = tk.Button(root, text=store.name, width=24, height=6,
                        command=lambda s=store: goto_store(s))
        btn.grid(row=row, column=col, padx=5, pady=5)
        col += 1
        if col >= 3:
            col = 0
            row += 1

    root.mainloop()


if __name__ == "__main__":
    shared_robot = None

    # Run PyBullet in a background thread
    sim_thread = threading.Thread(target=run_pybullet, daemon=True)
    sim_thread.start()

    # Run GUI in main thread
    run_gui()
