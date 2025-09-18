from enum import Enum

class TileType(Enum):
    START = 1
    ROAD = 1
    BUILDING = 2

class Tile:
    def __init__(self, tile_type, pos, height=0, road_style=None):
        """
        tile_type: TileType.ROAD or TileType.BUILDING
        pos: (x, y)
        height: only for buildings
        road_style: for roads, one of "R", "|", "/" (ignored for buildings)
        """
        self.type = tile_type
        self.pos = pos
        self.height = height
        self.road_style = road_style  # None for buildings

    def __repr__(self):
        return f"Tile(type={self.type}, pos={self.pos}, height={self.height}, road_style={self.road_style})"


def load_tiles(file_path, tile_size=1):
    """Load the map from a txt file and return a list of Tile objects."""
    with open(file_path, "r") as f:
        lines = [line.rstrip("\n") for line in f.readlines()]

    tiles = []
    height = len(lines)
    width = max(len(line) for line in lines)

    for row_idx, line in enumerate(lines):
        for col_idx, char in enumerate(line):
            x = col_idx * tile_size
            y = (height - row_idx - 1) * tile_size  # invert y so first line is top
            if char == 'S':
                tiles.append(Tile(TileType.START, (x, y), height))
            if char in ("R", "|", "/"):
                tiles.append(Tile(tile_type=TileType.ROAD, pos=(x, y), road_style=char))
            elif char.isdigit():
                tiles.append(Tile(tile_type=TileType.BUILDING, pos=(x, y), height=int(char)))
            # ignore spaces or "C" for now

    return tiles, width, height
