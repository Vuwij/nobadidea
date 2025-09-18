class Store:
    def __init__(self, name, location, floor=1, direction="UP"):
        """
        Store class
        :param name: str, store name
        :param location: tuple (x, y, z)
        :param floor: int, floor number
        :param direction: str, store entrance direction
        """
        self.name = name
        self.location = location
        self.floor = floor
        self.direction = direction