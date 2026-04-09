import numpy as np

class Map:
    def __init__(self, map_msg):
        self.info = map_msg.info
        self.data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

    def world_to_map(self, x_meters, y_meters):
        """
        Convert world coordinates (meters) to map pixel indices.
        """
        if self.info is None:
            return None

        ix = int((x_meters - self.info.origin.position.x) / self.info.resolution)
        iy = int((y_meters - self.info.origin.position.y) / self.info.resolution)

        if 0 <= ix < self.info.width and 0 <= iy < self.info.height:
            return np.array([ix, iy])
        else:
            return None # Out of bounds

    def map_to_index(self, map_x, map_y):
        """
        Convert map pixel indices to a single index in the occupancy grid data array.
        """
        if self.info is None:
            return None

        if 0 <= map_x < self.info.width and 0 <= map_y < self.info.height:
            return map_y * self.info.width + map_x
        else:
            return None # Out of bounds

    def world_to_index(self, x_meters, y_meters):
        """
        Convert world coordinates (meters) directly to a single index in the occupancy grid data array.
        """
        map_coords = self.world_to_map(x_meters, y_meters)
        if map_coords is not None:
            return self.map_to_index(map_coords[0], map_coords[1])
        else:
            return None # Out of bounds

    def index_to_map(self, index):
        """
        Convert a single index in the occupancy grid data array back to map pixel indices.
        """
        if self.info is None:
            return None

        if 0 <= index < self.info.width * self.info.height:
            map_y = index // self.info.width
            map_x = index % self.info.width
            return np.array([map_x, map_y])
        else:
            return None

    def map_to_world(self, map_x, map_y):
        """
        Convert map pixel indices back to world coordinates (meters).
        """
        if self.info is None:
            return None

        x_meters = self.info.origin.position.x + (map_x + 0.5) * self.info.resolution
        y_meters = self.info.origin.position.y + (map_y + 0.5) * self.info.resolution
        return np.array([x_meters, y_meters])

    def index_to_world(self, index):
        """
        Convert a single index in the occupancy grid data array back to world coordinates (meters).
        """
        map_coords = self.index_to_map(index)
        if map_coords is not None:
            return self.map_to_world(map_coords[0], map_coords[1])
        else:
            return None

    def get_occupancy(self, x_meters, y_meters):
        """
        Convert world coordinates (meters) to map occupancy value.
        """
        if self.info is None:
            return None

        ix = int((x_meters - self.info.origin.position.x) / self.info.resolution)
        iy = int((y_meters - self.info.origin.position.y) / self.info.resolution)

        if 0 <= ix < self.info.width and 0 <= iy < self.info.height:
            return self.data[iy, ix]
        else:
            return -1

    def is_free_world(self, x_meters, y_meters):
        """
        Check if the given world coordinates (meters) correspond to a free cell in the occupancy grid.
        """
        occupancy = self.get_occupancy(x_meters, y_meters)
        return occupancy == 0

    def is_free_map(self, map_x, map_y):
        """
        Check if the given map coordinates correspond to a free cell in the occupancy grid.
        """
        if 0 <= map_x < self.info.width and 0 <= map_y < self.info.height:
            return self.data[map_y, map_x] == 0
        else:
            return False

    def is_line_free(self, p1, p2):
        """
        Check if straight line between two world points is collision-free using Bresenham
        """
        m1 = self.world_to_map(p1[0], p1[1])
        m2 = self.world_to_map(p2[0], p2[1])

        if m1 is None or m2 is None:
            return False

        x0, y0 = m1
        x1, y1 = m2

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x, y = x0, y0

        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if not self.is_free_map(x, y):
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not self.is_free_map(x, y):
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        return self.is_free_map(x1, y1)

    def simplify_path(self, path):
        """
        Simplify a path by removing unnecessary waypoints.
        Input: list of [x, y] world coordinates
        Output: simplified list of [x, y]
        """
        if path is None or len(path) < 3:
            return path

        simplified = [path[0]]
        i = 0

        while i < len(path) - 1:
            j = i + 1

            while j < len(path) and self.is_line_free(path[i], path[j]):
                j += 1

            simplified.append(path[j - 1])
            i = j - 1

        return simplified