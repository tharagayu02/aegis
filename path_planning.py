import heapq
import math


def plan_astar_route(
    start: tuple[float, float],
    goal: tuple[float, float],
    polygons: list[list[tuple[float, float]]],
    cell_size_m: float = 25,
    clearance_m: float = 18,
) -> list[tuple[float, float]]:
    """Return a short lat/lon route around polygon obstacles using an 8-way A* grid."""
    origin_lat, origin_lon = start
    longitude_scale = 111320 * max(math.cos(math.radians(origin_lat)), 0.01)

    def project(point: tuple[float, float]) -> tuple[float, float]:
        latitude, longitude = point
        return (
            (longitude - origin_lon) * longitude_scale,
            (latitude - origin_lat) * 110540,
        )

    def unproject(point: tuple[float, float]) -> tuple[float, float]:
        x, y = point
        return origin_lat + y / 110540, origin_lon + x / longitude_scale

    projected_polygons = [
        [project((latitude, longitude)) for longitude, latitude in polygon]
        for polygon in polygons
        if len(polygon) >= 3
    ]
    start_xy = project(start)
    goal_xy = project(goal)
    all_x = [start_xy[0], goal_xy[0]]
    all_y = [start_xy[1], goal_xy[1]]
    for polygon in projected_polygons:
        all_x.extend(point[0] for point in polygon)
        all_y.extend(point[1] for point in polygon)

    margin = max(100, clearance_m + cell_size_m * 2)
    min_x, max_x = min(all_x) - margin, max(all_x) + margin
    min_y, max_y = min(all_y) - margin, max(all_y) + margin
    columns = math.ceil((max_x - min_x) / cell_size_m)
    rows = math.ceil((max_y - min_y) / cell_size_m)
    if columns * rows > 40000:
        return []

    def to_cell(point: tuple[float, float]) -> tuple[int, int]:
        return (
            round((point[0] - min_x) / cell_size_m),
            round((point[1] - min_y) / cell_size_m),
        )

    def cell_center(cell: tuple[int, int]) -> tuple[float, float]:
        return min_x + cell[0] * cell_size_m, min_y + cell[1] * cell_size_m

    def segment_distance(
        point: tuple[float, float],
        start_point: tuple[float, float],
        end_point: tuple[float, float],
    ) -> float:
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        length_squared = dx * dx + dy * dy
        if length_squared == 0:
            return math.dist(point, start_point)
        amount = max(
            0,
            min(
                1,
                ((point[0] - start_point[0]) * dx + (point[1] - start_point[1]) * dy)
                / length_squared,
            ),
        )
        nearest = (start_point[0] + amount * dx, start_point[1] + amount * dy)
        return math.dist(point, nearest)

    def blocked(point: tuple[float, float]) -> bool:
        x, y = point
        for polygon in projected_polygons:
            inside = False
            previous = polygon[-1]
            for current in polygon:
                if (current[1] > y) != (previous[1] > y):
                    crossing_x = (previous[0] - current[0]) * (y - current[1]) / (
                        previous[1] - current[1]
                    ) + current[0]
                    if x < crossing_x:
                        inside = not inside
                if segment_distance(point, current, previous) <= clearance_m:
                    return True
                previous = current
            if inside:
                return True
        return False

    start_cell, goal_cell = to_cell(start_xy), to_cell(goal_xy)
    if start_cell == goal_cell:
        return [start, goal]

    neighbors = [
        (-1, -1, math.sqrt(2)),
        (-1, 0, 1),
        (-1, 1, math.sqrt(2)),
        (0, -1, 1),
        (0, 1, 1),
        (1, -1, math.sqrt(2)),
        (1, 0, 1),
        (1, 1, math.sqrt(2)),
    ]

    def heuristic(cell: tuple[int, int]) -> float:
        return math.hypot(goal_cell[0] - cell[0], goal_cell[1] - cell[1])

    frontier = [(heuristic(start_cell), 0, start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    costs = {start_cell: 0.0}
    visited: set[tuple[int, int]] = set()

    while frontier and len(visited) < 40000:
        _, cost, cell = heapq.heappop(frontier)
        if cell in visited or cost > costs.get(cell, math.inf):
            continue
        visited.add(cell)
        if cell == goal_cell:
            cells = [cell]
            while cells[-1] != start_cell:
                cells.append(came_from[cells[-1]])
            cells.reverse()
            return [start, *(unproject(cell_center(item)) for item in cells[1:-1]), goal]

        for dx, dy, step_cost in neighbors:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if not (0 <= neighbor[0] <= columns and 0 <= neighbor[1] <= rows):
                continue
            if neighbor != goal_cell and neighbor != start_cell and blocked(cell_center(neighbor)):
                continue
            if dx and dy:
                if blocked(cell_center((cell[0] + dx, cell[1]))) or blocked(
                    cell_center((cell[0], cell[1] + dy))
                ):
                    continue
            new_cost = cost + step_cost
            if new_cost >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = new_cost
            came_from[neighbor] = cell
            heapq.heappush(frontier, (new_cost + heuristic(neighbor), new_cost, neighbor))

    return []
