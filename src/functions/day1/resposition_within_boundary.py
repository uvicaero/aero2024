from shapely.geometry import Point, LineString, Polygon

# Define the two convex polygons
polygon1 = Polygon([
    (-110.7329257, 50.0971537),
    (-110.7381487, 50.0971788),
    (-110.7381534, 50.0989139),
    (-110.7328910, 50.0988950)
])

polygon2 = Polygon([
    (-110.7328910, 50.0988950),
    (-110.7381534, 50.0989139),
    (-110.7436555, 50.1035452),
    (-110.7436756, 50.1060793),
    (-110.7328869, 50.1060519)
])

# Define the shared boundary points
shared_boundary = [
    (-110.7328910, 50.0988950),
    (-110.7381534, 50.0989139)
]


def move_command(current_position, target_point, boundary):
    """
    Main reposition command that checks boundary constraints before issuing movement commands.

    Parameters:
    - current_position: Tuple (x, y) representing the current location.
    - target_point: Tuple (x, y) representing the intended destination.
    - boundary: A polygon or set of constraints defining the allowed movement area.

    Returns:
    - None (but sends movement commands)
    """
    if not is_inside_boundary(target_point, boundary):
        # Case 2: Target point is out of bounds
        adjusted_point = adjust_target_point(target_point, boundary)
        send_reposition_command(adjusted_point)
    elif not is_path_fully_contained(current_position, target_point, boundary):
        # Case 3: Path crosses boundary
        segment1, segment2 = adjust_path(current_position, target_point, boundary)
        send_reposition_command(segment1)
        send_reposition_command(segment2)
    else:
        # Case 1: Everything is valid, send reposition command as is
        send_reposition_command(target_point)

def adjust_target_point(target_point, boundary):
    """
    Adjusts the target point to the nearest valid position inside the boundary.

    Parameters:
    - target_point: Tuple (x, y) representing the original out-of-bounds destination.
    - boundary: A polygon or set of constraints defining the allowed movement area.

    Returns:
    - adjusted_point: Tuple (x, y) of the nearest valid position.
    """
    # Find the closest valid point inside the boundary
    adjusted_point = find_nearest_valid_point(target_point, boundary)
    return adjusted_point

def adjust_path(current_position, target_point, boundary):
    """
    Adjusts the movement path to ensure it remains inside the boundary.

    Parameters:
    - current_position: Tuple (x, y) representing the current location.
    - target_point: Tuple (x, y) representing the intended destination.
    - boundary: A polygon or set of constraints defining the allowed movement area.

    Returns:
    - segment1: First waypoint inside the boundary.
    - segment2: Second waypoint inside the boundary, reaching the target.
    """
    # Compute two waypoints that keep the movement within the boundary
    segment1, segment2 = find_valid_path_segments(current_position, target_point, boundary)
    return segment1, segment2

def is_inside_boundary(point, boundary):
    """
    Checks if a given point is inside the boundary.

    Parameters:
    - point: Tuple (x, y) representing the location to check.
    - boundary: A Polygon representing the allowed movement area.

    Returns:
    - Boolean: True if inside, False otherwise.
    """
    return boundary.contains(Point(point))

def is_path_fully_contained(start, end, boundary):
    """
    Checks if the direct path between start and end is fully inside the boundary.

    Parameters:
    - start: Tuple (x, y) representing the start location.
    - end: Tuple (x, y) representing the destination.
    - boundary: A Polygon representing the allowed movement area.

    Returns:
    - Boolean: True if the path is fully inside, False otherwise.
    """
    path = LineString([start, end])
    return boundary.contains(path)

def find_nearest_valid_point(point, boundary):
    """
    Finds the closest valid point inside the boundary.

    Parameters:
    - point: Tuple (x, y) representing the out-of-bounds location.
    - boundary: A Polygon representing the allowed movement area.

    Returns:
    - Tuple (x, y): Nearest valid location inside the boundary.
    """
    if boundary.contains(Point(point)):
        return point  # Already inside, no adjustment needed

    nearest_point = boundary.exterior.interpolate(boundary.exterior.project(Point(point)))
    return (nearest_point.x, nearest_point.y)

def find_valid_path_segments(start, end):
    """
    Finds a valid path to keep the movement inside the boundary.

    Assumes move_command already checked if the path is fully contained.
    Returns:
    - If crossing sub-polygons, returns (nearest boundary point, end).
    - Otherwise, returns (start, end) as no adjustment is needed.
    """
    if is_inside_boundary(start) and is_inside_boundary(end):
        return start, end  # Already valid

    # Transitioning between polygons → move through shared boundary
    boundary_point = find_nearest_shared_boundary_point(start)
    return boundary_point, end

def find_nearest_shared_boundary_point(point):
    """Finds the nearest point on the shared boundary."""
    return min(shared_boundary, key=lambda p: Point(p).distance(Point(point)))

def send_reposition_command(position):
    """
    Sends a command to reposition the vehicle to the specified location.

    Parameters:
    - position: Tuple (x, y) representing the new target location.

    Returns:
    - None
    """
    print(f"Repositioning to: {position}")
    # Replace with actual command-sending logic




