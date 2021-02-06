
def point_to_map_coords(map_info, point):
    """
    Converts a pixel coordinate point into map coordinates
    :param map_info: A dictionary specifying the origin, resolution, width and height of the map
    :param point: A coordinate tuple in pixel coordinates
    :return: A coordinate tuple in map coordinates
    """
    map_origin, resolution, _, height = map_info["origin"][0:2], map_info["resolution"], map_info["width"], map_info[
        "height"]
    x, y = point
    # the map coordinate corresponding to the bottom left pixel
    origin_x, origin_y = map_origin
    vertically_flipped_point = x, height - y - 1
    map_x, map_y = vertically_flipped_point
    point = origin_x + map_x * resolution, origin_y + map_y * resolution
    return point


def draw_doors(map_image, doors):
    from PIL import ImageDraw

    draw = ImageDraw.Draw(map_image)
    for name, door_line, approach_points in doors:
        draw.line(door_line, fill=0, width=1)
    del draw
