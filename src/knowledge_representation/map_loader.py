import os
from xml.etree import ElementTree as ElTree
import yaml
from PIL import Image
from math import atan2

text_el = "{http://www.w3.org/2000/svg}text"
circle_el = "{http://www.w3.org/2000/svg}circle"
line_el = "{http://www.w3.org/2000/svg}line"
poly_el = "{http://www.w3.org/2000/svg}polygon"


def populate_with_map_annotations(ltmc, map_name, points, poses, regions):
    # Wipe any existing map by this name
    map = ltmc.get_map(map_name)
    map.delete()
    map = ltmc.get_map(map_name)

    for name, point in points:
        point = map.add_point(name, *point)
        assert point.is_valid()

    for name, (p1_x, p1_y), (p2_x, p2_y) in poses:
        pose = map.add_pose(name, p1_x, p1_y, p2_x, p2_y)
        assert pose

    for name, points in regions:
        region = map.add_region(name, points)
        assert region


def load_map_from_yaml(path_to_yaml):
    parent_dir = os.path.dirname(path_to_yaml)
    yaml_name = os.path.basename(path_to_yaml).split(".")[0]
    with open(path_to_yaml) as map_yaml:
        map_metadata = yaml.load(map_yaml, Loader=yaml.FullLoader)

    image_path = os.path.join(parent_dir, map_metadata["image"])
    map_image = Image.open(image_path)
    map_metadata["width"] = map_image.size[0]
    map_metadata["height"] = map_image.size[1]

    if "annotations" not in map_metadata:
        # Fallback when no annotations key is given: look for an svg
        # file that has the same name as the yaml file
        annotation_path = os.path.join(parent_dir, yaml_name + ".svg")
    else:
        annotation_path = os.path.join(parent_dir, map_metadata["annotations"])

    if not os.path.isfile(annotation_path):
        return None

    with open(annotation_path) as test_svg:
        svg_data = test_svg.readlines()
        svg_data = " ".join(svg_data)

    annotations = load_svg(svg_data)
    annotations = transform_to_map_coords(map_metadata, *annotations)
    return yaml_name, annotations


def load_svg(svg_data):
    tree = ElTree.fromstring(svg_data)
    point_annotations = tree.findall(".//{}[@class='circle_annotation']".format(circle_el))
    point_names = tree.findall(".//{}[@class='circle_annotation']/../{}".format(circle_el, text_el))
    pose_annotations = tree.findall(".//{}[@class='pose_line_annotation']".format(line_el))
    pose_names = tree.findall(".//{}[@class='pose_line_annotation']/../{}".format(line_el, text_el))
    region_annotations = tree.findall(".//{}[@class='region_annotation']".format(poly_el))
    region_names = tree.findall(".//{}[@class='region_annotation']/../{}".format(poly_el, text_el))

    points = []
    poses = []
    regions = []
    for point, text in zip(point_annotations, point_names):
        name = text.text
        pixel_coord = float(point.attrib["cx"]), float(point.attrib["cy"])
        points.append((name, pixel_coord))

    for pose, text in zip(pose_annotations, pose_names):
        name = text.text
        start_cord = float(pose.attrib["x1"]), float(pose.attrib["y1"])
        stop_cord = float(pose.attrib["x2"]), float(pose.attrib["y2"])
        poses.append((name, start_cord, stop_cord))

    for region, text in zip(region_annotations, region_names):
        name = text.text
        points_strs = region.attrib["points"].split()
        poly_points = [(float(x_str), float(y_str)) for x_str, y_str in map(lambda x: x.split(","), points_strs)]
        regions.append((name, poly_points))

    return points, poses, regions


def transform_to_map_coords(map_info, points, poses, regions):
    for i, point in enumerate(points):
        name, point = point
        point = point_to_map_coords(map_info, point)
        points[i] = (name, point)

    for i, pose in enumerate(poses):
        name, p1, p2 = pose
        p1 = point_to_map_coords(map_info, p1)
        p2 = point_to_map_coords(map_info, p2)
        poses[i] = (name, p1, p2)

    for i, region in enumerate(regions):
        name, poly_points = region
        poly_points = list(map(lambda p: point_to_map_coords(map_info, p), poly_points))
        regions[i] = (name, poly_points)

    return points, poses, regions


def point_to_map_coords(map_info, point):
    map_origin, resolution, width, height = map_info["origin"][0:2], map_info["resolution"], map_info["width"], \
                                            map_info["height"]
    x, y = point
    # the map coordinate corresponding to the bottom left pixel
    origin_x, origin_y = map_origin
    vertically_flipped_point = x, height - y - 1
    map_x, map_y = vertically_flipped_point
    point = origin_x + map_x * resolution, origin_y + map_y * resolution
    return point
