import os
from xml.etree import ElementTree as ElTree
import yaml
from PIL import Image
from warnings import warn

from knowledge_representation.map_image_utils import point_to_map_coords
from svgpathtools.parser import parse_transform, parse_path
import numpy as np

svg_el = "{http://www.w3.org/2000/svg}svg"
image_el = "{http://www.w3.org/2000/svg}image"
text_el = "{http://www.w3.org/2000/svg}text"
circle_el = "{http://www.w3.org/2000/svg}circle"
line_el = "{http://www.w3.org/2000/svg}line"
poly_el = "{http://www.w3.org/2000/svg}polygon"
path_el = "{http://www.w3.org/2000/svg}path"
group_el = "{http://www.w3.org/2000/svg}g"
tspan_el = "{http://www.w3.org/2000/svg}tspan"


def get_text_from_group(group):
    # Inkscape tucks things in a tspan. Check that first
    text = group.find(".//{}".format(tspan_el))
    if text is None:
        text = group.find(".//{}".format(text_el))
    if text is None:
        return None
    return text.text


def get_point(element):
    return float_s3(element.attrib["cx"]), float_s3(element.attrib["cy"])


def float_s3(string):
    return round(float(string), 3)


def np_point_to_tuple(np_point):
    return tuple(np_point[:2, 0])


def apply_transform(point, transform):
    if isinstance(point, tuple):
        # Convert to homogeneous form
        point = np.array([[point[0], point[1], 1]])
    return np_point_to_tuple(transform @ point.transpose())


def get_transform(element):
    if "transform" in element.attrib:
        return parse_transform(element.attrib["transform"])
    else:
        return np.identity(3)


def is_line(path_part):
    from svgpathtools import Line
    return isinstance(path_part, Line)


def extract_line_from_path(path, transform=None):
    """
    Treat a path as a line-segment and extract end points. Throws
    if the path isn't a line.

    :param path:
    :param transform: the transform to apply to the coordinates
    :return: tuple of line-segment start and end coordinates
    """
    path_geom = parse_path(path.attrib["d"])

    if len(path_geom) == 1 and is_line(path_geom[0]):
        line = path_geom[0]
        # We assume line starts at origin and points towards the second point
        start_coord = (float_s3(line.start.real), float_s3(line.start.imag))
        end_coord = (float_s3(line.end.real), float_s3(line.end.imag))
        return apply_transform(start_coord, transform), apply_transform(end_coord, transform)
    else:
        raise RuntimeError()


def populate_with_map_annotations(ltmc, map_name, points, poses, regions, doors):
    """
    Inserts a map and supporting geometry into a knowledgebase. Any existing map by the name will be deleted

    Emits warnings for any annotation that can't be added, as when there are name collisions.
    :param ltmc: The knowledgebase to insert into
    :param map_name: Name of the map to create.
    :param points:
    :param poses:
    :param regions:
    :param doors:
    :return: a tuple of counts of how many of each type of annotation were successfully inserted
    """
    # Wipe any existing map by this name
    map = ltmc.get_map(map_name)
    map.delete()
    map = ltmc.get_map(map_name)
    point_count = 0
    pose_count = 0
    region_count = 0
    door_count = 0

    for name, point in points:
        point = map.add_point(name, *point)
        if not point.is_valid():
            warn("Failed to add point '{}': {}".format(name, *point))
        else:
            point_count += 1

    for name, (p1_x, p1_y), (p2_x, p2_y) in poses:
        pose = map.add_pose(name, p1_x, p1_y, p2_x, p2_y)
        if not pose.is_valid():
            warn("Failed to add pose '{}': {}".format(name, *((p1_x, p1_y), (p2_x, p2_y))))
        else:
            pose_count += 1

    for name, points in regions:
        region = map.add_region(name, points)
        if not region.is_valid():
            warn("Failed to add region '{}': {}".format(name, *points))
        else:
            region_count += 1

    door_count, extra_points = populate_doors(doors, map)
    point_count += extra_points
    return point_count, pose_count, region_count, door_count


def populate_doors(doors, map):
    door_count = 0
    point_count = 0
    for name, ((x_0, y_0), (x_1, y_1)), approach_points in doors:
        door = map.add_door(name, x_0, y_0, x_1, y_1)
        if not door.is_valid():
            warn("Failed to add door '{}': ({},{}) ({},{})".format(name, x_0, y_0, x_0, y_1))
            continue
        for i, (x, y) in enumerate(approach_points):
            point = map.add_point("{}_approach{}".format(name, i), x, y)
            if not point.is_valid():
                warn("Failed to add approach {} for door '{}': {}, {}".format(i, name, x, y))
                continue
            point.add_attribute("approach_to", door)
            point_count += 1
        door_count += 1
    return door_count, point_count


def load_map_from_yaml(path_to_yaml, use_pixel_coords=False):
    """
    Attempt to load map annotations given a path to a YAML file. Emits warnings for issues with particular annotations.

    The PGM and the SVG will be loaded in the process. The SVG file to load is determined by the `annotations` key,
    or is an SVG with the same name as the YAML file.
    :param path_to_yaml:
    :return: a tuple of the name of the map and a nested tuple of points, poses and regions
    """
    parent_dir = os.path.dirname(path_to_yaml)
    yaml_name = os.path.basename(path_to_yaml).split(".")[0]
    with open(path_to_yaml) as map_yaml:
        map_metadata = yaml.load(map_yaml, Loader=yaml.FullLoader)
    map_metadata["name"] = yaml_name
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
        # No annotations to load. Since you're trying to load annotations, this is probably an error of some sort
        warn("No annotation file found at {}".format(annotation_path))
        return yaml_name, None

    with open(annotation_path) as test_svg:
        svg_data = test_svg.readlines()
        svg_data = " ".join(svg_data)

    svg_problems = check_svg_valid(svg_data, map_metadata)
    for prob in svg_problems:
        warn(prob)
    annotations = process_svg(svg_data)
    if not use_pixel_coords:
        annotations = transform_to_map_coords(map_metadata, *annotations)
    return map_metadata, annotations


def check_svg_valid(svg_data, map_info):
    problems = []
    tree = ElTree.fromstring(svg_data)
    # Root of tree is the SVG element
    svg = tree
    image = tree.find(image_el)

    def ori_is_zero(element):
        # Annotation tool may not add this
        if 'x' not in element.attrib:
            return
        ori_x = float(element.attrib['x'])
        ori_y = float(element.attrib['y'])

        if ori_x != 0 or ori_y != 0:
            problems.append("Image origin is ({}, {}) not (0, 0)".format(ori_x, ori_y))

    def dim_match(element, w, h):
        # Annotation tool may not add this
        if 'width' not in element.attrib:
            return
        e_w = float(element.attrib['width'])
        e_h = float(element.attrib['height'])

        if e_w != w or e_h != h:
            problems.append(
                "SVG or image dimensions are {}x{}, but YAML says they should be {}x{}".format(e_w, e_h, w, h))

    viewbox = svg.attrib["viewBox"]
    target_viewbox = "0 0 {} {}".format(map_info["width"], map_info["height"])
    if viewbox != target_viewbox:
        problems.append("SVG viewbox is {} but should be {}".format(viewbox, target_viewbox))
    dim_match(svg, map_info["width"], map_info["height"])
    ori_is_zero(image)
    dim_match(image, map_info["width"], map_info["height"])
    return problems


def process_svg(svg_data):
    """
    Extracts annotations from SVG data. See documentation for an explanation of
    how annotations are expected to be structured.

    :param svg_data: string containing SVG data
    :return: extracted point, pose, region and door annotations
    """
    tree = ElTree.fromstring(svg_data)
    parent_map = {c: p for p in tree.iter() for c in p}
    point_annotations = tree.findall(".//{}[@class='circle_annotation']".format(circle_el))
    point_names = tree.findall(".//{}[@class='circle_annotation']/../{}".format(circle_el, text_el))
    pose_annotations = tree.findall(".//{}[@class='pose_line_annotation']".format(line_el))
    pose_names = tree.findall(".//{}[@class='pose_line_annotation']/../{}".format(line_el, text_el))
    region_annotations = tree.findall(".//{}[@class='region_annotation']".format(poly_el))
    region_names = tree.findall(".//{}[@class='region_annotation']/../{}".format(poly_el, text_el))
    path_groups = tree.findall(".//{}[{}]".format(group_el, path_el))
    circle_groups = tree.findall(".//{}[{}]".format(group_el, circle_el))
    # The point annotations we care about have just a dot and a text label
    point_groups = filter(lambda g: len(list(g)) == 2, circle_groups)
    # Door annotations have a line, two circles and a text label
    door_groups = filter(lambda g: len(list(g)) == 4, circle_groups)

    point_parents = map(parent_map.__getitem__, point_annotations)
    points = process_point_annotations(point_names, point_annotations, point_parents)
    pose_parents = map(parent_map.__getitem__, pose_annotations)
    poses = process_pose_annotations(pose_names, pose_annotations, pose_parents)
    region_parents = map(parent_map.__getitem__, region_annotations)
    regions = process_region_annotations(region_names, region_annotations, region_parents)

    # NOTE(nickswalker): Haven't set a format for these in the annotation tool yet, so inkscape only assumption
    doors = process_door_groups(door_groups)

    # Messier extraction to get annotations stored as paths. These are from Inkscape or other regular editing tools.
    path_poses, path_regions = process_paths(path_groups)
    extra_points = []

    for group in point_groups:
        name = get_text_from_group(group)
        transform = get_transform(group)

        circle = group.find(".//{}".format(circle_el))

        if "class" in circle.attrib:
            # This was probably created by the annotation tool. Already processed above
            continue

        circle_center = get_point(circle)
        pixel_coord = apply_transform(circle_center, transform)
        extra_points.append((name, pixel_coord))

    points += extra_points
    poses += path_poses
    regions += path_regions

    return points, poses, regions, doors


def process_door_groups(door_groups):
    doors = []
    for door_group in door_groups:
        name = get_text_from_group(door_group)
        transform = get_transform(door_group)
        approach_points = []
        circles = door_group.findall(circle_el)
        if len(circles) != 2:
            # Would we ever want more than 2 approach points?
            warn("Can't process door group '{}' because it had {} approach points (2 are expected)".format(name, len(
                circles)))
            continue
        for circle in circles:
            center = get_point(circle)
            approach_points.append(apply_transform(center, transform))
        try:
            door_line = extract_line_from_path(door_group.find(path_el), transform)
        except RuntimeError:
            warn("Couldn't extract line from door group '{}'".format(name))
            continue
        doors.append((name, door_line, approach_points))

    return doors


def process_paths(path_groups):
    """
    Extracts pose and region annotations represented as paths

    :param path_groups: a list of groups each containing a text element and a path
    :return: a tuple of poses and regions
    """
    if len(path_groups) == 0:
        return [], []

    regions = []
    poses = []
    for group in path_groups:
        if len(list(group)) != 2:
            # May want to print a warning here
            continue

        # We assume that the text was created in inkscape so the string will be in a tspan
        path, text = group.find(".//{}".format(path_el)), get_text_from_group(group)
        if text is None:
            warn("No text label found for path group: {}".format(group))
            continue
        name = text

        transform = get_transform(group)

        # Single line segment path => pose
        try:
            pose = extract_line_from_path(path, transform)
            poses.append(tuple([name] + list(pose)))
            continue
        except RuntimeError:
            pass

        # SVG paths are specified in a rich language of segment commands:
        # https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/d
        # We'll use a new dependency to extract what we can
        path_geom = parse_path(path.attrib["d"])
        # If they're all lines, let's assume it's closed and use it as a region
        if all(map(is_line, path_geom)):
            # Real part => x, imag part => y
            lines = map(lambda l: ((l.start.real, l.start.imag), (l.end.real, l.end.imag)), path_geom)
            # Each line segment starts where the previous ended, so we can drop the end points
            points = map(lambda l: l[0], lines)
            points = map(lambda p: (float_s3(p[0]), float_s3(p[1])), points)
            points = map(lambda p: apply_transform(p, transform), points)
            regions.append((name, points))
        else:
            warn("Encountered path that couldn't be parsed {}".format(name))
    return poses, regions


def process_point_annotations(point_names, point_annotations, point_groups):
    points = []
    for point, text, parent in zip(point_annotations, point_names, point_groups):
        name = text.text
        pixel_coord = apply_transform(get_point(point), get_transform(parent))
        points.append((name, pixel_coord))
    return points


def process_pose_annotations(pose_names, pose_annotations, pose_groups):
    poses = []
    for pose, text, parent in zip(pose_annotations, pose_names, pose_groups):
        name = text.text
        transform = get_transform(parent)
        start_cord = float_s3(pose.attrib["x1"]), float_s3(pose.attrib["y1"])
        stop_cord = float_s3(pose.attrib["x2"]), float_s3(pose.attrib["y2"])
        poses.append((name, apply_transform(start_cord, transform), apply_transform(stop_cord, transform)))
    return poses


def process_region_annotations(region_names, region_annotations, region_groups):
    regions = []
    for region, text, parent in zip(region_annotations, region_names, region_groups):
        name = text.text
        transform = get_transform(parent)
        points_strs = region.attrib["points"].split()
        poly_points = [(float_s3(x_str), float_s3(y_str)) for x_str, y_str in map(lambda x: x.split(","), points_strs)]
        # Apply any translation
        poly_points = map(lambda p: apply_transform(p, transform), poly_points)
        regions.append((name, poly_points))
    return regions


def transform_to_map_coords(map_info, points, poses, regions, doors):
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

    for i, door in enumerate(doors):
        name, (d_p1, d_p2), (p1, p2) = door
        d_p1 = point_to_map_coords(map_info, d_p1)
        d_p2 = point_to_map_coords(map_info, d_p2)
        p1 = point_to_map_coords(map_info, p1)
        p2 = point_to_map_coords(map_info, p2)
        doors[i] = (name, (d_p1, d_p2), (p1, p2))

    return points, poses, regions, doors
