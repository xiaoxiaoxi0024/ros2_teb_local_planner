#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
WORLD_DIR = REPO_ROOT / 'src' / 'fishbot_description' / 'world'
MAP_DIR = REPO_ROOT / 'src' / 'fishbot_navigation2' / 'maps'

RESOLUTION = 0.02
MARGIN = 1.0
FREE = 254
OCCUPIED = 0

WORLD_TO_MAP = {
    'slam_narrow_corridor.world': 'slam_narrow_corridor',
    'slam_s_curve_corridor.world': 'slam_s_curve_corridor',
    'slam_l_turn_corridor.world': 'slam_l_turn_corridor',
}


def parse_pose(text):
    if not text:
        return 0.0, 0.0, 0.0
    values = [float(v) for v in text.split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[5]


def compose_pose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    return (
        px + cos_yaw * cx - sin_yaw * cy,
        py + sin_yaw * cx + cos_yaw * cy,
        pyaw + cyaw,
    )


def get_direct_pose(element):
    return parse_pose(element.findtext('pose'))


def add_box(shapes, pose, size):
    sx, sy, sz = size
    if sz < 0.05:
        return
    if sx > 30.0 and sy > 30.0:
        return
    shapes.append(('box', pose, sx, sy))


def add_cylinder(shapes, pose, radius, length):
    if length < 0.05:
        return
    shapes.append(('cylinder', pose, radius))


def collect_shapes(node, parent_pose, shapes):
    node_pose = compose_pose(parent_pose, get_direct_pose(node))
    node_tag = node.tag
    node_name = node.attrib.get('name', '')

    if node_tag == 'model' and node_name == 'ground_plane':
        return

    if node_tag == 'collision':
        geometry = node.find('geometry')
        if geometry is not None:
            box = geometry.find('box')
            cylinder = geometry.find('cylinder')
            if box is not None:
                size_text = box.findtext('size')
                if size_text:
                    size = [float(v) for v in size_text.split()]
                    add_box(shapes, node_pose, size)
            elif cylinder is not None:
                radius = float(cylinder.findtext('radius', default='0'))
                length = float(cylinder.findtext('length', default='0'))
                add_cylinder(shapes, node_pose, radius, length)

    for child in list(node):
        if child.tag in {'model', 'link', 'collision'}:
            collect_shapes(child, node_pose, shapes)


def rotated_box_corners(cx, cy, yaw, sx, sy):
    hx = sx / 2.0
    hy = sy / 2.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    corners = []
    for lx, ly in ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy)):
        wx = cx + cos_yaw * lx - sin_yaw * ly
        wy = cy + sin_yaw * lx + cos_yaw * ly
        corners.append((wx, wy))
    return corners


def shape_bounds(shape):
    kind = shape[0]
    if kind == 'box':
        _, (cx, cy, yaw), sx, sy = shape
        corners = rotated_box_corners(cx, cy, yaw, sx, sy)
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return min(xs), max(xs), min(ys), max(ys)

    _, (cx, cy, _), radius = shape
    return cx - radius, cx + radius, cy - radius, cy + radius


def point_in_shape(x, y, shape):
    kind = shape[0]
    if kind == 'box':
        _, (cx, cy, yaw), sx, sy = shape
        dx = x - cx
        dy = y - cy
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        lx = cos_yaw * dx - sin_yaw * dy
        ly = sin_yaw * dx + cos_yaw * dy
        return abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0

    _, (cx, cy, _), radius = shape
    return (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2


def rasterize(shapes, origin_x, origin_y, width, height):
    pixels = bytearray([FREE]) * (width * height)

    for shape in shapes:
        min_x, max_x, min_y, max_y = shape_bounds(shape)
        min_ix = max(0, int(math.floor((min_x - origin_x) / RESOLUTION)) - 1)
        max_ix = min(width - 1, int(math.ceil((max_x - origin_x) / RESOLUTION)) + 1)
        min_iy_world = max(0, int(math.floor((min_y - origin_y) / RESOLUTION)) - 1)
        max_iy_world = min(height - 1, int(math.ceil((max_y - origin_y) / RESOLUTION)) + 1)

        for ix in range(min_ix, max_ix + 1):
            world_x = origin_x + (ix + 0.5) * RESOLUTION
            for iy_world in range(min_iy_world, max_iy_world + 1):
                world_y = origin_y + (iy_world + 0.5) * RESOLUTION
                if point_in_shape(world_x, world_y, shape):
                    row = height - 1 - iy_world
                    pixels[row * width + ix] = OCCUPIED

    return pixels


def build_map(world_path, stem):
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise RuntimeError(f'No <world> found in {world_path}')

    shapes = []
    collect_shapes(world, (0.0, 0.0, 0.0), shapes)
    if not shapes:
        raise RuntimeError(f'No collision geometry found in {world_path}')

    min_x = min(shape_bounds(shape)[0] for shape in shapes) - MARGIN
    max_x = max(shape_bounds(shape)[1] for shape in shapes) + MARGIN
    min_y = min(shape_bounds(shape)[2] for shape in shapes) - MARGIN
    max_y = max(shape_bounds(shape)[3] for shape in shapes) + MARGIN

    width = int(math.ceil((max_x - min_x) / RESOLUTION))
    height = int(math.ceil((max_y - min_y) / RESOLUTION))
    pixels = rasterize(shapes, min_x, min_y, width, height)

    pgm_path = MAP_DIR / f'{stem}.pgm'
    yaml_path = MAP_DIR / f'{stem}.yaml'

    with pgm_path.open('wb') as pgm_file:
        pgm_file.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        pgm_file.write(pixels)

    yaml_path.write_text(
        '\n'.join([
            f'image: {pgm_path.name}',
            'mode: trinary',
            f'resolution: {RESOLUTION:.2f}',
            f'origin: [{min_x:.3f}, {min_y:.3f}, 0.0]',
            'negate: 0',
            'occupied_thresh: 0.65',
            'free_thresh: 0.25',
            '',
        ]),
        encoding='ascii',
    )

    print(f'Generated {yaml_path}')


def main():
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    for world_name, stem in WORLD_TO_MAP.items():
        build_map(WORLD_DIR / world_name, stem)


if __name__ == '__main__':
    main()
