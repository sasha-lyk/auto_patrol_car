"""Route file validation and map fingerprinting without ROS dependencies."""

import hashlib
import math
import os
import ast

class RouteValidationError(ValueError):
    pass


def _sha256_file(path, digest):
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)


def map_bundle_sha256(map_yaml):
    """Hash both the map YAML and its referenced image."""
    map_yaml = os.path.abspath(os.path.expanduser(map_yaml))
    image = ''
    with open(map_yaml, encoding='utf-8') as stream:
        for line in stream:
            if line.lstrip().startswith('image:'):
                image = line.split(':', 1)[1].strip().strip('"\'')
                break
    if not image:
        raise RouteValidationError('map YAML has no image field: %s' % map_yaml)
    image_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(map_yaml), image)
    digest = hashlib.sha256()
    _sha256_file(map_yaml, digest)
    _sha256_file(os.path.abspath(image_path), digest)
    return digest.hexdigest()


def _map_config(map_yaml):
    config = {}
    with open(map_yaml, encoding='utf-8') as stream:
        for raw_line in stream:
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            config[key.strip()] = value.strip().strip('"\'')
    try:
        config['resolution'] = float(config['resolution'])
        config['origin'] = ast.literal_eval(config['origin'])
        config['negate'] = int(config.get('negate', '0'))
        config['free_thresh'] = float(config.get('free_thresh', '0.25'))
    except (KeyError, ValueError, SyntaxError) as exc:
        raise RouteValidationError('invalid map YAML metadata: %s' % map_yaml) from exc
    image = config.get('image', '')
    config['image_path'] = (
        image if os.path.isabs(image) else os.path.join(os.path.dirname(map_yaml), image))
    return config


def _read_pgm(path):
    with open(path, 'rb') as stream:
        if stream.readline().strip() != b'P5':
            raise RouteValidationError('only binary PGM (P5) maps are supported')

        def tokens_needed(count):
            tokens = []
            while len(tokens) < count:
                line = stream.readline()
                if not line:
                    break
                line = line.split(b'#', 1)[0]
                tokens.extend(line.split())
            return tokens

        dimensions = tokens_needed(2)
        maximum = tokens_needed(1)
        if len(dimensions) != 2 or len(maximum) != 1:
            raise RouteValidationError('invalid PGM header: %s' % path)
        width, height = map(int, dimensions)
        if int(maximum[0]) != 255:
            raise RouteValidationError('PGM max value must be 255')
        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            raise RouteValidationError('truncated PGM image: %s' % path)
    return width, height, pixels


def validate_waypoints_on_map(waypoints, map_yaml, clearance_m=0.30):
    map_yaml = os.path.abspath(os.path.expanduser(map_yaml))
    config = _map_config(map_yaml)
    width, height, pixels = _read_pgm(config['image_path'])
    resolution = config['resolution']
    origin_x, origin_y = float(config['origin'][0]), float(config['origin'][1])
    clearance_cells = int(math.ceil(max(0.0, float(clearance_m)) / resolution))

    def is_free(column, row):
        if column < 0 or row < 0 or column >= width or row >= height:
            return False
        pixel = pixels[row * width + column]
        occupancy = pixel / 255.0 if config['negate'] else (255 - pixel) / 255.0
        return occupancy < config['free_thresh']

    for index, waypoint in enumerate(waypoints):
        column = int(math.floor((waypoint['x'] - origin_x) / resolution))
        map_row = int(math.floor((waypoint['y'] - origin_y) / resolution))
        row = height - 1 - map_row
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if dx * dx + dy * dy > clearance_cells * clearance_cells:
                    continue
                if not is_free(column + dx, row + dy):
                    raise RouteValidationError(
                        'waypoint %d is outside free space or lacks %.2fm clearance'
                        % (index + 1, clearance_m))
    return True


def validate_route(data, map_yaml=None, require_calibrated=True, min_waypoints=2):
    if not isinstance(data, dict):
        raise RouteValidationError('route file must contain a YAML mapping')
    metadata = data.get('route') or {}
    if require_calibrated and metadata.get('calibrated') is not True:
        raise RouteValidationError('route is not field-calibrated; run waypoint_recorder_node')
    if metadata.get('frame_id', 'map') != 'map':
        raise RouteValidationError('only map-frame patrol routes are supported')
    waypoints = data.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) < min_waypoints:
        raise RouteValidationError('route needs at least %d waypoints' % min_waypoints)
    normalized = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise RouteValidationError('waypoint %d is not a mapping' % (index + 1))
        try:
            x = float(waypoint['x'])
            y = float(waypoint['y'])
            yaw = float(waypoint.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise RouteValidationError('waypoint %d has invalid x/y/yaw' % (index + 1)) from exc
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise RouteValidationError('waypoint %d contains a non-finite value' % (index + 1))
        normalized.append({'x': x, 'y': y, 'yaw': yaw})
    if map_yaml:
        expected = str(metadata.get('map_sha256', '')).lower()
        if not expected:
            raise RouteValidationError('route has no map fingerprint')
        actual = map_bundle_sha256(map_yaml)
        if expected != actual:
            raise RouteValidationError(
                'route/map fingerprint mismatch: route=%s map=%s' % (expected[:12], actual[:12]))
        validate_waypoints_on_map(
            normalized, map_yaml, float(metadata.get('clearance_m', 0.30)))
    return {'route': metadata, 'waypoints': normalized}


def load_route(path, map_yaml=None, require_calibrated=True, min_waypoints=2):
    import yaml

    path = os.path.abspath(os.path.expanduser(path))
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return validate_route(data, map_yaml, require_calibrated, min_waypoints)
