"""Save a map and marker result together without requiring a map-saver node."""

import json
from pathlib import Path

import numpy as np
import yaml


def save_run(directory, run_id, grid, result):
    destination = Path(directory).expanduser() / run_id
    destination.mkdir(parents=True, exist_ok=True)
    if grid is not None:
        pixels = np.full(grid.data.shape, 205, dtype=np.uint8)
        pixels[(grid.data >= 0) & (grid.data <= 25)] = 254
        pixels[grid.data >= 65] = 0
        height, width = pixels.shape
        # OccupancyGrid row zero is at the bottom; PGM row zero is at the top.
        with (destination / 'map.pgm').open('wb') as handle:
            handle.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            handle.write(np.flipud(pixels).tobytes())
        (destination / 'map.yaml').write_text(yaml.safe_dump({
            'image': 'map.pgm', 'mode': 'trinary', 'resolution': grid.resolution,
            'origin': [grid.origin_x, grid.origin_y, grid.origin_yaw],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25,
        }), encoding='utf-8')
    result = dict(result, map_yaml=str(destination / 'map.yaml') if grid is not None else None)
    temporary = destination / 'result.json.tmp'
    temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(destination / 'result.json')
    return str(destination / 'result.json')
