#!/usr/bin/env python3
"""Remove isolated occupied components from a candidate Nav2 PGM map."""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image


def remove_small_occupied_components(
        image: np.ndarray, *, min_component_size: int = 5,
        occupied_pixel_threshold: int = 64,
        connectivity: int = 8) -> tuple[np.ndarray, int]:
    """Return a copy with occupied components smaller than the threshold removed."""
    if image.ndim != 2:
        raise ValueError('the PGM image must be single-channel')
    if min_component_size < 1 or not 0 <= occupied_pixel_threshold <= 255:
        raise ValueError('invalid component or pixel threshold')
    if connectivity not in (4, 8):
        raise ValueError('connectivity must be 4 or 8')
    occupied = image <= occupied_pixel_threshold
    visited = np.zeros_like(occupied, dtype=bool)
    output = image.copy()
    if connectivity == 4:
        neighbors = ((-1, 0), (1, 0), (0, -1), (0, 1))
    else:
        neighbors = ((-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1))
    removed = 0
    height, width = occupied.shape
    for row, col in zip(*np.where(occupied)):
        if visited[row, col]:
            continue
        queue = deque([(int(row), int(col))])
        visited[row, col] = True
        cells = []
        while queue:
            current_row, current_col = queue.popleft()
            cells.append((current_row, current_col))
            for drow, dcol in neighbors:
                next_row = current_row + drow
                next_col = current_col + dcol
                if (0 <= next_row < height and 0 <= next_col < width
                        and occupied[next_row, next_col]
                        and not visited[next_row, next_col]):
                    visited[next_row, next_col] = True
                    queue.append((next_row, next_col))
        if len(cells) < min_component_size:
            removed += len(cells)
            for current_row, current_col in cells:
                output[current_row, current_col] = 254
    return output, removed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_pgm', type=Path)
    parser.add_argument('output_pgm', type=Path)
    parser.add_argument('--min-component-size', type=int, default=5)
    parser.add_argument('--occupied-pixel-threshold', type=int, default=64)
    parser.add_argument('--connectivity', type=int, choices=(4, 8), default=8)
    args = parser.parse_args()
    image = np.asarray(Image.open(args.input_pgm).convert('L'))
    cleaned, removed = remove_small_occupied_components(
        image,
        min_component_size=args.min_component_size,
        occupied_pixel_threshold=args.occupied_pixel_threshold,
        connectivity=args.connectivity)
    args.output_pgm.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cleaned, mode='L').save(args.output_pgm)
    print(f'removed {removed} occupied cells; wrote {args.output_pgm}')


if __name__ == '__main__':
    main()
