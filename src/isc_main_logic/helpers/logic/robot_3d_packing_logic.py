from typing import List, Tuple
from itertools import permutations

from isc_main_logic.settings import robot_dimensions

def runs_needed_3d(item_dims_3d: List[Tuple[int, int, int]], L=robot_dimensions["l"], W=robot_dimensions["w"], H=robot_dimensions["h"]) -> int:
    """
    3D shelf packing with layers stacked along H (height).
    Inside each layer, pack rows along L and fill across W (rotation allowed).
    Returns the number of robots (containers) required.
    """
    if not item_dims_3d:
        return 0

    # Place big items first (by volume, then longest edge)
    items = sorted(item_dims_3d, key=lambda x: (x[0]*x[1]*x[2], max(x)), reverse=True)

    robots = 1
    layers = []          # each layer: {'band_H': <mm>, 'used_L': <mm>, 'rows': [{'band_L': <mm>, 'used_W': <mm>}, ...]}
    used_H = 0           # total height used on current robot

    for a, b, c in items:
        placed = False

        # rotate to try all 6 orientations
        orients = list({tuple(o) for o in permutations((a, b, c), 3)})
        orients.sort(key=lambda t: (t[2], -max(t[0], t[1])))

        # 1) Try to place in existing layers
        for l, w, h in orients:
            if l > L or w > W or h > H:
                continue
            for layer in layers:
                if h <= layer['band_H']:
                    # Try existing rows (first-fit)
                    for row in layer['rows']:
                        if l <= row['band_L'] and row['used_W'] + w <= W:
                            row['used_W'] += w
                            placed = True
                            break
                    if placed:
                        break
                    # Open a new row in this layer
                    if w <= W and layer['used_L'] + l <= L:
                        layer['rows'].append({'band_L': l, 'used_W': w})
                        layer['used_L'] += l
                        placed = True
                        break
            if placed:
                break

        # 2) Open a new layer on the current robot
        if not placed:
            for l, w, h in orients:
                if l <= L and w <= W and h <= H and used_H + h <= H:
                    layers.append({'band_H': h, 'used_L': l, 'rows': [{'band_L': l, 'used_W': w}]})
                    used_H += h
                    placed = True
                    break

        # 3) Need a new robot
        if not placed:
            robots += 1
            layers = []
            used_H = 0
            for l, w, h in orients:
                if l <= L and w <= W and h <= H:
                    layers.append({'band_H': h, 'used_L': l, 'rows': [{'band_L': l, 'used_W': w}]})
                    used_H = h
                    placed = True
                    break

        if not placed:
            raise ValueError(f"Item {a,b,c} cannot fit into {L}x{W}x{H} in any orientation.")

    return robots