from typing import List, Tuple
from collections import defaultdict

from isc_main_logic.settings import ROBOT_DIMENSIONS

def runs_needed_2d(item_dims: List[Tuple[int, int]], L=ROBOT_DIMENSIONS["l"], W=ROBOT_DIMENSIONS["w"]) -> int:
    """
    Shelf packing with L vertical (long side) and W horizontal (short side).
    Each item is (length_mm, width_mm). Rotation allowed per item.

    - Rows (shelves) are horizontal bands stacked along L.
    - row['band_L'] is the shelf thickness along L (fixed by first item in that row).
    - row['used_W'] is the  horizontal width already consumed in that row along W.
    """
    
    if not item_dims:
        return 0

    # Place big items first (by larger side)
    items = sorted(item_dims, key=lambda wh: max(wh), reverse=True)

    robots = 1
    rows = []           # each row: {'band_L': <mm>, 'used_W': <mm>}
    used_L = 0          # total vertical thickness used by rows on current robot

    for w, l in items:
        placed = False

        # 1) Try existing rows (first-fit), test both orientations → (iL, iW)
        for row in rows:
            for iL, iW in ((l, w), (w, l)):
                if iL <= row['band_L'] and row['used_W'] + iW <= W:
                    row['used_W'] += iW
                    placed = True
                    break
            if placed:
                break

        # 2) Try to open a new row on current robot
        if not placed:
            opened = False
            for iL, iW in ((l, w), (w, l)):
                if iW <= W and used_L + iL <= L:
                    rows.append({'band_L': iL, 'used_W': iW})
                    used_L += iL
                    opened = True
                    placed = True
                    break

            # 3) Need a new robot/deck
            if not opened:
                robots += 1
                rows = []
                used_L = 0
                for iL, iW in ((l, w), (w, l)):
                    if iW <= W and iL <= L:
                        rows.append({'band_L': iL, 'used_W': iW})
                        used_L = iL
                        placed = True
                        break
                    
    return robots

