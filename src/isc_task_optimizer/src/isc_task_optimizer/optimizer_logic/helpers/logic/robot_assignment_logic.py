from collections import Counter
import math

def pack_same_time(items, capacity, order=(120, 60, 30)):
    bins = []
    cnt = Counter(items)

    for d in order:
        n = cnt.get(d, 0)
        if n == 0:
            continue
        per_robot = capacity // d
        if per_robot == 0:
            raise ValueError(f"Item {d} does not fit into capacity {capacity}")
        full, rem = divmod(n, per_robot)
        # full robots
        bins.extend([[d] * per_robot for _ in range(full)])
        # leftover robot (partial), still same-time only
        if rem:
            bins.append([d] * rem)

    for d, n in cnt.items():
        if d in order or n == 0:
            continue
        per_robot = capacity // d
        if per_robot == 0:
            raise ValueError(f"Item {d} does not fit into capacity {capacity}")
        full, rem = divmod(n, per_robot)
        bins.extend([[d] * per_robot for _ in range(full)])
        if rem:
            bins.append([d] * rem)

    return bins
