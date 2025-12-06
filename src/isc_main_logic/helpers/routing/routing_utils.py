from collections import defaultdict
from typing import Dict, List, Tuple
import json, heapq

Adj = Dict[str, List[Tuple[str, int]]]

def load_edges(edges_path: str) -> Dict[Tuple[str, str], int]:
    with open(edges_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    edges: Dict[Tuple[str, str], int] = {}
    for e in data:
        u, v, w = e["u"], e["v"], int(e["w"])
        edges[(u, v)] = w
    return edges

def load_zones(path: str) -> List[str]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def build_graph(edges: Dict[Tuple[str, str], int]) -> Adj:
    g: Adj = defaultdict(list)
    for (u, v), d in edges.items():
        g[u].append((v, d))
        #undirected behavior, keep this line.
        #directed behavior, REMOVE this line.
        if (v, u) not in edges:
            g[v].append((u, d))
    return g

def dijkstra(source: str, zones: List[str], graph: Adj) -> Dict[str, int]:
    heap = [(0, source)]
    dist = {z: float("inf") for z in zones}
    dist[source] = 0
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        for v, w in graph.get(u, []):
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return dist

def all_pairs_travel_time(zones: List[str], edges: Dict[Tuple[str, str], int]) -> Dict[Tuple[str, str], int]:
    g = build_graph(edges)
    travel: Dict[Tuple[str, str], int] = {}
    for z in zones:
        shortest = dijkstra(z, zones, g)
        for zz in zones:
            travel[(z, zz)] = shortest[zz]
    return travel