from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from typing import Dict, List

from ...helpers.utils import _normalize_home
from ...helpers.routing.routing_utils import load_edges, load_zones, all_pairs_travel_time


def _maps_dir() -> Path:
    share = Path(get_package_share_directory("isc_task_optimizer"))
    return share / "optimizer_logic" / "helpers" / "routing" / "maps"

MERCHANT_ZONES_PATH = _maps_dir() / "merchant_zones.json"
AIRPORT_EDGES_PATH  = _maps_dir() / "airport_edges.json"
AIRPORT_ZONES_PATH  = _maps_dir() / "airport_zones.json"

def compute_distances(
        summary_by_merchant: Dict[str, Dict],
        merchant_zones_path = MERCHANT_ZONES_PATH,
        edges_path = AIRPORT_EDGES_PATH,
        zones_path = AIRPORT_ZONES_PATH,
        ) -> Dict[str, Dict]:

    merchant_home_map = load_zones(merchant_zones_path)
    zones = load_zones(zones_path)
    edges = load_edges(edges_path)
    travel = all_pairs_travel_time(zones, edges)

    def dist(u: str, v: str) -> int:
        if (u, v) in edges:
            return int(edges[(u, v)])
        if (v, u) in edges:
            return int(edges[(v, u)])
        d = travel.get((u, v), float("inf"))
        return int(d) if d != float("inf") else 0

    out: Dict[str, Dict] = {}

    for m, info in summary_by_merchant.items():
        robots_by_t: Dict[str, int] = info.get("robots", {})

        hvals = _normalize_home(merchant_home_map.get(m))
        home = hvals[0] if hvals else None
        
        dur_by_terminal = {"T1": 0, "T2": 0, "T3": 0}
        if home:
            for t in ["T1", "T2", "T3"]:
                dur_by_terminal[t] = dist(home, t)

        if not home is None:
            out[m] = {
                "home": home,
                "dur_by_terminal": dur_by_terminal,
            }
            continue

        counts: Dict[str, int] = {
            "T1": robots_by_t.get("1", 0),
            "T2": robots_by_t.get("2", 0),
            "T3": robots_by_t.get("3", 0),
        }

        others = [t for t in ["T1", "T2", "T3"] if t != home and counts.get(t, 0) > 0]
        others.sort(key=lambda t: (dur_by_terminal[t], t))

        out[m] = {
            "home": home,
            "dur_by_terminal": dur_by_terminal, 
        }

    return out
