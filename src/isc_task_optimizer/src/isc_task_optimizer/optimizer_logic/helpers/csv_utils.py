from pathlib import Path
import csv
from typing import List

def _open_writer(path: Path, headers: List[str]):
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    f = path.open("a", newline="", encoding="utf-8")
    w = csv.DictWriter(f, fieldnames=headers)
    if not exists:
        w.writeheader()
    return f, w
