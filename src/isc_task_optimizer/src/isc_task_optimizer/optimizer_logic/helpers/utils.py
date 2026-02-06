from typing import List
from pathlib import Path
import sys, os, shutil, re
from datetime import datetime

def to_int(s, default=0) -> int:
    try:
        return int(float(s))
    except Exception:
        return default
    
def _normalize_home(home_val) -> List[str]:
        if home_val is None:
            return []
        if isinstance(home_val, str):
            return [home_val.strip()]
        if isinstance(home_val, list):
            return [str(x).strip() for x in home_val if str(x).strip()]
        return []
