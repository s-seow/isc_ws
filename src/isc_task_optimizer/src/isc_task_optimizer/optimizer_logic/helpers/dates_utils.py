from datetime import datetime
    
def _fmt_ddmmyyyy(d):
        # e.g., 10 Jan 2025 -> "10/1/2025"
        return f"{d.day}/{d.month}/{d.year}"

def _parse_ddmmyyyy(s: str):
    # Expect "D/M/YYYY"
    d, m, y = map(int, s.strip().split("/"))
    return datetime(y, m, d).date()

def _parse_flight_date(s: str, fallback_date):
    s = (s or "").strip()
    for fmt in ("%d/%m/%Y", "%Y-%m-%d", "%d-%m-%Y", "%m/%d/%Y"):
        try:
            return datetime.strptime(s, fmt).date()
        except Exception:
            pass
    return fallback_date