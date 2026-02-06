# ==============================
# iSC Main Logic: run_logic()
# ==============================
#
# Purpose:
#   - Load orders from MySQL for a target day (D)
#   - Include qualifying prior orders whose flights are in {D, D+1}
#   - Filter invalid rows (missing terminal/dimensions/qty)
#   - Summarise per merchant:
#       (A) "runs" per terminal via 3D packing 
#       (B) "robots" per terminal via time-window bin packing
#   - Print a day-by-day report for debugging purpose

from collections import defaultdict
from datetime import timedelta
from typing import Dict, List
from pathlib import Path
import sys, os, pymysql

sys.dont_write_bytecode = True

# ----------------------------
# Packing and assignment helpers
# ----------------------------
from .helpers.logic.robot_3d_packing_logic import runs_needed_3d
from .helpers.logic.robot_assignment_logic import pack_same_time  
from .helpers.utils import to_int

# ----------------------------
# Date related helpers
# ----------------------------
from .helpers.dates_utils import _parse_ddmmyyyy, _fmt_ddmmyyyy, _parse_flight_date

# ----------------------------
# Routing helpers
# ----------------------------
from .helpers.routing.compute_merchant_distances import compute_distances

# ----------------------------
# Parameters and environment
# ----------------------------
from .main_logic_settings import LOW_TIME_WINDOW, HIGH_TIME_WINDOW, HIGH_WINDOW_MERCHANTS, ROBOT_DIMENSIONS
from dotenv import load_dotenv

for p in Path(__file__).resolve().parents:
    if (p / ".env").is_file():
        load_dotenv(p / ".env")
        break

# ----------------------------
# Database settings
# ----------------------------
DB_HOST = os.getenv("DB_HOST")
DB_PORT = int(os.getenv("DB_PORT"))
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")


def run_logic(target_date: str) -> Dict[str, Dict[str, dict]]:

    # ----------------------------
    # Initializes DB connection
    # ----------------------------
    # Uses a single connection for the duration of run_logic()
    conn = pymysql.connect(
        host=DB_HOST,
        port=DB_PORT,
        user=DB_USER,
        password=DB_PASSWORD,
        database=DB_NAME,
        charset="utf8mb4",
        cursorclass=pymysql.cursors.DictCursor,
        autocommit=False,
    )

    # ============================================================
    # Helper: Execute query and return dict rows
    # ============================================================
    def _fetch_rows(query: str, params: tuple) -> List[dict]:
        with conn.cursor() as cur:
            cur.execute(query, params)
            return list(cur.fetchall())


    # ============================================================
    # Helper: Convert DB row to order dict
    #
    # Validates:
    #   - Terminal in {1,2,3}
    #   - Dimensions are present (mm) and are non-zero
    #   - Quantity is positive integer
    #
    # Produces:
    #   - A Dict of orders with parsed dates and labels
    # ============================================================
    def _make_order_from_row(row, fallback_order_date, date_str_for_label):

        terminal = (row.get("Terminal") or "").strip()
        if terminal not in {"1", "2", "3"}:
            return None

        merchant = (row.get("Merchant") or "").strip() or "UNKNOWN"
        qty = to_int(row.get("Quantity"), 1)

        # Dimensions in mm from ItemDictionary
        w = to_int(row.get("width_mm"), 0)
        l = to_int(row.get("length_mm"), 0)
        h = to_int(row.get("height_mm"), 0)
        if not w or not l or qty <= 0:
            return None

        order_num = (row.get("Order number") or "").strip()
        order_date = _parse_flight_date(row.get("Order date"), fallback_order_date)
        flight_date = _parse_flight_date(row.get("Flight date"), fallback_order_date)

        # Label is for debug printing only
        label = (
            f"[{order_num}] {merchant} x {qty} | {(row.get('Details') or '').strip()} | "
            f"Order {date_str_for_label} | Flight {(row.get('Flight date') or '').strip()} "
            f"{(row.get('Flight time') or '').strip()} | T{terminal}"
        )
        return {
            "id": order_num or f"{merchant}|{order_date.isoformat()}|{flight_date.isoformat()}|{w}x{l}x{qty}",
            "terminal": terminal,
            "merchant": merchant,
            "w": w,
            "l": l,
            "h": h,
            "qty": qty,
            "order_date": order_date,
            "flight_date": flight_date,
            "label": label,
        }


    # ============================================================
    # DB Loader (LOFOD): Load orders where OrderDateTime == D
    #
    # SQL Joins:
    #   - iSCOrderData (orders table - for orders)
    #   - MerchantData (merchant table - for merchant name lookup)
    #   - ItemDictionary (item dimensions table - for dimensions by merchant+description)
    #
    # Returns:
    #   - A list of normalized orders Dicts, with invalid rows dropped
    # ============================================================
    def _load_orders_for_order_date(date_str: str) -> List[dict]:

        target = _parse_ddmmyyyy(date_str)
        orders: List[dict] = []

        query = """
        SELECT
          o.OrderID                                    AS `Order number`,
          DATE_FORMAT(o.OrderDateTime,  '%%e/%%c/%%Y')    AS `Order date`,
          DATE_FORMAT(o.FlightDateTime, '%%e/%%c/%%Y')    AS `Flight date`,
          TIME_FORMAT(o.FlightDateTime, '%%H:%%i:%%s')    AS `Flight time`,
          CAST(o.TerminalCode AS CHAR(1))              AS `Terminal`,
          COALESCE(m.MerchantName, o.MerchantCode)     AS `Merchant`,
          o.ItemQty                                    AS `Quantity`,
          o.ItemDescription                            AS `ItemDetails`,
          d.ItemWidthMM                                AS `width_mm`,
          d.ItemLengthMM                               AS `length_mm`,
          d.ItemHeightMM                               AS `height_mm`
        FROM iSCOrderData o
        LEFT JOIN MerchantData m
          ON m.MerchantCode = o.MerchantCode
        LEFT JOIN ItemDictionary d
          ON d.MerchantCode    = o.MerchantCode
         AND TRIM(d.ItemDescription) = TRIM(o.ItemDescription)
        WHERE DATE(o.OrderDateTime) = %s;
        """

        rows = _fetch_rows(query, (target,))
        label_date = _fmt_ddmmyyyy(target)

        for row in rows:
            o = _make_order_from_row(row, fallback_order_date=target, date_str_for_label=label_date)
            if o:
                orders.append(o)

        return orders


    # ============================================================
    # DB Loader (PP1): Load prior orders where flight ∈ {D, D+1}
    #
    # Include:
    #   - Prior orders where OrderDateTime date < D
    #   - FlightDateTime date in {D, D+1}
    #
    # Returns:
    #   - A list of normalized orders Dicts, with invalid rows dropped
    # ============================================================
    def _load_orders_prior_plus1(day) -> List[dict]:

        keep: List[dict] = []

        query = """
        SELECT
          o.OrderID                                    AS `Order number`,
          DATE_FORMAT(o.OrderDateTime,  '%%e/%%c/%%Y')    AS `Order date`,
          DATE_FORMAT(o.FlightDateTime, '%%e/%%c/%%Y')    AS `Flight date`,
          TIME_FORMAT(o.FlightDateTime, '%%H:%%i:%%s')    AS `Flight time`,
          CAST(o.TerminalCode AS CHAR(1))              AS `Terminal`,
          COALESCE(m.MerchantName, o.MerchantCode)     AS `Merchant`,
          o.ItemQty                                    AS `Quantity`,
          o.ItemDescription                            AS `ItemDetails`,
          d.ItemWidthMM                                AS `width_mm`,
          d.ItemLengthMM                               AS `length_mm`,
          d.ItemHeightMM                               AS `height_mm`
        FROM iSCOrderData o
        LEFT JOIN MerchantData m
          ON m.MerchantCode = o.MerchantCode
        LEFT JOIN ItemDictionary d
          ON d.MerchantCode    = o.MerchantCode
         AND TRIM(d.ItemDescription) = TRIM(o.ItemDescription)
        WHERE DATE(o.OrderDateTime) < %s
          AND DATE(o.FlightDateTime) IN (%s, %s);
        """

        rows = _fetch_rows(query, (day, day, day + timedelta(days=1)))

        for row in rows:
            od_str = (row.get("Order date") or "").strip()
            fallback_od = _parse_flight_date(od_str, fallback_date=day)
            o = _make_order_from_row(row, fallback_od, od_str)
            if o:
                keep.append(o)

        return keep
    

    # ============================================================
    # Aggregation: Summarise selected orders by merchant
    #
    # Output per each merchant:
    #   - orders: the number of order rows that were selected for servicing
    #   - items: the total number of items (sum of qty) across these orders
    #   - runs: runs_needed_3d, a Dict of runs needed per terminal {"1","2","3"} (via 3D packing)
    #
    # Important Note:
    #   - runs_needed_3d is derived from packing items into shelves in the robot
    # ============================================================
    def _summarise_selected_by_merchant(selected_orders: List[dict]) -> Dict[str, Dict[str, dict]]:

        dims_by_mt = defaultdict(lambda: defaultdict(list))
        qty_by_mt = defaultdict(lambda: defaultdict(int))
        orders_count = defaultdict(int)
        items_count = defaultdict(int)

        for o in selected_orders:
            m, t = o["merchant"], o["terminal"]
            orders_count[m] += 1
            items_count[m] += o["qty"]
            qty_by_mt[m][t] += o["qty"]
            for _ in range(o["qty"]):
                #dims_by_mt[m][t].append((o["w"], o["l"]))  # 2D old logic
                dims_by_mt[m][t].append((o["w"], o["l"], o["h"]))

        summary: Dict[str, Dict[str, dict]] = {}
        for m in sorted(dims_by_mt.keys()):
            runs_by_t: Dict[str, int] = {}
            for t in ["1", "2", "3"]:
                item_dims = dims_by_mt[m].get(t, [])
                runs_by_t[t] = (
                    #runs_needed_2d(item_dims, L=ROBOT_DIMENSIONS["l"], W=ROBOT_DIMENSIONS["w"]) # 2D old logic
                    runs_needed_3d(item_dims, L=ROBOT_DIMENSIONS["l"], W=ROBOT_DIMENSIONS["w"], H=ROBOT_DIMENSIONS["h"])
                    if item_dims
                    else 0
                )

            summary[m] = {
                "orders": orders_count[m],
                "items": items_count[m],
                "runs": runs_by_t,                       
                #"robots_max": max(runs_by_t.values()) if runs_by_t else 0,             # Not used: debug variable
                #"qty_by_t": {t: int(qty_by_mt[m].get(t, 0)) for t in ["1", "2", "3"]}, # Not used: debug variable
            }
        return summary
    

    # ============================================================
    # Helper: Convert packed bins into counts for each terminal
    #
    # Input:
    #   - bins: Output of pack_same_time, each bin contains a duration of "items"
    #   - dur_by_t: Duration it takes to travel to and fro for each terminal
    # 
    # Output:
    #   - A Dict with counts of bins per terminal {"T1": int, "T2": int, "T3": int}, used in Optimizer Node
    # ============================================================
    def _count_bins_per_terminal(bins: List[List[int]], dur_by_t: Dict[str, int]) -> Dict[str, int]:
        per_t_bins = {"T1": 0, "T2": 0, "T3": 0}
        dur_to_term = {}
        for term_key in ["T1", "T2", "T3"]:
            w = int(dur_by_t.get(term_key, 0))
            if w > 0:
                dur_to_term[2 * w] = term_key

        for b in bins:
            if not b:
                continue
            d = b[0]
            t = dur_to_term.get(d)
            if t:
                per_t_bins[t] += 1
        return per_t_bins
    

    # ============================================================
    # Metric: Compute robots needed per terminal (per merchant)
    #
    # Purpose:
    #   - Converts "runs" (from 3D shelf packing) into "robots" (time-window bins)
    #   - Returns structured metrics for RMF / ROS2 publisher use via /isc/merchant_summary topic
    #
    # Output per merchant:
    #   - home
    #   - runs_by_terminal: {"T1","T2","T3"}
    #   - robots_by_terminal: {"T1","T2","T3"}
    #   - robots_total
    # ============================================================
    def _compute_robots_by_terminal(
            
        summary_by_merchant: Dict[str, Dict[str, dict]],
        plans: dict
        ) -> Dict[str, dict]:

        robots_by_merchant: Dict[str, dict] = {}

        for merchant, info in summary_by_merchant.items():
            plan = (plans or {}).get(merchant, {})
            home = plan.get("home")
            if not home:
                continue

            use_high = merchant.strip().lower() in HIGH_WINDOW_MERCHANTS
            merchant_window_capacity = HIGH_TIME_WINDOW if use_high else LOW_TIME_WINDOW

            runs_by_t = info["runs"]                    # keys: "1","2","3"
            dur_by_t = plan.get("dur_by_terminal", {})  # keys: "T1","T2","T3"

            # Convert each run into one duration item and pack_same_time enforces no mixing.
            dur_items: List[int] = []
            if runs_by_t.get("1", 0):
                dur_items += [2 * int(dur_by_t.get("T1", 0))] * runs_by_t["1"]
            if runs_by_t.get("2", 0):
                dur_items += [2 * int(dur_by_t.get("T2", 0))] * runs_by_t["2"]
            if runs_by_t.get("3", 0):
                dur_items += [2 * int(dur_by_t.get("T3", 0))] * runs_by_t["3"]

            if dur_items:
                bins = pack_same_time(dur_items, merchant_window_capacity)
                per_t_bins = _count_bins_per_terminal(bins, dur_by_t)
                robots_total = len(bins)
            else:
                per_t_bins = {"T1": 0, "T2": 0, "T3": 0}
                robots_total = 0
                bins = []

            robots_by_merchant[merchant] = {
                "home": home, 
                "merchant_window_capacity": merchant_window_capacity, 
                # "dur_by_terminal": {
                #     "T1": int(dur_by_t.get("T1", 0)),  
                #     "T2": int(dur_by_t.get("T2", 0)),
                #     "T3": int(dur_by_t.get("T3", 0)),
                # },
                "runs_by_terminal": {
                    "T1": runs_by_t.get("1", 0), 
                    "T2": runs_by_t.get("2", 0), 
                    "T3": runs_by_t.get("3", 0), 
                },
                "robots_by_terminal": {
                    "T1": per_t_bins["T1"],
                    "T2": per_t_bins["T2"],
                    "T3": per_t_bins["T3"],
                },
                "robots_total": robots_total, 
                # "bins": bins, 
            }

        return robots_by_merchant


    # ============================================================
    # Debug Printer: Prints a day summary by each merchant
    #
    # Note:
    #   - This version NO LONGER computes bins/robots.
    #   - It prints from precomputed robots_by_merchant (as the single source of truth).
    # 
    # Prints:
    #   (a) item qty in terms of runs per terminal (T1/T2/T3)
    #   (b) how the packing behind the time-window bins per robot was derived
    #   (c) total robots needed, then how this was derived by printing robots per terminal
    # ============================================================
    def _print_day_summary_by_merchant(day, summary_by_merchant, plans=None, robots_by_merchant=None):

        print(f"\n=== {day.isoformat()} ===")
        if not summary_by_merchant:
            print("\n(no orders serviced)\n")
            return

        if robots_by_merchant is None:
            robots_by_merchant = {}

        skipped = 0
        for merchant in sorted(summary_by_merchant.keys()):
            plan = (plans or {}).get(merchant, {})
            home = plan.get("home")
            if not home:
                skipped += 1
                continue

            info = summary_by_merchant[merchant]

            item_qty_for_run_t1 = info["runs"].get("1", 0)
            item_qty_for_run_t2 = info["runs"].get("2", 0)
            item_qty_for_run_t3 = info["runs"].get("3", 0)

            metrics = robots_by_merchant.get(merchant, {})

            merchant_window_capacity = int(metrics.get("merchant_window_capacity", 0))
            bins = metrics.get("bins", []) or []
            robots_total = int(metrics.get("robots_total", 0))
            robots_by_t = metrics.get("robots_by_terminal", {"T1": 0, "T2": 0, "T3": 0})

            print()
            print("┌" + "─" * 60)
            print(f"│ Merchant: {merchant}")
            print(f"│   Home: {home}")
            print("│")

            print(
                "│   Runs by terminal (item_qty_for_run_t): "
                f"T1={item_qty_for_run_t1}, T2={item_qty_for_run_t2}, T3={item_qty_for_run_t3}"
            )

            if not metrics:
                print("│   Merchant window capacity: (missing metrics)")
                print("│   Time-window bins (debug): (missing metrics)")
                print("│   Robots by terminal (robots_by_terminal): T1=0, T2=0, T3=0")
                print("│   Total robots needed for all runs: 0")
                print("└" + "─" * 60)
                continue

            print(f"│   Merchant window capacity: {merchant_window_capacity}")

            print("│   Time-window bins (debug):")
            if bins:
                for i, b in enumerate(bins, 1):
                    prefix = "│     ├─" if i < len(bins) else "│     └─"
                    print(f"{prefix} Robot {i}: {b} = {sum(b)} / {merchant_window_capacity}")
            else:
                print("│     └─ (no bins)")

            print("│")
            print(
                "│   Robots by terminal (robots_by_terminal): "
                f"T1={robots_by_t.get('T1', 0)}, T2={robots_by_t.get('T2', 0)}, T3={robots_by_t.get('T3', 0)}"
            )
            print(f"│   Total robots needed for all runs: {robots_total}")
            print("└" + "─" * 60)

        if skipped:
            print(f"\n(hidden {skipped} merchants not part of iSC trials)")


    # ============================================================
    # Driver: Main processing logic for a single day
    #
    # Pipeline:
    #   1) Load LOFOD orders (OrderDate = D)
    #   2) Load PP1 orders (OrderDate < D and FlightDate ∈ {D, D+1})
    #   3) Dedupe orders within this run
    #   4) Select serviceable orders (FlightDate ∈ {D, D+1})
    #   5) Summarise selected orders by merchant and print report
    # ============================================================
    day = _parse_ddmmyyyy(target_date)
    date_key = _fmt_ddmmyyyy(day)

    # Calls DB Loader (LOFOD)
    todays_new = _load_orders_for_order_date(date_key)
    if todays_new:
        print(f"\nLoaded {len(todays_new)} new orders for Order date (D): {date_key}.")
    else:
        print(f"\nNo new orders for Order date {date_key}.")

    # Calls DB Loader (PP1)
    prior_plus1 = _load_orders_prior_plus1(day)  
    if prior_plus1:
        print(f"Included {len(prior_plus1)} prior orders with flight in D/D+1.")

    # Dedupe for today's processing
    combined = []
    seen_ids = set()
    for o in todays_new + prior_plus1:
        if o["id"] not in seen_ids:
            combined.append(o)
            seen_ids.add(o["id"])

    # Select orders to service (flight offset 0 or 1)
    selected = []
    for o in combined:
        offset = (o["flight_date"] - day).days
        if 0 <= offset <= 1:
            selected.append(o)

    print(f"\nServiced {len(selected)} orders today.")
    
    runs_summary_by_merchant = _summarise_selected_by_merchant(selected)

    routing_plan_by_merchant = compute_distances(runs_summary_by_merchant)

    robots_summary_by_merchant = _compute_robots_by_terminal(
        runs_summary_by_merchant, 
        routing_plan_by_merchant 
    )

    _print_day_summary_by_merchant(day, runs_summary_by_merchant, routing_plan_by_merchant, robots_summary_by_merchant)

    return robots_summary_by_merchant
