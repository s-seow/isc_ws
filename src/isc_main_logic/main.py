from collections import defaultdict
from datetime import timedelta
from typing import Dict, List
from pathlib import Path
import csv, sys, os, pymysql

sys.dont_write_bytecode = True

from isc_main_logic.helpers.logic.robot_2d_packing_logic import runs_needed_2d
from isc_main_logic.helpers.logic.robot_3d_packing_logic import runs_needed_3d
from isc_main_logic.helpers.logic.robot_assignment_logic import pack_same_time  
from isc_main_logic.helpers.utils import to_int
from isc_main_logic.helpers.dates_utils import _parse_ddmmyyyy, _fmt_ddmmyyyy, _parse_flight_date
from isc_main_logic.helpers.routing.compute_merchant_distances import compute_distances

from isc_main_logic.settings import LOW_TIME_WINDOW, HIGH_TIME_WINDOW, HIGH_WINDOW_MERCHANTS, robot_dimensions
from dotenv import load_dotenv

ROOT = Path(__file__).resolve().parents[2]
load_dotenv(ROOT / ".env")

DB_HOST = os.getenv("DB_HOST")
DB_PORT = int(os.getenv("DB_PORT"))
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")

def run_logic(target_date: str) -> Dict[str, Dict[str, dict]]:

    # ---------- helpers ----------
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

    def _fetch_rows(query: str, params: tuple) -> List[dict]:
        with conn.cursor() as cur:
            cur.execute(query, params)
            return list(cur.fetchall())

    def _make_order_from_row(row, fallback_order_date, date_str_for_label):
        terminal = (row.get("Terminal") or "").strip()
        if terminal not in {"1", "2", "3"}:
            return None

        merchant = (row.get("Merchant") or "").strip() or "UNKNOWN"
        qty = to_int(row.get("Quantity"), 1)
        w = to_int(row.get("width_mm"), 0)
        l = to_int(row.get("length_mm"), 0)
        h = to_int(row.get("height_mm"), 0)
        if not w or not l or qty <= 0:
            return None

        order_num = (row.get("Order number") or "").strip()
        order_date = _parse_flight_date(row.get("Order date"), fallback_order_date)
        flight_date = _parse_flight_date(row.get("Flight date"), fallback_order_date)

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

    # ---------- DB load order helpers ---------- 
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

    def _load_orders_prior_plus2(day) -> List[dict]:
        """
        PP2: Include prior orders where flight ∈ {D, D+1}
        (OrderDateTime date < D, FlightDateTime date in {D, D+1})
        """
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
    

    def _summarise_selected_by_merchant(selected_orders: List[dict]) -> Dict[str, Dict[str, dict]]:
        """
        Compute runs_by_t via 2D shelf packing (per terminal).
        Track qty_by_t (counts of items per terminal).
        """
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
                #dims_by_mt[m][t].append((o["w"], o["l"]))  # 2D
                dims_by_mt[m][t].append((o["w"], o["l"], o["h"]))

        summary: Dict[str, Dict[str, dict]] = {}
        for m in sorted(dims_by_mt.keys()):
            runs_by_t: Dict[str, int] = {}
            for t in ["1", "2", "3"]:
                item_dims = dims_by_mt[m].get(t, [])
                runs_by_t[t] = (
                    #runs_needed_2d(item_dims, L=robot_dimensions["l"], W=robot_dimensions["w"])
                    runs_needed_3d(item_dims, L=robot_dimensions["l"], W=robot_dimensions["w"], H=robot_dimensions["h"])
                    if item_dims
                    else 0
                )

            summary[m] = {
                "orders": orders_count[m],
                "items": items_count[m],
                "runs": runs_by_t,                       
                "robots_max": max(runs_by_t.values()) if runs_by_t else 0,
                #"qty_by_t": {t: int(qty_by_mt[m].get(t, 0)) for t in ["1", "2", "3"]},
            }
        return summary
    
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

    def _print_day_summary_by_merchant(day, summary_by_merchant, plans=None):
        print(f"\n=== {day.isoformat()} ===")
        if not summary_by_merchant:
            print("\n(no orders serviced)\n")
            return

        skipped = 0
        for merchant in sorted(summary_by_merchant.keys()):
            plan = (plans or {}).get(merchant, {})
            home = plan.get("home")
            if not home:
                skipped += 1
                continue

            info = summary_by_merchant[merchant]
            r1 = info["runs"].get("1", 0)
            r2 = info["runs"].get("2", 0)
            r3 = info["runs"].get("3", 0)

            print(f"\nMerchant: {merchant} ({home})")
            #print(f"  -> no. of orders : {info['orders']}")
            #print(f"  -> no. of items  : {info['items']}")

            # (A) 2D shelf metric
            print(f"  -> item qty in terms of runs (T1/T2/T3): {r1}/{r2}/{r3}")

            use_high = merchant.strip().lower() in HIGH_WINDOW_MERCHANTS
            capacity = HIGH_TIME_WINDOW if use_high else LOW_TIME_WINDOW

            # (B) Time-window metric (no-mix), built FROM the 2D result
            # Convert each terminal’s 2D-robots count into a duration item.
            runs_by_t = info["runs"]
            dur_items: List[int] = []
            dur_by_t = plan.get("dur_by_terminal", {})  

            if runs_by_t.get("1", 0):
                w1 = int(dur_by_t.get("T1", 0))
                dur_items += [2*w1] * runs_by_t["1"]
            if runs_by_t.get("2", 0):
                w2 = int(dur_by_t.get("T2", 0))
                dur_items += [2*w2] * runs_by_t["2"]
            if runs_by_t.get("3", 0):
                w3 = int(dur_by_t.get("T3", 0))
                dur_items += [2*w3] * runs_by_t["3"]

            if dur_items:
                bins = pack_same_time(dur_items, capacity=capacity)

                per_t_bins = _count_bins_per_terminal(bins, dur_by_t)

                print(f"  -> time-window packing (capacity={capacity})")
                for i, b in enumerate(bins, 1):
                    print(f"        Robot {i}: {b} = {sum(b)} / {capacity}")

                print(f"  -> total robots needed to fulfill all runs: {len(bins)}")
                print(f"        By terminal: T1={per_t_bins['T1']}, T2={per_t_bins['T2']}, T3={per_t_bins['T3']}")
            else:
                print(f"  -> time-window packing: robots=0 (no durations)")

            #total = (plans or {}).get(merchant, {}).get("total", 0)
            #print(f"  -> total travel time for 1 robot (route planner): {total}")

        if skipped:
            print(f"\n(hidden {skipped} merchants not part of iSC trials)")
 

    # ---------- interactive loop ----------
    day = _parse_ddmmyyyy(target_date)
    pending: List[dict] = []
    served_ids = set()
    last_day_summary: Dict[str, Dict[str, dict]] = {}

    while True:
        date_key = _fmt_ddmmyyyy(day)

        # LOFOD: Load today's new orders (Order date = D)
        todays_new = _load_orders_for_order_date(date_key)
        if todays_new:
            print(f"\nLoaded {len(todays_new)} new orders for Order date (D): {date_key}.")
        else:
            print(f"\nNo new orders for Order date {date_key}.")

        # PP2: Include prior orders where flight ∈ {D, D+1}
        prior_plus2 = [o for o in _load_orders_prior_plus2(day) if o["id"] not in served_ids]
        if prior_plus2:
            print(f"Included {len(prior_plus2)} prior orders with flight in D/D+1.")

        if pending:
            print(f"Pending carried from previous day: {len(pending)}")

        # Merge & dedupe
        id_seen = {o["id"] for o in pending}
        for o in todays_new + prior_plus2:
            if o["id"] not in id_seen:
                pending.append(o)
                id_seen.add(o["id"])

        # Select to service (flight offset 0 or 1), carry others
        selected, carry = [], []
        for o in pending:
            offset = (o["flight_date"] - day).days
            if 0 <= offset <= 1:
                selected.append(o)
            else:
                carry.append(o)

        for o in selected:
            served_ids.add(o["id"])
            #print(f"  OK   - {o['label']}")

        # Summarize and print
        print(f"\nServiced {len(selected)} orders today.")
        last_day_summary = _summarise_selected_by_merchant(selected)

        # Compute plans to get 'home'
        plans = compute_distances(last_day_summary)
        _print_day_summary_by_merchant(day, last_day_summary, plans)

        # Backlog & next step
        if carry:
            next_day_str = _fmt_ddmmyyyy(day + timedelta(days=1))
            print(f"\nCarried forward {len(carry)} orders to next day {next_day_str}.")
        else:
            print("No backlog. All orders have appeared.")

        ans = input("\n>> Continue to next day? (Y/N): ").strip().lower()
        if ans not in {"y", "yes"}:
            break
        pending = carry
        day = day + timedelta(days=1)

    return last_day_summary


# ---------- NOTES ON METRICS ----------

# NEED: Number of runs per terminal - info["robots"]
# NEED: Number of robots per terminal - per_t_bins

# number of runs per terminal are calculated from 2D shelf packing (robots_needed_2d) of items into robot shelves
# number of robots per terminal are calculated from time-window packing (pack_same_time) of runs into time windows