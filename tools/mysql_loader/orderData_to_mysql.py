import csv, os
import pymysql

from datetime import datetime, date
from pathlib import Path
from dotenv import load_dotenv

# =========================================
# DB CONFIG 
# =========================================
ROOT = Path(__file__).resolve().parents[2]
load_dotenv(ROOT / ".env")

DB_HOST = os.getenv("DB_HOST")
DB_PORT = int(os.getenv("DB_PORT"))
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")

# =========================================
# CSV CONFIG 
# =========================================
BASE_DIR = Path(__file__).resolve().parent
CSV_PATH = BASE_DIR / "iSC_DataForMySQL.csv"

# load everything, set TARGET_DATE = None, else date(2024, 12, 1)
TARGET_DATE = None

CSV_COL_ORDER_ID        = "Order number"
CSV_COL_ORDER_DATETIME  = "Order Date-time"
CSV_COL_FLIGHT_DATETIME = "Flight Date-time"
CSV_COL_MERCHANT_CODE   = "Merchant ID"
CSV_COL_MERCHANT_NAME   = "Merchant"
CSV_COL_TERMINAL_CODE   = "Terminal"
CSV_COL_ITEM_QTY        = "Quantity"
CSV_COL_ITEM_DESC       = "ItemDetails"
CSV_COL_IS_COMPLETE     = "Is Complete"

# e.g. "2024-12-01T10:30:00"
CSV_DATETIME_FORMAT = "%Y-%m-%dT%H:%M:%S"

# =========================================
# HELPERS
# =========================================

def parse_bool_like(value):
    """Convert CSV 'Is Complete' values into 0/1."""
    if value is None:
        return 0
    s = str(value).strip().lower()
    if s in ("1", "true", "yes", "y", "done", "complete"):
        return 1
    return 0


def safe_int(value, default=0):
    try:
        return int(str(value).strip())
    except Exception:
        return default


def main():
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

    processed_rows = 0
    inserted_orders = 0

    with conn:
        with conn.cursor() as cur, open(CSV_PATH, newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)

            for row in reader:
                processed_rows += 1

                # --- 1. Parse and filter by date ---
                raw_order_dt = (row.get(CSV_COL_ORDER_DATETIME) or "").strip()
                if not raw_order_dt:
                    continue

                try:
                    order_dt = datetime.strptime(raw_order_dt, CSV_DATETIME_FORMAT)
                except ValueError:
                    continue

                if TARGET_DATE is not None and order_dt.date() != TARGET_DATE:
                    continue

                raw_flight_dt = (row.get(CSV_COL_FLIGHT_DATETIME) or "").strip()
                try:
                    flight_dt = datetime.strptime(raw_flight_dt, CSV_DATETIME_FORMAT)
                except ValueError:
                    continue

                # --- 2. Merchant upsert (MerchantData) ---
                merchant_code = (row.get(CSV_COL_MERCHANT_CODE) or "").strip()
                merchant_name = (row.get(CSV_COL_MERCHANT_NAME) or "").strip()

                if merchant_code:
                    cur.execute(
                        """
                        INSERT INTO MerchantData (MerchantCode, MerchantName)
                        VALUES (%s, %s)
                        ON DUPLICATE KEY UPDATE MerchantName = VALUES(MerchantName)
                        """,
                        (merchant_code, merchant_name),
                    )

                # --- 3. iSCOrderData insert/upsert ---
                order_id      = (row.get(CSV_COL_ORDER_ID) or "").strip()
                terminal_code = safe_int(row.get(CSV_COL_TERMINAL_CODE))
                item_qty      = safe_int(row.get(CSV_COL_ITEM_QTY))
                is_complete   = parse_bool_like(row.get(CSV_COL_IS_COMPLETE))
                item_desc     = (row.get(CSV_COL_ITEM_DESC) or "").strip()

                if not order_id:
                    continue

                cur.execute(
                    """
                    INSERT INTO iSCOrderData
                      (OrderID, OrderDateTime, FlightDateTime,
                       MerchantCode, TerminalCode, ItemQty,
                       ItemDescription, IsComplete)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                    ON DUPLICATE KEY UPDATE
                      OrderDateTime   = VALUES(OrderDateTime),
                      FlightDateTime  = VALUES(FlightDateTime),
                      MerchantCode    = VALUES(MerchantCode),
                      TerminalCode    = VALUES(TerminalCode),
                      ItemQty         = VALUES(ItemQty),
                      ItemDescription = VALUES(ItemDescription),
                      IsComplete      = VALUES(IsComplete)
                    """,
                    (
                        order_id,
                        order_dt,
                        flight_dt,
                        merchant_code,
                        terminal_code,
                        item_qty,
                        item_desc,
                        is_complete,
                    ),
                )
                inserted_orders += 1

        conn.commit()

    print(f"Processed {processed_rows} CSV rows.")
    if TARGET_DATE is None:
        print(f"Inserted/updated {inserted_orders} orders (all dates).")
    else:
        print(f"Inserted/updated {inserted_orders} orders for {TARGET_DATE}.")


if __name__ == "__main__":
    main()


# python3 src/csv_loader/orderData_to_mysql.py

# SELECT * FROM iSCOrderData
# WHERE DATE(OrderDateTime) = '2024-12-01';