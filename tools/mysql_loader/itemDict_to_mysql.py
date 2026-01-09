import csv, os
import pymysql

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
CSV_PATH = BASE_DIR / "iSC_ItemDict.csv"   

CSV_COL_MERCHANT_CODE   = "Merchant ID"
CSV_COL_MERCHANT_NAME   = "Merchant"
CSV_COL_ITEM_DESC       = "ItemDetails"
CSV_COL_ITEM_WIDTH_MM   = "width_mm"
CSV_COL_ITEM_LENGTH_MM  = "length_mm"
CSV_COL_ITEM_HEIGHT_MM  = "height_mm"

# =========================================
# HELPERS
# =========================================
def safe_int(value, default=0):
    try:
        return int(str(value).strip())
    except Exception:
        return default
    
def safe_float(value, default=0.0):
    try:
        return float(str(value).strip())
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

    processed = 0
    inserted = 0

    with conn:
        with conn.cursor() as cur, open(CSV_PATH, newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames:
                reader.fieldnames = [h.strip() for h in reader.fieldnames]

            for row in reader:
                processed += 1

                merchant_code = (row.get(CSV_COL_MERCHANT_CODE) or "").strip()
                merchant_name = (row.get(CSV_COL_MERCHANT_NAME) or "").strip()
                item_desc     = (row.get(CSV_COL_ITEM_DESC) or "").strip()

                width_mm  = safe_float(row.get(CSV_COL_ITEM_WIDTH_MM))
                length_mm = safe_float(row.get(CSV_COL_ITEM_LENGTH_MM))
                height_mm = safe_float(row.get(CSV_COL_ITEM_HEIGHT_MM))


                # if not (merchant_code and item_desc and width_mm and length_mm and height_mm):
                #     continue

                # Upsert merchant
                cur.execute(
                    """
                    INSERT INTO MerchantData (MerchantCode, MerchantName)
                    VALUES (%s, %s)
                    ON DUPLICATE KEY UPDATE MerchantName = VALUES(MerchantName)
                    """,
                    (merchant_code, merchant_name),
                )

                # Upsert item dictionary entry
                cur.execute(
                    """
                    INSERT INTO ItemDictionary
                      (MerchantCode, ItemDescription, ItemWidthMM, ItemLengthMM, ItemHeightMM)
                    VALUES (%s, %s, %s, %s, %s)
                    ON DUPLICATE KEY UPDATE
                      ItemWidthMM  = VALUES(ItemWidthMM),
                      ItemLengthMM = VALUES(ItemLengthMM),
                      ItemHeightMM = VALUES(ItemHeightMM)
                    """,
                    (merchant_code, item_desc, width_mm, length_mm, height_mm),
                )

                inserted += 1

        conn.commit()

    print(f"Processed {processed} CSV rows.")
    print(f"Inserted/updated {inserted} ItemDictionary rows.")

if __name__ == "__main__":
    main()
