"""
One-off migration: add batch_id and schema_version to runs table.
Run from omega_lab: python db/migrate_add_batch_schema.py
"""

import sqlite3
from pathlib import Path

DB_PATH = Path(__file__).parent / "omega.db"

def main():
    if not DB_PATH.exists():
        print(f"No DB at {DB_PATH}; init_db will create tables with new columns.")
        return
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    try:
        cur.execute("ALTER TABLE runs ADD COLUMN batch_id VARCHAR(128);")
        print("Added batch_id")
    except sqlite3.OperationalError as e:
        if "duplicate column" in str(e).lower():
            print("batch_id already exists")
        else:
            raise
    try:
        cur.execute("ALTER TABLE runs ADD COLUMN schema_version VARCHAR(16) DEFAULT '1.0';")
        print("Added schema_version")
    except sqlite3.OperationalError as e:
        if "duplicate column" in str(e).lower():
            print("schema_version already exists")
        else:
            raise
    conn.commit()
    conn.close()
    print("Migration done.")

if __name__ == "__main__":
    main()
