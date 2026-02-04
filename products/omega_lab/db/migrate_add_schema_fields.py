"""Add schema_version to hypotheses and evidence tables"""
import sqlite3
from pathlib import Path

DB_PATH = Path(__file__).parent / "omega.db"


def migrate():
    if not DB_PATH.exists():
        print(f"No DB at {DB_PATH}; init_db will create tables with new columns.")
        return
    conn = sqlite3.connect(str(DB_PATH))
    cursor = conn.cursor()

    # Check and add to hypotheses
    cursor.execute("PRAGMA table_info(hypotheses)")
    hyp_cols = [row[1] for row in cursor.fetchall()]

    if "schema_version" not in hyp_cols:
        cursor.execute(
            "ALTER TABLE hypotheses ADD COLUMN schema_version VARCHAR(16) DEFAULT '1.0'"
        )
        print("Added schema_version to hypotheses")

    # Check and add to evidence
    cursor.execute("PRAGMA table_info(evidence)")
    ev_cols = [row[1] for row in cursor.fetchall()]

    if "schema_version" not in ev_cols:
        cursor.execute(
            "ALTER TABLE evidence ADD COLUMN schema_version VARCHAR(16) DEFAULT '1.0'"
        )
        print("Added schema_version to evidence")

    conn.commit()
    conn.close()
    print("Migration complete.")


if __name__ == "__main__":
    migrate()
