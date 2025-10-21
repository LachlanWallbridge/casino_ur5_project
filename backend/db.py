import sqlite3
from pathlib import Path

# Database path
DB_PATH = Path(__file__).parent / "players.db"

def get_conn():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    """Create the players table if it doesn't exist."""
    conn = get_conn()
    conn.execute("""
    CREATE TABLE IF NOT EXISTS players (
        player_id INTEGER PRIMARY KEY,
        balance REAL DEFAULT 0.0,
        games_played INTEGER DEFAULT 0,
        games_won INTEGER DEFAULT 0
    );
    """)
    conn.commit()
    conn.close()
    return True  # Always succeeds

def add_player(player_id: int, balance: float = 0.0):
    """Add a new player if not exists, with optional starting balance."""
    conn = get_conn()
    try:
        cursor = conn.execute(
            "INSERT OR IGNORE INTO players (player_id, balance) VALUES (?, ?)",
            (player_id, balance)
        )
        conn.commit()
        return cursor.rowcount > 0  # True if inserted, False if ignored
    finally:
        conn.close()

def remove_player(player_id: int):
    conn = get_conn()
    try:
        cursor = conn.execute("DELETE FROM players WHERE player_id = ?", (player_id,))
        conn.commit()
        return cursor.rowcount > 0  # True if a row was deleted
    finally:
        conn.close()

def update_balance(player_id: int, amount: float):
    conn = get_conn()
    try:
        cursor = conn.execute(
            "UPDATE players SET balance = balance + ? WHERE player_id = ?",
            (amount, player_id)
        )
        conn.commit()
        return cursor.rowcount > 0  # True if updated
    finally:
        conn.close()

def record_game(player_id: int, won: bool = False):
    conn = get_conn()
    try:
        if won:
            cursor = conn.execute(
                "UPDATE players SET games_played = games_played + 1, games_won = games_won + 1 WHERE player_id = ?",
                (player_id,)
            )
        else:
            cursor = conn.execute(
                "UPDATE players SET games_played = games_played + 1 WHERE player_id = ?",
                (player_id,)
            )
        conn.commit()
        return cursor.rowcount > 0
    finally:
        conn.close()

def get_player(player_id: int):
    conn = get_conn()
    try:
        row = conn.execute("SELECT * FROM players WHERE player_id = ?", (player_id,)).fetchone()
        return dict(row) if row else None
    finally:
        conn.close()

def get_all_players():
    conn = get_conn()
    try:
        rows = conn.execute("SELECT * FROM players").fetchall()
        return [dict(row) for row in rows]
    finally:
        conn.close()
