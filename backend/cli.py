import typer
from db import init_db, add_player, remove_player, update_balance, record_game, get_player, get_all_players

app = typer.Typer()

@app.command()
def create(
    player_id: int = typer.Argument(..., help="ID of the new player"),
    balance: float = typer.Option(0.0, "--balance", "-b", help="Starting balance for the player")
):
    """Add a new player with optional starting balance."""
    success = add_player(player_id, balance)
    if success:
        print(f"✅ Player {player_id} added with balance {balance}.")
    else:
        print(f"⚠️ Player {player_id} already exists.")

@app.command()
def delete(player_id: int):
    """Remove a player."""
    success = remove_player(player_id)
    if success:
        print(f"❌ Player {player_id} removed.")
    else:
        print(f"⚠️ Player {player_id} not found.")

@app.command()
def balance(player_id: int, amount: float):
    """Update a player's balance by an amount."""
    success = update_balance(player_id, amount)
    if success:
        print(f"💰 Player {player_id} balance updated by {amount}.")
    else:
        print(f"⚠️ Player {player_id} not found.")

@app.command()
def record(player_id: int, won: bool = typer.Option(False, "--won", "-w", help="Mark this game as a win")):
    """Record a game for a player."""
    success = record_game(player_id, won)
    if success:
        print(f"🏆 Recorded game for {player_id}, won={won}")
    else:
        print(f"⚠️ Player {player_id} not found.")

@app.command()
def show(player_id: int):
    """Show a single player's info."""
    player = get_player(player_id)
    if player:
        print(player)
    else:
        print(f"⚠️ Player {player_id} not found.")

@app.command()
def list_all():
    """List all players."""
    players = get_all_players()
    if players:
        for p in players:
            print(p)
    else:
        print("⚠️ No players found.")

if __name__ == "__main__":
    init_db()
    app()
