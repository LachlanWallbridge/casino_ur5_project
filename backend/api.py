from fastapi import FastAPI
from pydantic import BaseModel
from db import init_db, add_player, remove_player, update_balance, record_game, get_player, get_all_players

app = FastAPI()
init_db()

class BalanceUpdate(BaseModel):
    amount: float

class GameUpdate(BaseModel):
    won: bool = False

# --- Player endpoints ---

@app.post("/players/{player_id}")
def create_player(player_id: str):
    add_player(player_id)
    return {"status": "ok", "player_id": player_id}

@app.delete("/players/{player_id}")
def delete_player(player_id: str):
    remove_player(player_id)
    return {"status": "deleted", "player_id": player_id}

@app.get("/players/{player_id}")
def read_player(player_id: str):
    player = get_player(player_id)
    return player or {"error": "not found"}

@app.get("/players")
def read_all_players():
    return get_all_players()

@app.post("/players/{player_id}/balance")
def change_balance(player_id: str, data: BalanceUpdate):
    update_balance(player_id, data.amount)
    return {"status": "ok", "player_id": player_id, "amount": data.amount}

@app.post("/players/{player_id}/game")
def add_game(player_id: str, data: GameUpdate):
    record_game(player_id, data.won)
    return {"status": "ok", "player_id": player_id, "won": data.won}
