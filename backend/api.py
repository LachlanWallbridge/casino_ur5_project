from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from db import init_db, add_player, remove_player, update_balance, record_game, get_player, get_all_players
from fastapi.middleware.cors import CORSMiddleware

# FastAPI application instance
app = FastAPI()

# Initialize the database
init_db()

# CORS settings to allow frontend access
origins = [
    "http://localhost:3000",  # React dev server
]

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Pydantic models for request bodies ---
class BalanceUpdate(BaseModel):
    amount: float

class GameUpdate(BaseModel):
    won: bool = False

# --- Player endpoints ---

@app.post("/players/{player_id}")
def create_player_endpoint(player_id: int):
    success = add_player(player_id)
    if success:
        return {"status": "ok", "player_id": player_id}
    else:
        return {"status": "exists", "player_id": player_id}

@app.delete("/players/{player_id}")
def delete_player_endpoint(player_id: int):
    success = remove_player(player_id)
    if success:
        return {"status": "deleted", "player_id": player_id}
    else:
        raise HTTPException(status_code=404, detail=f"Player {player_id} not found")

@app.get("/players/{player_id}")
def read_player_endpoint(player_id: int):
    player = get_player(player_id)
    if player:
        return player
    else:
        raise HTTPException(status_code=404, detail=f"Player {player_id} not found")

@app.get("/players")
def read_all_players_endpoint():
    return get_all_players()

@app.post("/players/{player_id}/balance")
def change_balance_endpoint(player_id: int, data: BalanceUpdate):
    success = update_balance(player_id, data.amount)
    if success:
        return {"status": "ok", "player_id": player_id, "amount": data.amount}
    else:
        raise HTTPException(status_code=404, detail=f"Player {player_id} not found")

@app.post("/players/{player_id}/game")
def add_game_endpoint(player_id: int, data: GameUpdate):
    success = record_game(player_id, data.won)
    if success:
        return {"status": "ok", "player_id": player_id, "won": data.won}
    else:
        raise HTTPException(status_code=404, detail=f"Player {player_id} not found")
