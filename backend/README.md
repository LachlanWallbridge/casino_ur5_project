# Backend (FastAPI + CLI)

This folder contains the backend of the project, including:

- FastAPI server (`api.py`) for player management.
- CLI (`cli.py`) for manually creating, deleting, and updating players.
- SQLite database (`players.db`) to store player stats.

## Requirements

- Python 3.10+
- Install dependencies:

```bash
pip install -r requirements.txt
```

# Recommended Workflows
## 1. Run FastAPI Server Only

Start the backend API (useful for frontend integration):

```bash
cd backend
uvicorn api:app --reload --port 8000
```
- FastAPI will be accessible at: http://localhost:8000
- Make sure CORS is enabled if you are running the frontend on another port (e.g., 3000).

## 2. Run CLI Only
Perform player operations via CLI:
```bash
cd backend
python3 cli.py create 151 --balance 500
python3 cli.py balance 151 100
python3 cli.py delete 151
```
- Use python3 cli.py --help to see all commands.

## 3. Run API and CLI Together (Optional)


`main.py` can run the FastAPI server in a background thread and then start the CLI:

```bash
python3 main.py
```
Note: Running this way may require your working directory to be the project root and Python path properly configured. It is also broken currently


## Notes

- The database `players.db` is created automatically if it doesn't exist.
- All player stats can be accessed via REST endpoints:

| Method | Endpoint | Description |
|--------|---------|-------------|
| GET    | /players | Get all players |
| GET    | /players/{player_id} | Get single player |
| POST   | /players/{player_id} | Create player |
| DELETE | /players/{player_id} | Delete player |
| POST   | /players/{player_id}/balance | Update player balance |
| POST   | /players/{player_id}/game | Record a game for player |
---