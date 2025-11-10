# Frontend (React)

This folder contains the React frontend for displaying:

- Board state (top area)
- Player cards (bottom area) dynamically filled from the backend
- Integration with `Players.msg` via `ui_bridge.js` (stub for now)

## Requirements

- Node.js 18+ and npm
- React dependencies installed via:

```bash
cd frontend
npm install
```

## Recommended Workflow
### 1. Start Backend API

Ensure FastAPI backend is running on `http://localhost:8000`:

```bash
cd backend
uvicorn api:app --reload --port 8000
```

### 2. Start React Frontend
In a separate terminal:

```bash
cd frontend
npm start
```
- The frontend dev server will run on http://localhost:3000.
- Open your browser to this URL to see the board and player cards.

### 3. Run rosbridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```