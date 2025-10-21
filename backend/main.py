import subprocess
import threading
from db import init_db
from cli import app as cli_app

def run_api():
    """Run FastAPI server in a separate thread."""
    subprocess.run(["uvicorn", "backend.api:app", "--reload", "--port", "8000"])

def main():
    init_db()
    
    # Optionally start the API in background
    api_thread = threading.Thread(target=run_api, daemon=True)
    api_thread.start()
    
    # Start CLI
    cli_app()

if __name__ == "__main__":
    main()
