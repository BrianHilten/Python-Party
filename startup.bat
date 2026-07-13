@echo off
setlocal

REM Ensure the virtual environment exists
if not exist ".venv\Scripts\activate.bat" (
    echo Virtual environment not found. Creating one...
    python -m venv .venv
)

REM Activate the venv and install dependencies if needed
call ".venv\Scripts\activate.bat"
pip install -r requirements.txt

REM Start the services in separate windows
start "Serial Handler" cmd /k "call .venv\Scripts\activate.bat && python src\serial_handler.py"
start "Mock CM5" cmd /k "call .venv\Scripts\activate.bat && python src\mock_cm5.py"
start "Parser" cmd /k "call .venv\Scripts\activate.bat && python src\parser.py"
start "GUI" cmd /k "call .venv\Scripts\activate.bat && python src\GUI.py"

REM start "Data Logger" cmd /k "call .venv\Scripts\activate.bat && python src\data_logger.py"

endlocal
  