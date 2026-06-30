@echo off
REM Start the services with venv activation in each window
start "Serial Handler" cmd /k ".venv\Scripts\activate.bat && python src\serial_handler.py"  
start "Mock CM5" cmd /k ".venv\Scripts\activate.bat && python src\mock_cm5.py"
start "Parser" cmd /k ".venv\Scripts\activate.bat && python src\parser.py"
  