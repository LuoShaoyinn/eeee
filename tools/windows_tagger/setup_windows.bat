@echo off
setlocal
py -3 -m venv .venv
call .venv\Scripts\python.exe -m pip install --upgrade pip
call .venv\Scripts\python.exe -m pip install -r tools\windows_tagger\requirements-windows.txt
echo.
echo Setup complete. Run tools\windows_tagger\run_tagger.bat from this folder.
