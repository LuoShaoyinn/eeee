@echo off
setlocal
if not exist .venv\Scripts\python.exe (
  echo Missing .venv. Run tools\windows_tagger\setup_windows.bat first.
  exit /b 1
)
if "%~1"=="" (
  echo Usage: tools\windows_tagger\run_tagger.bat DATASET_DIR CHECKPOINT [extra tag.py arguments]
  echo Example: tools\windows_tagger\run_tagger.bat dataset dataset\models\yolo26n_810_best.pt
  exit /b 1
)
if "%~2"=="" (
  echo A checkpoint path is required.
  exit /b 1
)
call .venv\Scripts\python.exe tools\windows_tagger\tag.py --dataset "%~1" --model "%~2" %3 %4 %5 %6 %7 %8 %9
