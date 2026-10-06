@echo off
setlocal
chcp 65001 >nul
set PYTHONUTF8=1
pushd "%~dp0"
python tools\capture_floor_calibration.py %*
set "CAPTURE_EXIT_CODE=%ERRORLEVEL%"
popd
if not "%CAPTURE_EXIT_CODE%"=="0" pause
exit /b %CAPTURE_EXIT_CODE%
