@echo off
setlocal
pushd "%~dp0"
if not exist "build_2cam_x64\track_test_cpp_2cam.exe" (
    echo Build first with build_2cam_x64.cmd
    popd
    exit /b 1
)
"build_2cam_x64\track_test_cpp_2cam.exe" "track_config_2_floor.json"
set "TRACK_FLOOR_RESULT=%errorlevel%"
popd
if not "%TRACK_FLOOR_RESULT%"=="0" pause
exit /b %TRACK_FLOOR_RESULT%
