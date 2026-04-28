@echo off
setlocal

set "ROOT=%~dp0"
set "VSDEVCMD=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
set "CMAKE_EXE=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "BUILD_DIR=%ROOT%build_2cam_x64"

if not exist "%VSDEVCMD%" (
    echo VsDevCmd.bat was not found:
    echo   %VSDEVCMD%
    exit /b 1
)

if not exist "%CMAKE_EXE%" (
    echo cmake.exe was not found:
    echo   %CMAKE_EXE%
    exit /b 1
)

call "%VSDEVCMD%" -arch=amd64 -host_arch=amd64
if errorlevel 1 exit /b %errorlevel%

cd /d "%ROOT%"

echo [1/2] Configuring x64 Ninja build...
"%CMAKE_EXE%" -S "%ROOT%" -B "%BUILD_DIR%" -G Ninja
if errorlevel 1 exit /b %errorlevel%

echo [2/2] Building track_test_cpp_2cam...
"%CMAKE_EXE%" --build "%BUILD_DIR%" --target track_test_cpp_2cam
exit /b %errorlevel%
