@echo off
rem Build the F710 teleop client on Windows 11 (ADR-0017).
rem Prereqs: ROS2 Humble zip unpacked to C:\dev\ros2_humble, VS Build Tools 2022
rem (C++ workload), SDL2-devel-VC unpacked to C:\dev\SDL2, F710 in XInput mode.
setlocal
set ROS2_DIR=C:\dev\ros2_humble
set SDL2_DIR=C:\dev\SDL2

call "%ROS2_DIR%\local_setup.bat"
if errorlevel 1 (echo ROS2 setup failed & exit /b 1)

call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 (echo vcvars64 failed & exit /b 1)

cmake -S "%~dp0." -B "%~dp0build" ^
  -G "Visual Studio 17 2022" -A x64 ^
  -DSDL2_DIR=%SDL2_DIR%\cmake
if errorlevel 1 exit /b 1

cmake --build "%~dp0build" --config Release
if errorlevel 1 exit /b 1

echo.
echo Built. Run the client with:
echo   call "%ROS2_DIR%\local_setup.bat"
echo   "%~dp0build\Release\f710_teleop.exe"
endlocal
