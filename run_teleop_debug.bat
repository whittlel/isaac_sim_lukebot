@echo off
REM Run Lukebot Teleop Diagnostic Tool

echo ========================================
echo Lukebot Mecanum Wheel Diagnostic Tool
echo ========================================
echo.
echo This will help diagnose mecanum wheel configuration issues
echo.
echo Controls:
echo   W/S - Forward/Backward
echo   A/D - Strafe Left/Right
echo   Q/E - Rotate CCW/CW
echo   SPACE - Stop
echo.
echo   1-4 - Test individual wheels
echo   0 - Normal mode (all wheels)
echo.
echo Watch the console for detailed movement analysis!
echo ========================================
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_debug.py

pause
