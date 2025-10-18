@echo off
REM Run Lukebot Teleop - FIXED VERSION

echo ========================================
echo Lukebot Teleoperation - FIXED VERSION
echo ========================================
echo.
echo FIXES APPLIED:
echo   - Axis swap compensation (90 degree offset)
echo   - Increased damping to reduce jitter
echo   - Rotation direction correction
echo.
echo Controls:
echo   W - Move forward (toward RED marker)
echo   S - Move backward
echo   A - Strafe left (toward GREEN marker)
echo   D - Strafe right
echo   Q/E - Rotate CCW/CW
echo   SPACE - Stop
echo.
echo Press W and watch - robot should move FORWARD now!
echo ========================================
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_fixed.py

pause
