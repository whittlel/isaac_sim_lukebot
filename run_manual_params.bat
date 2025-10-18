@echo off
REM Run Lukebot Teleop - MANUAL PARAMETERS VERSION

echo ========================================
echo Lukebot Teleoperation
echo MANUAL CONTROLLER PARAMETERS
echo ========================================
echo.
echo This version manually specifies all controller parameters
echo based on the URDF geometry, bypassing automatic detection.
echo.
echo Wheel Positions (from URDF):
echo   Front Left:  X=+0.15, Y=+0.17
echo   Front Right: X=+0.15, Y=-0.17
echo   Rear Left:   X=-0.15, Y=+0.17
echo   Rear Right:  X=-0.15, Y=-0.17
echo.
echo Mecanum Angles (Standard X-Pattern):
echo   FL: +45 deg, FR: -45 deg
echo   RL: -45 deg, RR: +45 deg
echo.
echo Controls:
echo   W - Move forward (toward RED marker)
echo   A - Strafe left (toward GREEN marker)
echo   S/D/Q/E - Other directions
echo   SPACE - Stop
echo.
echo Watch the console for detailed wheel velocity info!
echo ========================================
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_manual_params.py

pause
