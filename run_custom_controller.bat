@echo off
REM Run Lukebot Teleop - CUSTOM CONTROLLER

echo ========================================
echo Lukebot Custom Mecanum Controller
echo ========================================
echo.
echo This version bypasses Isaac Sim's HolonomicController
echo and implements mecanum kinematics manually.
echo.
echo Mecanum Inverse Kinematics:
echo   FL = (vx - vy - L*omega) / r
echo   FR = (vx + vy + L*omega) / r
echo   RL = (vx + vy - L*omega) / r
echo   RR = (vx - vy + L*omega) / r
echo.
echo Where:
echo   vx = forward velocity
echo   vy = lateral velocity (positive = left)
echo   omega = rotation velocity (positive = CCW)
echo   L = (wheelbase_length + wheelbase_width) / 2
echo   r = wheel radius
echo.
echo Controls:
echo   W/S - Forward/Backward
echo   A/D - Strafe Left/Right
echo   Q/E - Rotate CCW/CW
echo   SPACE - Stop
echo.
echo This should give PROPER omnidirectional control!
echo ========================================
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_custom_controller.py

pause
