@echo off
echo ========================================
echo Lukebot Wheel Debug Test
echo ========================================
echo.
echo This test will check:
echo - What joints exist in the robot
echo - If wheel DOF names are correct
echo - What the articulation sees
echo - Actual vs commanded velocities
echo.
pause

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_debug_wheels.py

pause
