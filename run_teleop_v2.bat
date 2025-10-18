@echo off
echo ========================================
echo Lukebot Teleoperation V2
echo Manual Mecanum Kinematics (Sim-to-Real)
echo ========================================
echo.
pause

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_v2.py

pause
