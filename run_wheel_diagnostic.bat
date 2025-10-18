@echo off
echo ========================================
echo Lukebot Wheel Diagnostic Test
echo ========================================
echo.
echo This test will:
echo - Test 4 different wheel configurations
echo - Move the robot in all 6 directions
echo - Print wheel velocities and parameters
echo.
echo Watch the robot and note which direction it moves!
echo.
pause

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_wheel_diagnostic.py

pause
