@echo off
echo ========================================
echo Lukebot Individual Wheel Test
echo ========================================
echo.
echo This test spins each wheel ONE AT A TIME
echo to verify the wheel order is correct.
echo.
echo WATCH CAREFULLY which wheel spins!
echo.
pause

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_individual_wheel_test.py

pause
