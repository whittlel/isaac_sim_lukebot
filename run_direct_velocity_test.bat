@echo off
echo Testing DIRECT velocity control...
cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_direct_velocity_test.py
pause
