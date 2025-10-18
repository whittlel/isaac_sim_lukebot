@echo off
REM Run Lukebot Automatic Wheel Velocity Test

echo ========================================
echo Lukebot Automatic Wheel Velocity Test
echo ========================================
echo.
echo This script will automatically test each movement direction
echo and print the wheel velocities to help diagnose the issue.
echo.
echo Tests:
echo   1. Forward (W)
echo   2. Backward (S)
echo   3. Strafe Left (A)
echo   4. Strafe Right (D)
echo   5. Rotate CCW (Q)
echo   6. Rotate CW (E)
echo.
echo Each test runs for a few seconds, then waits.
echo Watch the console for wheel velocity output!
echo.
echo The test will run TWICE:
echo   - First round: NO axis swap
echo   - Second round: WITH axis swap
echo.
echo ========================================
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_auto_test.py

pause
