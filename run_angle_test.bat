@echo off
REM Test Different Mecanum Angle Configurations

echo ========================================
echo Mecanum Angle Configuration Tester
echo ========================================
echo.
echo This will test 5 different mecanum wheel angle configurations
echo to find the one that works correctly for your robot.
echo.
echo CONFIGURATIONS TO TEST:
echo   1 - Standard X-Pattern:  FL=+45, FR=-45, RL=-45, RR=+45
echo   2 - Inverted X-Pattern:  FL=-45, FR=+45, RL=+45, RR=-45
echo   3 - Mirrored Front/Back: FL=+45, FR=-45, RL=+45, RR=-45
echo   4 - O-Pattern:           FL=-45, FR=-45, RL=+45, RR=+45
echo   5 - All Positive:        FL=+45, FR=+45, RL=+45, RR=+45
echo.
echo CONTROLS:
echo   W - Move forward (should go toward RED marker)
echo   A - Strafe left (should go toward GREEN marker)
echo   1-5 - Switch configurations on the fly
echo   SPACE - Stop
echo.
echo TEST PROCEDURE:
echo   1. Press W and see if robot moves forward
echo   2. Press A and see if robot strafes left smoothly
echo   3. If wrong, press 2/3/4/5 to try other configs
echo   4. Note which config number works best!
echo.
echo ========================================
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat lukebot_teleop_angle_test.py

pause
