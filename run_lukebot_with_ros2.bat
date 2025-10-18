@echo off
REM Run Lukebot Stereo vSLAM Demo with ROS 2 Support

echo ========================================
echo Lukebot Stereo vSLAM with ROS 2
echo ========================================
echo.

REM Set up EXTERNAL ROS 2 Humble environment (provides complete DLL dependencies)
echo Setting up ROS 2 Humble environment from C:\dev\ros2_humble...
call "C:\dev\ros2_humble\ros2-windows\local_setup.bat"

REM Override to use Isaac Sim's message types while using external ROS 2 runtime
echo Adding Isaac Sim ROS 2 message libraries...
set PATH=C:\Users\Luke\Desktop\issac_sim\_build\windows-x86_64\release\exts\isaacsim.ros2.bridge\humble\lib;%PATH%

echo ROS_DISTRO: %ROS_DISTRO%
echo RMW_IMPLEMENTATION: %RMW_IMPLEMENTATION%
echo.
echo ROS 2 Version:
ros2 --version
echo.

REM Now run Isaac Sim with ROS 2 environment active
echo Starting Isaac Sim with Lukebot Stereo vSLAM...
echo.

cd /d "C:\Users\Luke\Desktop\issac_sim"
_build\windows-x86_64\release\python.bat source\standalone_examples\api\isaacsim.robot.wheeled_robots\lukebot_stereo_vslam.py

pause
