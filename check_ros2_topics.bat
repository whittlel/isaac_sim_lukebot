@echo off
REM Quick script to check ROS 2 topics

echo Setting up ROS 2 Humble environment...
call "C:\dev\ros2_humble\ros2-windows\local_setup.bat"

echo.
echo ========================================
echo ROS 2 Topics Currently Publishing:
echo ========================================
echo.

C:\dev\ros2_humble\ros2-windows\Scripts\ros2.exe topic list

echo.
echo ========================================
echo Checking stereo camera topic frequency:
echo ========================================
echo.
echo Press Ctrl+C to stop monitoring...
echo.

C:\dev\ros2_humble\ros2-windows\Scripts\ros2.exe topic hz /stereo_camera/left/image_raw

pause
