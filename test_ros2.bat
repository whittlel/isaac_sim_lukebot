@echo off
REM Test ROS 2 Humble Installation

echo Testing ROS 2 Humble...
echo.

REM Setup ROS 2 environment
call "C:\dev\ros2_humble\ros2-windows\local_setup.bat"

echo.
echo ROS 2 Version:
ros2 --version

echo.
echo ROS 2 Commands Available:
ros2 --help

echo.
echo Testing topic list:
ros2 topic list

echo.
echo ROS 2 Installation Test Complete!
pause
