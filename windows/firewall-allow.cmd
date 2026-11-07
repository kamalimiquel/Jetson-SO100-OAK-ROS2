@echo off
rem Allow ROS2/DDS traffic through Windows Firewall (ADR-0017 known silent
rem failure point). Run from an ELEVATED prompt (right-click -> Run as admin).
rem The rules cover the FastDDS UDP port range and the ros2 binaries themselves
rem on domain 42. LAN profile only — nothing is opened to public networks.

netsh advfirewall firewall delete rule name="ROS2 DDS UDP" >nul 2>&1
netsh advfirewall firewall add rule name="ROS2 DDS UDP" dir=in action=allow ^
  protocol=UDP localport=7400-7450 profile=private
netsh advfirewall firewall add rule name="ROS2 DDS UDP out" dir=out action=allow ^
  protocol=UDP remoteport=7400-7450 profile=private

echo Done. If discovery still fails between Windows and the Xavier, use static
echo FastDDS peers (ADR-0002): add FASTRTPS_DEFAULT_PROFILES_FILE pointing at a
echo profile with the Xavier's IP.
pause
