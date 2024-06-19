# autonomous_car
KKU Autonomous Car with ardupilot

prerequisites

Setup Ardurover Flight Controller to Rover Base Drive
Make sure Set parameter Servo1 = ground_stearing / Servo3 = throttle 
if vehicle not have Board Button -> Set BRD_SAFETY parameter to 0 

On Computer
install Linux Ubuntu
install Python3
Install VSCode
install Qground follow instruction  https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

for mqtt install paho mqtt

To run mav proxy with multiple udp out
mavproxy.py --master=/dev/ttyACM1 --out 127.0.0.1:14550 --out 127.0.0.1:14570 --out 127.0.0.1:14560
to run Q ground
./QGroundControl.AppImage

Incase of USB Serial not alow or bussy
to list tty
dmesg | grep tty
if Access denied
sudo chmod 666 /dev/ttyUSB0
if bussy then Kill Process
fuser -k /dev/ttyUSB0

get file 
wget https://github.com/watcharains/autonomous_car/blob/main/aris_mission_with_stop.py
