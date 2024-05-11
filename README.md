# Project Updates Documentation
This document outlines the recent enhancements and modifications to the codebase, structured by files and specific functionalities. Each point details the nature of the changes and their impact on the corresponding modules.

## ROS2 & Raspberry Pi 
1. On Ubuntu and Rasspberry Pi: 
``` export ROS_DOMAIN_ID=ID domain (13)``` 
2. Make this setting persistent across terminal sessions and reboots:
```echo "export ROS_DOMAIN_ID=13" >> ~/.bashrc```
```source ~/.bashrc```
3. Publisher:
```ros2 run demo_nodes_cpp talker```
4. Listener:
```ros2 run demo_nodes_cpp listener```
5. ping:
```ping hostname -I```
6. Important note about multiplexing machines with ROS
The UF network appears to prevent us from talking and listening between two devices. For this reason we need to connect both machines to a mobile hotspot. 

## Launch Instructions

``` bash
configs chmod +x sam_bot_run.sh
        ./sam_bot_run.sh
```



## Appendix: 
1. Link to the ROS2 & Pi communication:
```https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/```
