# CPSquare-Bitcraze-firmware

![Image](https://github.com/user-attachments/assets/569c3641-8b44-4794-8f21-5bf1033764fa)

## Description
This repository was created by undergraduate researchers: Minwoo Park and Niko Tambornini and advisor: Dr.Siavash Farzan. This project is apart of Dr.Siavash Farzan's Cal Poly Cyber-Physical Systems Lab or CPSquare Lab under the title: Experimental Framework for Multi-Agent Coordination and Distributed Localization in Micro-UAVs. Furthermore, this firmware is a modification of the Bitcraze firmware for the Crazyflie platform found in: https://github.com/bitcraze/crazyflie-firmware. 

To view other research or work produced by CPSquare, please visit: https://sfarzan.com/index.html.

## Directions

### Prerequisite Software Installation

1. Install Python 3.11.9 

2. Install cflib python package utilizing pip 
    - cflib version is 0.1.27

3. Setup Crazyradio 2.0
    - firmware is crazyradio2-CRPA-emulation-1.1.uf2

4. Install Zadig 2.9 
    - install libusb v1.4.0.0 driver

5. Install cfclient (run using python -m cfclient.gui)
    - pip install cfclient
      
### Firmware Compilation
In the examples folder (/examples), there are 4 individual folders that hold the custom firmware designed for both follower and leader drones. These folders include: app_follower, app_follower_tester, app_leader, and app_multiranger. 

Command Line Commands to compile firmware:

1. ```make clean```
2. ```make -j$(nproc)```
3. ```make cload```


#### Leader drone firmware

Found in the app_leader folder (/examples/app_leader), the src folder (/examples/app_leader/src) is where the commands 
