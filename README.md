# motor_control_rpi3
Motor control with PID. Motor controller for AlphaBot using only RaspberryPi 3B
https://www.waveshare.com/wiki/AlphaBot#Overview

Be careful! Although raspberry Pi can be directly connected to Alphabot board All signals going into raspberry pi are 0-5V! According to docs of RPi3 t=%V can fry it! I tested it on raspberry Pi 3 and It was working fine but I used voltage converter 5V to 3.3V to make it safer.

# Dependencies
## Pigpio
Install pigpio according to site https://abyz.me.uk/rpi/pigpio/download.html
```
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```
### Libevdev
Libevdev is used for remote control via XBoxController
```
sudo apt-get install libevdev-dev # Install libevdev development package
```

# Compiling
## using raw g++ command
To build project tou can use direct g++ commands
### single_control
```
cd build/
g++ -Wall -pthread -o single_control ../single_control.cpp ../pid.cpp ../adc.cpp ../motor_ctrl.cpp ../robot_move.cpp ../rotary_encoder.cpp -lpigpio -lrt -lc
```
or
```
cd build/
g++ -Wall -pthread -o single_control ../single_control.cpp ../pid.cpp ../adc.cpp ../motor_ctrl.cpp ../robot_move.cpp ../rotary_encoder.cpp -lpigpio -lrt -lc -g
```
for debugging
### remote_control
```
cd build/
g++ -Wall -pthread -o remote_control ../remote_control.cpp ../pid.cpp ../adc.cpp ../motor_ctrl.cpp ../robot_move.cpp ../xbox_controller.cpp ../rotary_encoder.cpp -I/usr/include/libevdev-1.0 -levdev -lpigpio -lrt -lc
```
or
```
cd build/
g++ -Wall -pthread -o remote_control ../remote_control.cpp ../pid.cpp ../adc.cpp ../motor_ctrl.cpp ../robot_move.cpp ../xbox_controller.cpp ../rotary_encoder.cpp -I/usr/include/libevdev-1.0 -levdev -lpigpio -lrt -lc -g
```
for debugging

## using cmake
```
cd build/
cmake ..
make
```
for debug or
```
cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
for clean release version
