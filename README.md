Welcome to step by step for GO2 unitree.
------For ROS2 Jazzy only------

We using Livox so we need to install the driver of it. But since we use ROS Jazzy we need to install SDK header first
```
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2
cd Livox-SDK2
```

then we apply C++20 

```
nano /Livox-SDK2/sdk_core/comm/define.h
#include <cstdint>
nano /sdk_core/logger_handler/file_manager.h
#include <cstdint>
```
then ``` Crlt + o``` then Enter to save, and ``` Crtl + X``` to exit

then build and install
```
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
Build ROS Driver

```
git clone https://github.com/Livox-SDK/livox_ros_driver2
cd ~/ws_livox/src/livox_ros_driver2
```

build for ROS Jazzy
```
./build.sh humble
source /install/setup.bash
```

then try run

```
ros2 launch livox_ros_driver2 rvis_MID360.launch or smt
```

after that to MAPPING run
```
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

another termainal run
```
ros2 launch fast_lio mapping.launch.py
```

after that to save map run 
```
ros2 service call /map_save std_srvs/srv/Trigger {}
```

to view map use 
```
#go to ~/your_ws/src/FAST_LIO/PCD
pcl_viewer [file_name].pcd
```





