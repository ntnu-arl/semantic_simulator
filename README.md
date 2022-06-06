# semantic_simulator
Synthetic Semantic Dataset Generator (SSDG)
![](/imgs/sample.png)


## Build and Installation
1. For building the collision avoidance node:
```
mkdir build
cd build
cmake ..
make 
./lidar_node
```
2. For installing the labelling node
- Add labelling_node folder to your catkin workspace
- source ~/workspace/devel/setup.bash

## Run
```
source ~/workspace/devel/setup.bash
roslaunch launch/segmentation.launch directory:=/path/to/your/directory
```
or 
```
roslaunch launch/multi_object.launch directory:=/path/to/your/directory
```