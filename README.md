[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)  

ROS package for segmentation based object detection of the KITTI data set
==========

This repo is cloned from [SARosPerceptionKitti](https://github.com/appinho/SARosPerceptionKitti) for object detection of the KITTI data set

## Demo
<p align="center">
  <img src="./doc/images/semantic.gif">
</p>

# Setup

Store the rosbag file of KITTI data set as follows:
```
    ~                                        # Home directory
    └── catkin_ws                            # Catkin workspace
        └── src                              # Source folder
            └── SARosPerceptionKitti         # Repo
                ├── data                     # Dataset directory
                    └── rosbags              # ROSbag directory
                        ├── 0000.bag         # ROSbag file
                        ... 
                        └── 0013.bag         # ROSbag file
```
or change the directory folder in all launch files `.${node_name}/${node_name}.launch`.

1) Install [ROS](http://wiki.ros.org/Installation/Ubuntu) and create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in your home directory:  

```
mkdir -p ~/catkin_ws/src
```

2) Clone this repository into the catkin workspace's source folder (src) and build it:  

```
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/appinho/SARosPerceptionKitti.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3) [Download a preprocessed scenario](https://drive.google.com/drive/folders/1vHpkoC78fPXT64-VFL1H5Mm1bdukK5Qz?usp=sharing). Unzip and store under the folder in [this instruction](#setup)

