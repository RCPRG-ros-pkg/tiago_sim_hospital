# tiago_sim_hospital

Repository of Tiago robot simulation in hospital environment. Requires package [tiago_sim_integration](https://github.com/RCPRG-ros-pkg/tiago_sim_integration).

## Instalation

First install ROS melodic as it is described on the developer site. Then run the following commands:
```
mkdir $HOME/tiago_public_ws
cd $HOME/tiago_public_ws
mkdir -p src/rcprg/tiago_sim_hospital
git clone https://github.com/RCPRG-ros-pkg/tiago_sim_hospital.git src/rcprg/tiago_sim_hospital
bash src/rcprg/tiago_sim_hospital/tiago_sim_hospital_packages.sh
rosinstall src /opt/ros/melodic src/rcprg/tiago_sim_hospital/tiago_sim_hospital_repositories.rosinstall
source /opt/ros/melodic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0
source $HOME/tiago_public_ws/devel/setup.bash
```

## Contents

### Models

New furniture models used in Gazebo worlds are in:
```
/models
```
Original models created in Blender are stored in:
```
/blender_ws
```
Created customizable robot model for doors is in:
```
/urdf
```
It's usage can be found in:
```
/launch/gazebo_test_door.launch
```

### Worlds

Gazebo worlds can be found in:
```
/worlds
```
Their generated maps are in:
```
/maps
```

### Custom motions

Custom motions created for the robot are in folder:
```
/config
```

## Usage

All launch files will open 'hospital_v2' world on default. To run launch files with different world use world_version argument.

### Mapping

To map any world found in the package use launch:
```
/launch/tiago_mapping_hospital.launch
```
Then you should change option **Global Options -> Fixed Frame** in rviz from **map** to **odom**. You can move around the world using **2D Nav Goal** tool. Alternatively you can utilize **key_teleop** package developed by pal, that enables you to steer robot with your keyboard. To do this run the following command in another console:
```
rosrun key_teleop key_teleop.py
```
You can save the created map with command:
```
rosrun map_server map_saver -f <target_directory/target_map_name.pgm>
```

### Navigation

To navigate with robot through the environment use:
```
/launch/tiago_navigation_hospital.launch
```

### Manipulation

To use the robot's version with manipulator connected and load custom motions use:
```
/launch/tiago_manipulation_hospital.launch
```

## TIPS

### Editing Gazebo world

While editing the worlds remember in advance to export the furniture models. To do this source the used worksapce and run the following command:
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find tiago_sim_hospital)/models:$(rospack find tiago_sim_integration)/models
```

### Fixing faded colors in .dae files exported by Gazebo

To fix colors of models exported from Bleder in .dae format, open the generated models and in every color, above **diffuse** section, add the following lines while changing the 0.0 values to those copied from **diffuse** section.
```
            <ambient>
              <color sid="ambient">0.0 0.0 0.0 0.0</color>
            </ambient>
```
