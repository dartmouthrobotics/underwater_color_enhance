# Underwater Color Enhance Methods

NOTE: This is a C++ version. Original is in python, thus this is a work in progress.

## To Do:

Overall:
* Add comments and check style guide
* Optimize code

Running the program
* (ROS) Make it run via launch file
* (ROS) Check is catkin_make will actually build the files needed

Single Image Application
* create YAML file to provide image formation: depth, distance, color patch size and location, method, estimation or calculation
* write script that creates that YAML file (EXTRA)

New Model Method:
* Calculate attenuation values:
  * Sample pixels from patches
* Wideband veiling light:
  * Need to get average background pixel
  * Calculate using known camera and water properties (EXTRA)

ROS Application
* Start it

USB Application
* Start it

## Getting Started

This package has been tested on Ubuntu 16.04.

Set up your catkin workspace

```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

## Install

Clone repository:

```
git clone
cd ..
```

Build:

```
chmod +x build.sh
./build.sh
```

## Run

In the `color_correct` repository:

For one image:

```
./Options/Image/image_correct <image path>
```

Example:

```
./Options/Image/image_correct Images/shipwreck_depth_000606.png
```
