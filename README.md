# Underwater Color Enhance Methods

NOTE: This is a C++ version. Original is in python, thus this is a work in progress.

Implementing image correction model proposed in:

"A Revised Underwater Image Formation Model"  
CVPR 2018  
Derya Akkaynak and Tall Treibitz  
University of Haifa

## To Do:

Overall:
* Add escalibr, ping nodelet, and ORB-SLAM2 (modified) into catkin_ws

Running the program
* (ROS) Make it run via launch file
* (ROS) Check is catkin_make will actually build the files needed

New Model Method:
* Wideband veiling light [FUTURE]:
  * Calculate average background pixel using image processing

ROS Application
* add optimization
* check if is_adaptive is needed


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
cd underwater_color_enhance
```

## Third-party Packages

* yaml-cpp

In its CMakeLists.txt, comment out:
```
### Extras

if(YAML_CPP_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
if(YAML_CPP_BUILD_TOOLS)
  add_subdirectory(util)
endif()
```

* ticpp

* dlib

## Build

```
chmod +x build.sh
./build.sh
```

## Configuration

`image_config.yaml`:
* image: \<path to singular input image\>  <br><br>

* distance: <from the camera to the object of interest, in meters>
* depth: <positive value, in meters>
* background_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* color_1_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* color_2_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>] <br><br>

* method: <0: A Revised Underwater Image Formation Model>
* is_adaptive: <???> (currently: false)
* optimize: <???> (currently: false)
* est_veiling_light: <true: uses background sample to calculate average wideband veiling light | false: TO DO> <br><br>

* show_image: <true: shows color corrected image | false: does not show color corrected image>  <br><br>

* save_data: <true/false: attenuation values saved to 'output_filename' or not>
* prior_data: <true/false: attenuation values used from 'input_filename' or not>
* output_filename: \<xml file to save attenuation values with its depth measurement\>
* input_filename: \<xml file to load attenuation values with its depth measurement\>

## Run

For one image (no ROS implementation), based on the parameters in `image_config.yaml`:

```
roslaunch underwater_color_enhance image_color_enhance.launch
```

For rosbag files, based on the parameters in `ros_config.yaml`:

```
roslaunch underwater_color_enhance ros_color_enhance.launch
```
