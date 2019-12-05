# Underwater Color Enhancement Method

[Dartmouth Reality and Robotics Lab](http://rlab.cs.dartmouth.edu/home/)

Authors: [Monika Roznere](http://monikaroznere.com) and [Alberto Quattrini Li](https://sites.google.com/view/albertoq)

NOTE: Current application should work well. There are some small issues that need to dealt with, see below.

## Related Publications

M.  Roznere  and  A.  Quattrini  Li,  “Real-time  model-based  image  color correction  for  underwater  robots,” IROS, 2019. [PDF](https://arxiv.org/abs/1904.06437)

## License

If you use our underwater color enhancement application in an academic work, please cite:

```
@inproceedings{RoznereColor2019,
  title={Real-time Model-based Image Color Correction for Underwater Robots},
  author={Monika Roznere and Alberto {Quattrini Li}},
  booktitle={Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems},
  year={2019}
 }
```

## Work in Progress

Overall:
* Time optimize color enhancement.
  * Currently it takes ~7 ms to enhance an image (1920 x 1080).

New Model Method:
* Wideband veiling light [FUTURE]:
  * Calculate average background pixel using image processing

## Getting Started

This package has been tested on Ubuntu 16.04.

Set up your catkin workspace

```
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace
```

## Install

Clone repository:

```
git clone https://github.com/dartmouthrobotics/underwater_color_enhance
```

Clone and build this repository as well:

* [modified monocular ORB-SLAM](https://github.com/dartmouthrobotics/ORB_SLAM2)

```
git clone https://github.com/dartmouthrobotics/ORB_SLAM2
cd ORB_SLAM2/
cd Thirdparty/
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin/
mkdir build
cd build/
cmake ..
cmake --build .
cd ../../../
chmod +x build.sh
./build.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Examples/ROS/ORB_SLAM2ROS
chmod +x build_ros.sh
./build_ros.sh
```

## Third-party Packages

Run:

```
cd underwater_color_enhance/
mkdir third-party
cd third-party/
```

In the `third-party` directory, clone these repositories:

* [yaml-cpp](https://github.com/jbeder/yaml-cpp)

```
git clone https://github.com/jbeder/yaml-cpp
git clone https://github.com/wxFormBuilder/ticpp
git clone https://github.com/davisking/dlib
```

<!--
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
-->

* [ticpp](https://github.com/wxFormBuilder/ticpp)

* [dlib](https://github.com/davisking/dlib)

## Build

```
catkin_make
source devel/setup.bash
```

## Configuration

`config/image_config.yaml`:
* image: \<path to singular input image\>  <br><br>

* distance: \<from the camera to the object of interest, in meters\>
* depth: \<altitude depth; positive value, in meters\>
* camera_response_filename: \<path to camera response file\>
  * `Sony_IMX322LQJ-C_Camera_Response.csv` is the USB camera used on the BlueROV2.
* jerlov_water_filename: \<path to jerlov water properties file\>
* water_type: \<define approximate type of water the image was taken in\> <br><br>

* method: <0: A Revised Underwater Image Formation Model>
* optimize: <true: optimize attenuation values in depth range | false: calculate attenuation values per image frame> (currently not implemented)
* color_1_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* color_2_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* est_veiling_light: <true: uses background sample to calculate average wideband veiling light | false: calculate wideband veiling light> <br><br>
* background_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]

* show_image: \<true/false: shows raw and color corrected image\>
* check_time: \<true/false: track and print to screen time latency at different points\>
* log_screen: \<true/false: log to screen debug messages\> <br><br>

* save_data: <true/false: attenuation values saved to 'output_filename' or not>
* prior_data: <true/false: attenuation values used from 'input_filename' or not>
* output_filename: \<xml file to save attenuation values with its depth measurement\>
* input_filename: \<xml file to load attenuation values with its depth measurement\>

<br><br>
Note: `optimize` option will not color correct any images. Instead it will go through and calculate optimized attenuation values,
which can then be used later through the `prior` data option.

<br>
`config/ros_config.yaml`:
* camera_topic: \<topic name for the camera image messages\>
* depth_topic: \<topic name for the altitude depth messages\> <br><br>

* distance: \<from the camera to the object of interest, in meters\>
* camera_response_filename: \<path to camera response file\>
  * `Sony_IMX322LQJ-C_Camera_Response.csv` is the USB camera used on the BlueROV2.
* jerlov_water_filename: \<path to jerlov water properties file\>
* water_type: \<define approximate type of water the image was taken in\> <br><br>

* method: <0: A Revised Underwater Image Formation Model>
* optimize: <true: optimize attenuation values in depth range | false: calculate attenuation values per image frame>
* slam_input: <true/false: distance values are used from monocular ORB-SLAM features\>
* color_1_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* color_2_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* est_veiling_light: <true: uses background sample to calculate average wideband veiling light | false: calculate wideband veiling light> <br><br>
* background_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]

* show_image: \<true/false: shows raw and color corrected image\>
* check_time: \<true/false: track and print to screen time latency at different points\>
* log_screen: \<true/false: log to screen debug messages\> <br><br>

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
