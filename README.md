# Underwater Color Enhance Methods

NOTE: This is a C++ version. Original is in python, thus this is a work in progress.

Implementing image correction model proposed in:

"A Revised Underwater Image Formation Model"  
CVPR 2018  
Derya Akkaynak and Tall Treibitz  
University of Haifa

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
* Wideband veiling light [FUTURE]:
  * Calculate average background pixel using image processing
  * Calculate using known camera and water properties (EXTRA)

ROS Application
* image_correct file for ROS
* new yaml file for ROS application
* implement with and without SLAM
* add optimization
* check if is_adaptive is needed

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
cd underwater_color_enhance
```

Build:

```
chmod +x build.sh
./build.sh
```

## Configuration

`image_config.yaml`:
* image <path to singular input image>

* distance: <from the camera to the object of interest, in meters>
* depth: <positive value, in meters>
* background_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* color_1_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]
* color_2_sample: [\<x-coordinate\>, \<y-coordinate\>, \<width of region\>, \<height of region\>]

* method: <0: A Revised Underwater Image Formation Model>
* is_adaptive: <???> (currently: false)
* optimize: <???> (currently: false)
* est_veiling_light: <true: uses background sample to calculate average wideband veiling light | false: TO DO>

* show_image: <true: shows color corrected image | false: does not show color corrected image>

* save_data: <true/false: attenuation values saved to 'output_filename' or not>
* prior_data: <true/false: attenuation values used from 'input_filename' or not>
* output_filename: <xml file to save attenuation values with its depth measurement>
* input_filename: <xml file to load attenuation values with its depth measurement>

## Run

In the `underwater_color_enhance` repository:

For one image, based on the parameters in `image_config.yaml`:

```
./Options/Image/image_correct
```
