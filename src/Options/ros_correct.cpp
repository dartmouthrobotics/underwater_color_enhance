/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <tinyxml.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <stdlib.h>
#include <iostream>

#include <ctype.h>
#include <ctime>
#include <string>
#include <vector>

#include "underwater_color_enhance/Scene.h"
#include "underwater_color_enhance/ColorCorrect.h"
#include "underwater_color_enhance/ImageHandler.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_color_enhance");

  // Load configuration file
  std::string path = ros::package::getPath("underwater_color_enhance") + argv[1];
  YAML::Node config = YAML::LoadFile(path);

  // ROS topics for imagery and depth values
  std::string CAMERA_TOPIC = config["camera_topic"].as<std::string>();
  std::string DEPTH_TOPIC = config["depth_topic"].as<std::string>();

  // Scene properties: distance to object of interest in image and depth in water
  // NOTE: distance will not be used in cases when SLAM features are integrated
  float DISTANCE = config["distance"].as<float>();

  std::string CAMERA_RESPONSE_FILENAME = config["camera_response_filename"].as<std::string>();
  std::string JERLOV_WATER_FILENAME = config["jerlov_water_filename"].as<std::string>();
  std::string WATER_TYPE = config["water_type"].as<std::string>();

  // Color enhancement method
  int METHOD_ID = config["method_id"].as<int>();

  // TO DO: if optimized then we need to provide the current attenuation values
  bool OPTIMIZE = config["optimize"].as<bool>();

  // Check if SLAM features will be used
  bool SLAM_INPUT = config["slam_input"].as<bool>();

  // Color patch locations if using color chart
  std::vector<int> COLOR_1_SAMPLE = config["color_1_sample"].as<std::vector<int>>();
  std::vector<int> COLOR_2_SAMPLE = config["color_2_sample"].as<std::vector<int>>();

  // Wideband veiling light: estimated (true) or calculated (false)
  bool EST_VEILING_LIGHT = config["est_veiling_light"].as<bool>();

  // TO DO: Instead use image processing to calculate average background color
  std::vector<int> BACKGROUND_SAMPLE = config["background_sample"].as<std::vector<int>>();

  // Other checks
  bool SHOW_IMAGE = config["show_image"].as<bool>();
  bool CHECK_TIME = config["check_time"].as<bool>();
  bool LOG_SCREEN = config["log_screen"].as<bool>();

  bool SAVE_DATA = config["save_data"].as<bool>();
  bool PRIOR_DATA = config["prior_data"].as<bool>();
  std::string OUTPUT_FILENAME = ros::package::getPath("underwater_color_enhance") + "/" +
    config["output_filename"].as<std::string>();
  std::string INPUT_FILENAME = ros::package::getPath("underwater_color_enhance") + "/" +
    config["input_filename"].as<std::string>();

  if (LOG_SCREEN)
  {
    std::cout << "LOG: Configuration file loading complete" << std::endl;
  }

  // Underwater scene
  underwater_color_enhance::Scene underwater_scene;
  underwater_scene.DISTANCE = DISTANCE;
  underwater_scene.COLOR_1_SAMPLE = COLOR_1_SAMPLE;
  underwater_scene.COLOR_2_SAMPLE = COLOR_2_SAMPLE;
  // TO DO: unsure if this is required
  underwater_scene.set_depth(0.01);   // For simplicity set an initial value

  if (EST_VEILING_LIGHT)   // Wideband veiling light assumed to be the average background color
  {
    underwater_scene.BACKGROUND_SAMPLE = BACKGROUND_SAMPLE;
  }
  else  // Wideband veiling light calculated using camera response values and jerlov waters
  {
    underwater_scene.load_camera_response_data(ros::package::getPath("underwater_color_enhance") +
      "/Camera_Response_Files/" + CAMERA_RESPONSE_FILENAME);
    underwater_scene.load_jerlov_water_data(ros::package::getPath("underwater_color_enhance") +
      "/Jerlov_Water/" + JERLOV_WATER_FILENAME, WATER_TYPE);
  }

  // TO DO: If we have SLAM, do not optimize the attenuation values
  if (SLAM_INPUT)
  {
    OPTIMIZE = false;
  }

  if (LOG_SCREEN)
  {
    std::cout << "LOG: Scene set up comlete" << std::endl;
  }

  // Initialize color correction method
  underwater_color_enhance::ColorCorrect correction_method(underwater_scene, METHOD_ID,
    EST_VEILING_LIGHT, OPTIMIZE, SAVE_DATA, CHECK_TIME, LOG_SCREEN, PRIOR_DATA, INPUT_FILENAME, OUTPUT_FILENAME);

  if (LOG_SCREEN)
  {
    std::cout << "LOG: Enhancement complete" << std::endl;
    std::cout << "LOG: Begin enhancing image" << std::endl;
  }

  underwater_color_enhance::ImageHandler image_scene_handler(correction_method, SLAM_INPUT, SAVE_DATA,
    SHOW_IMAGE, CHECK_TIME, CAMERA_TOPIC, DEPTH_TOPIC);

  ros::spin();

  return 0;
}
