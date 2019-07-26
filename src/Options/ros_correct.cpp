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
#include <ctime>
#include <string>
#include <vector>

#include "underwater_color_enhance/Scene.h"
#include "underwater_color_enhance/ColorCorrect.h"
#include "underwater_color_enhance/ImageHandler.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_color_enhance");

  std::string path = ros::package::getPath("underwater_color_enhance") + "/ros_config.yaml";
  YAML::Node config = YAML::LoadFile(path);

  std::string camera_topic = config["camera_topic"].as<std::string>();
  std::string depth_topic = config["depth_topic"].as<std::string>();

  float distance = config["distance"].as<float>();

  int method = config["method"].as<float>();

  // WHAT??
  bool is_adaptive = config["is_adaptive"].as<bool>();

  // NOTE: if optimized then we need to provide the current attenuation values
  bool optimize = config["optimize"].as<bool>();

  bool est_veiling_light = config["est_veiling_light"].as<bool>();

  bool show_image = config["show_image"].as<bool>();
  bool check_time = config["check_time"].as<bool>();

  bool save_data = config["save_data"].as<bool>();
  bool prior_data = config["prior_data"].as<bool>();
  const std::string output_filename = ros::package::getPath("underwater_color_enhance") +
    "/" + config["output_filename"].as<std::string>();
  const std::string input_filename = ros::package::getPath("underwater_color_enhance") +
    "/" + config["input_filename"].as<std::string>();

  std::vector<int> background_sample = config["background_sample"].as<std::vector<int>>();
  std::vector<int> color_1_sample = config["color_1_sample"].as<std::vector<int>>();
  std::vector<int> color_2_sample = config["color_2_sample"].as<std::vector<int>>();

  std::cout << "LOG: Configuration file loading complete" << std::endl;

  underwater_color_enhance::Scene underwater_scene;
  underwater_scene.distance = distance;
  underwater_scene.depth = 0.0;
  underwater_scene.background_sample = background_sample;
  underwater_scene.color_1_sample = color_1_sample;
  underwater_scene.color_2_sample = color_2_sample;

  std::cout << "LOG: Scene set up comlete" << std::endl;

  underwater_color_enhance::ColorCorrect correction_method(underwater_scene, method, is_adaptive,
    est_veiling_light, optimize, save_data, check_time, prior_data, input_filename, output_filename);

  std::cout << "LOG: Enhancement method initialization complete" << std::endl;

  std::cout << "LOG: Begin enhancing image" << std::endl;

  underwater_color_enhance::ImageHandler image_scene_handler(correction_method, save_data,
    show_image, check_time, camera_topic, depth_topic);

  ros::spin();

  return 0;
}
