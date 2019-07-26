/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <tinyxml.h>

#include <stdlib.h>
#include <ctime>
#include <string>
#include <vector>

#include "underwater_color_enhance/Scene.h"
#include "underwater_color_enhance/ColorCorrect.h"


int main(int argc, char* argv[])
{
  // TO DO: check if file exists, otherwise send error
  YAML::Node config = YAML::LoadFile(argv[1]);

  const std::string image_file = config["image"].as<std::string>();

  cv::Mat img = cv::imread(image_file);

  float distance = config["distance"].as<float>();
  double depth = config["depth"].as<double>();
  int method = config["method"].as<float>();

  // TO DO: WHAT??
  bool is_adaptive = config["is_adaptive"].as<bool>();

  // TO DO: if optimized then we need to provide the current attenuation values
  bool optimize = config["optimize"].as<bool>();

  bool est_veiling_light = config["est_veiling_light"].as<bool>();

  bool show_image = config["show_image"].as<bool>();
  bool check_time = config["check_time"].as<bool>();
  bool save_data = config["save_data"].as<bool>();
  bool prior_data = config["prior_data"].as<bool>();
  const std::string output_filename = config["output_filename"].as<std::string>();
  const std::string input_filename = config["input_filename"].as<std::string>();

  std::vector<int> background_sample = config["background_sample"].as<std::vector<int>>();
  std::vector<int> color_1_sample = config["color_1_sample"].as<std::vector<int>>();
  std::vector<int> color_2_sample = config["color_2_sample"].as<std::vector<int>>();

  std::cout << "LOG: Configuration file loading complete" << std::endl;

  underwater_color_enhance::Scene underwater_scene;
  underwater_scene.distance = distance;
  underwater_scene.depth = depth;
  underwater_scene.background_sample = background_sample;
  underwater_scene.color_1_sample = color_1_sample;
  underwater_scene.color_2_sample = color_2_sample;

  std::cout << "LOG: Scene set up comlete" << std::endl;

  // CLOCK
  std::clock_t begin;
  std::clock_t end;
  if (check_time)
  {
    begin = clock();
  }

  underwater_color_enhance::ColorCorrect correction_method(underwater_scene, method,
    is_adaptive, est_veiling_light, optimize, save_data, check_time, prior_data, input_filename, output_filename);
  std::cout << "LOG: Enhancement method initialization complete" << std::endl;

  cv::Mat corrected_frame = correction_method.enhance(img);

  // CLOCK
  if (check_time)
  {
    end = clock();
    std::cout << "LOG: Enhancement complete. Total time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
  }
  else
  {
    std::cout << "LOG: Enhancement complete" << std::endl;
  }

  if (save_data)
  {
    correction_method.save_final_data();
  }

  if (show_image)
  {
    cv::imshow("Original", img);
    cv::imshow("Corrected", corrected_frame);
    cv::waitKey(0);
  }

  return 0;
}
