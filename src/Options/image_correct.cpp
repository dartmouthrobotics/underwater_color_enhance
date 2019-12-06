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
  // ROOT_PATH environment variable, prefix for all files.
  const char* ROOT_PATH = std::getenv("ROOT_PATH");
  // Load configuration file
  YAML::Node config = YAML::LoadFile(argv[1]);

  // Single image to color enhance
  const std::string IMAGE_FILE = std::string(ROOT_PATH) + "/" + config["image"].as<std::string>();

  // Scene properties: distance to object of interest in image and depth in water
  float DISTANCE = config["distance"].as<float>();
  double DEPTH = config["depth"].as<double>();

  std::string CAMERA_RESPONSE_FILENAME = std::string(ROOT_PATH) + "/" + config["camera_response_filename"].as<std::string>();
  std::string JERLOV_WATER_FILENAME = std::string(ROOT_PATH) + "/" + config["jerlov_water_filename"].as<std::string>();
  std::string WATER_TYPE = config["water_type"].as<std::string>();

  // Color enhancement method
  int METHOD_ID = config["method_id"].as<int>();

  // Optimized option is unnecessary in single image color correction
  bool OPTIMIZE = false;
  float RANGE = -1.0;

  // Color patch locations if using color chart
  std::vector<int> COLOR_1_SAMPLE = config["color_1_sample"].as<std::vector<int>>();
  std::vector<int> COLOR_2_SAMPLE = config["color_2_sample"].as<std::vector<int>>();

  // Wideband veiling light: estimated (true) or calculated (false)
  bool EST_VEILING_LIGHT = config["est_veiling_light"].as<bool>();

  // TO DO: Instead use image processing to calculate the average background color
  std::vector<int> BACKGROUND_SAMPLE = config["background_sample"].as<std::vector<int>>();

  // Other checks
  bool SHOW_IMAGE = config["show_image"].as<bool>();
  bool CHECK_TIME = config["check_time"].as<bool>();
  bool LOG_SCREEN = config["log_screen"].as<bool>();

  bool SAVE_DATA = config["save_data"].as<bool>();
  bool PRIOR_DATA = config["prior_data"].as<bool>();
  const std::string OUTPUT_FILENAME = std::string(ROOT_PATH) + "/" + config["output_filename"].as<std::string>();
  const std::string INPUT_FILENAME = std::string(ROOT_PATH) + "/" + config["input_filename"].as<std::string>();

  if (LOG_SCREEN)
  {
    std::cout << "LOG: Configuration file loading complete" << std::endl;
  }

  // Image to color enhance
  cv::Mat image = cv::imread(IMAGE_FILE);

  // Underwater scene
  underwater_color_enhance::Scene underwater_scene;
  underwater_scene.DISTANCE = DISTANCE;
  underwater_scene.COLOR_1_SAMPLE = COLOR_1_SAMPLE;
  underwater_scene.COLOR_2_SAMPLE = COLOR_2_SAMPLE;

  if (EST_VEILING_LIGHT)  // Wideband veiling lgiht assumed to be the average background color
  {
    underwater_scene.BACKGROUND_SAMPLE = BACKGROUND_SAMPLE;
  }
  else  // Wideband veiling light calculated using camera response values and jerlov waters
  {
    underwater_scene.load_camera_response_data(CAMERA_RESPONSE_FILENAME);
    underwater_scene.load_jerlov_water_data(JERLOV_WATER_FILENAME, WATER_TYPE);
    underwater_scene.set_depth(static_cast<float>(DEPTH));
  }

  if (LOG_SCREEN)
  {
    std::cout << "LOG: Scene set up comlete" << std::endl;
  }

  // Initialize color correction method
  underwater_color_enhance::ColorCorrect correction_method(underwater_scene, METHOD_ID,
    EST_VEILING_LIGHT, OPTIMIZE, RANGE, SAVE_DATA, CHECK_TIME, LOG_SCREEN, PRIOR_DATA,
    INPUT_FILENAME, OUTPUT_FILENAME);

  if (LOG_SCREEN)
  {
    std::cout << "LOG: Enhancement method initialization complete" << std::endl;
  }

  std::clock_t begin;
  std::clock_t end;
  if (CHECK_TIME)
  {
    begin = clock();
  }

  cv::Mat corrected_frame = correction_method.enhance(image);

  if (CHECK_TIME)
  {
    end = clock();
    std::cout << "LOG: Enhancement complete. Total time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
  }
  else if (LOG_SCREEN)
  {
    std::cout << "LOG: Enhancement complete" << std::endl;
  }

  if (SAVE_DATA)
  {
    correction_method.save_final_data();
  }

  if (SHOW_IMAGE)
  {
    cv::imshow("Original", image);
    cv::imshow("Corrected", corrected_frame);
    cv::waitKey(0);
  }

  return 0;
}
