#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <tinyxml.h>

#include <stdlib.h>
#include <ctime>

#include "Scene.h"
#include "ColorCorrect.h"


int main(int argc, char* argv[])
{
  YAML::Node config = YAML::LoadFile("image_config.yaml");

  const std::string image_file = config["image"].as<std::string>();

  cv::Mat img = cv::imread(image_file);

  float distance = config["distance"].as<float>();
  double depth = config["depth"].as<double>();
  int method = config["method"].as<float>();

  // WHAT??
  bool is_adaptive = config["is_adaptive"].as<bool>();

  // NOTE: if optimized then we need to provide the current attenuation values
  bool optimize = config["optimize"].as<bool>();

  bool est_veiling_light = config["est_veiling_light"].as<bool>();

  bool show_image = config["show_image"].as<bool>();

  bool save_data = config["save_data"].as<bool>();
  bool prior_data = config["prior_data"].as<bool>();
  const std::string output_filename = config["output_filename"].as<std::string>();
  const std::string input_filename = config["input_filename"].as<std::string>();

  std::vector<int> background_sample = config["background_sample"].as<std::vector<int>>();
  std::vector<int> color_1_sample = config["color_1_sample"].as<std::vector<int>>();
  std::vector<int> color_2_sample = config["color_2_sample"].as<std::vector<int>>();

  Scene underwater_scene;
  underwater_scene.distance = distance;
  underwater_scene.depth = depth;
  underwater_scene.background_sample = background_sample;
  underwater_scene.color_1_sample = color_1_sample;
  underwater_scene.color_2_sample = color_2_sample;

  // std::map<double,std::vector<double>> *att_map = new std::map<double, std::vector<double>>();
  //
  // if(!prior_data)
  // {
  //   att_map = retrieve_data(input_filename);
  //   // std::cout << (*att_map)[depth][0] << std::endl;
  // }

  std::clock_t begin = clock();

  ColorCorrect correction_method(underwater_scene, method, is_adaptive, est_veiling_light, optimize, save_data, prior_data, input_filename);

  cv::Mat corrected_frame = correction_method.enhance(img);

  std::clock_t end = clock();

  // if(save_data)
  // {
  //   correction_method.save_final_data(output_filename);
  // }

  std::cout << "Total time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  if(show_image)
  {
    cv::imshow("Original", img);
    cv::imshow("Corrected", corrected_frame);
    cv::waitKey(0);
  }

  return 0;
}
