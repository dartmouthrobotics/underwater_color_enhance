#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <ctime>

#include "Scene.h"
#include "ColorCorrect.h"

int main(int argc, char* argv[])
{
  YAML::Node config = YAML::LoadFile("image_config.yaml");

  const std::string image_file = config["image"].as<std::string>();

  cv::Mat img = cv::imread(image_file);

  float distance = config["distance"].as<float>();
  float depth = config["depth"].as<float>();
  int method = config["method"].as<float>();

  // WHAT??
  bool is_adaptive = config["is_adaptive"].as<bool>();

  // NOTE: if optimized then we need to provide the current attenuation values
  bool optimize = config["optimize"].as<bool>();

  bool est_veiling_light = config["est_veiling_light"].as<bool>();

  bool show_image = config["show_image"].as<bool>();

  std::vector<int> background_sample = config["background_sample"].as<std::vector<int>>();
  // std::cout << background_sample[0] << std::endl;

  // cv::Vec3f intensity = img.at<cv::Vec3b>(background_sample[1], background_sample[0]);
  // uchar blue = intensity.val[0];
  // std::cout << blue << std::endl;
  // std::cout << intensity.val[0] << " " << intensity.val[1] << " " << intensity.val[2] << std::endl;

  Scene underwater_scene;
  underwater_scene.distance = distance;
  underwater_scene.depth = depth;
  underwater_scene.background_sample = background_sample;

  std::clock_t begin = clock();

  ColorCorrect correction_method(underwater_scene, method, is_adaptive, est_veiling_light, optimize);

  cv::Mat corrected_frame = correction_method.enhance(img);

  std::clock_t end = clock();

  std::cout << "Total time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  if(show_image)
  {
    cv::imshow("Original", img);
    cv::imshow("Corrected", corrected_frame);
    cv::waitKey(0);
  }

  return 0;
}
