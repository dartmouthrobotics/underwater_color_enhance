#ifndef COLORCORRECT_H
#define COLORCORRECT_H

#include "Scene.h"
#include "Method.h"

#include <opencv2/opencv.hpp>

class ColorCorrect {
public:
  ColorCorrect(){};
  ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool est_veiling_light, bool optimize, bool save_data, bool prior_data, std::string input_filename);

  cv::Mat enhance(cv::Mat& img);
  void save_final_data(std::string output_filename);

private:
  Scene underwater_scene;
  Method *method;

  bool is_adaptive;
  bool optimize;
};

#endif  // COLORCORRECT_H
