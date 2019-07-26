/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_COLORCORRECT_H
#define UNDERWATER_COLOR_ENHANCE_COLORCORRECT_H

#include "underwater_color_enhance/Scene.h"
#include "underwater_color_enhance/Method.h"

#include <opencv2/opencv.hpp>
#include <string>

namespace underwater_color_enhance
{

class ColorCorrect
{
public:
  ColorCorrect() {}
  ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool est_veiling_light,
    bool optimize, bool save_data, bool check_time, bool prior_data, std::string input_filename,
    std::string output_filename);
  ~ColorCorrect() {}

  cv::Mat enhance(cv::Mat& img);
  void save_final_data();

  double get_depth() {return underwater_scene.depth;}
  void set_depth(double new_depth) {underwater_scene.depth = new_depth;}

  float get_distance() {return underwater_scene.distance;}
  void set_distance(float new_distance) {underwater_scene.distance = new_distance;}

private:
  Scene underwater_scene;
  Method *method;

  std::string output_filename;
  bool is_adaptive;
  bool optimize;
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_COLORCORRECT_H
