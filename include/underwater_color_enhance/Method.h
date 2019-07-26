/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_METHOD_H
#define UNDERWATER_COLOR_ENHANCE_METHOD_H

#include "underwater_color_enhance/Scene.h"

#include <opencv2/opencv.hpp>
#include <tinyxml.h>
#include <string>

namespace underwater_color_enhance
{

class Method
{
public:
  Method() {}
  ~Method() {}

  bool est_veiling_light;
  Scene scene;

  bool prior_data;

  bool check_time;

  TiXmlDocument out_doc;
  bool save_data;
  bool file_initialized;

  virtual cv::Mat color_correct(cv::Mat& img) = 0;
  virtual void end_file(std::string output_filename) = 0;

  virtual void load_data(std::string input_filename) = 0;

private:
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_METHOD_H
