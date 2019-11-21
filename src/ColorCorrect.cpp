/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/ColorCorrect.h"

#include "underwater_color_enhance/NewModel.h"

#include <string>
#include <vector>

namespace underwater_color_enhance
{

ColorCorrect::ColorCorrect(Scene& underwater_scene, int METHOD_ID, bool EST_VEILING_LIGHT, bool OPTIMIZE,
  bool SAVE_DATA, bool CHECK_TIME, bool LOG_SCREEN, bool PRIOR_DATA, std::string INPUT_FILENAME,
  std::string OUTPUT_FILENAME)
{
  this->underwater_scene = underwater_scene;
  this->OPTIMIZE = OPTIMIZE;
  this->OUTPUT_FILENAME = OUTPUT_FILENAME;

  if (METHOD_ID == 0)
  {
    this->method = new NewModel;
    this->method->EST_VEILING_LIGHT = EST_VEILING_LIGHT;
    this->method->SAVE_DATA = SAVE_DATA;
    this->method->CHECK_TIME = CHECK_TIME;
    this->method->PRIOR_DATA = PRIOR_DATA;
    this->method->LOG_SCREEN = LOG_SCREEN;
    this->method->file_initialized = false;
    this->method->scene = &this->underwater_scene;
    this->method->depth = this->underwater_scene.get_depth();

    if (PRIOR_DATA)
    {
      this->method->load_data(INPUT_FILENAME);
    }
  }
  else
  {
    this->method = 0;
  }
}


cv::Mat ColorCorrect::enhance(cv::Mat& img)
{
  this->method->depth = this->underwater_scene.get_depth();
  cv::Mat corrected_img = this->method->color_correct(img);

  return corrected_img;
}


cv::Mat ColorCorrect::enhance_slam(cv::Mat& img, std::vector<cv::Point2f> point_data, std::vector<float> distance_data)
{
  this->method->depth = this->underwater_scene.get_depth();
  cv::Mat corrected_img = this->method->color_correct_slam(img, point_data, distance_data);

  return corrected_img;
}


void ColorCorrect::save_final_data()
{
  this->method->end_file(this->OUTPUT_FILENAME);
}

}  // namespace underwater_color_enhance
