/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/ColorCorrect.h"

#include "underwater_color_enhance/NewModel.h"

#include <string>

namespace underwater_color_enhance
{

ColorCorrect::ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool est_veiling_light,
  bool optimize, bool save_data, bool check_time, bool prior_data, std::string input_filename,
  std::string output_filename)
{
  this->underwater_scene = new_scene;
  this->is_adaptive = is_adaptive;
  this->optimize = optimize;
  this->output_filename = output_filename;

  if (new_method == 0)
  {
    this->method = new NewModel;
    this->method->est_veiling_light = est_veiling_light;
    this->method->save_data = save_data;
    this->method->check_time = check_time;
    this->method->prior_data = prior_data;
    this->method->file_initialized = false;
    this->method->scene = this->underwater_scene;

    if (prior_data)
    {
      this->method->load_data(input_filename);
    }
  }
  else
  {
    this->method = 0;
  }
}

cv::Mat ColorCorrect::enhance(cv::Mat& img)
{
  cv::Mat corrected_img = this->method->color_correct(img);

  return corrected_img;
}


void ColorCorrect::save_final_data()
{
  this->method->end_file(this->output_filename);
}

}  // namespace underwater_color_enhance
