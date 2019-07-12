#include "ColorCorrect.h"

#include "NewModel.h"

ColorCorrect::ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool est_veiling_light, bool optimize, bool save_data, bool prior_data, std::string input_filename)
{
  this->underwater_scene = new_scene;
  this->is_adaptive = is_adaptive;
  // this->est_veiling_light = est_veiling_light;
  this->optimize = optimize;

  if(new_method == 0)
  {
    this->method = new NewModel;
    this->method->est_veiling_light = est_veiling_light;
    this->method->save_data = save_data;
    this->method->prior_data = prior_data;
    this->method->input_filename = input_filename;
    this->method->file_initialized = false;
    this->method->scene = this->underwater_scene;

    if(prior_data)
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


void ColorCorrect::save_final_data(std::string output_filename)
{
  this->method->end_file(output_filename);
}
