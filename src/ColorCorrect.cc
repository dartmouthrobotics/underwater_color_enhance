#include "ColorCorrect.h"

#include "NewModel.h"

ColorCorrect::ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool est_veiling_light, bool optimize)
{
  this->underwater_scene = new_scene;
  this->is_adaptive = is_adaptive;
  // this->est_veiling_light = est_veiling_light;
  this->optimize = optimize;

  if(new_method == 0)
  {
    this->method = new NewModel;
    this->method->prior = false;
    this->method->est_veiling_light = est_veiling_light;
    this->method->scene = this->underwater_scene;
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
