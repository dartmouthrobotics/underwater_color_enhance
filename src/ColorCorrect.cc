#include "ColorCorrect.h"
#include "NewModel.h"

ColorCorrect::ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool optimize)
{
  this->underwater_scene = new_scene;
  this->is_adaptive = is_adaptive;
  this->optimize = optimize;

  if(new_method == 0)
  {
    this->method = new NewModel;
    this->method->prior = false;
    this->method->scene = this->underwater_scene;
  }
  else
  {
    this->method = 0;
  }

}

Mat ColorCorrect::enhance(Mat& img)
{
  Mat corrected_img = this->method->color_correct(img);

  return corrected_img;
}
