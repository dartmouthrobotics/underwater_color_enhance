/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_SCENE_H
#define UNDERWATER_COLOR_ENHANCE_SCENE_H

#include <vector>

namespace underwater_color_enhance
{

class Scene
{
public:
  float distance;
  double depth;
  std::vector<int> background_sample;
  std::vector<int> color_1_sample;
  std::vector<int> color_2_sample;

private:
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_SCENE_H
