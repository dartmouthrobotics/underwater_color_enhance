/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/Scene.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

namespace underwater_color_enhance
{

void Scene::set_depth(float new_depth)
{
  new_depth = fabs(new_depth);
  new_depth = roundf(new_depth * 100) / 100;

  // Errors occur if depth is 0
  if (0.0 == new_depth)
  {
    new_depth = this->MIN_DEPTH;
  }

  // Update depth if new
  if (this->depth != new_depth)
  {
    this->depth = new_depth;
    this->reset_data();  // Update irradiance and veiling light with new depth
  }
}


/** Update irradiance and veiling light because of a new depth
 */
void Scene::reset_data()
{
  for (int i = 0; i < this->K_d.size(); i++)
  {
    this->irradiance[i] = (this->IRRADIANCE_0 * exp(-this->K_d[i] * this->depth));
    this->veiling_light[i] = ((this->b_sca[i] * this->irradiance[i]) / this->b_att[i]);
  }
}


void Scene::load_camera_response_data(std::string CAMERA_RESPONSE_FILENAME)
{
  std::string line;
  std::vector<std::string> result;

  std::ifstream myfile(CAMERA_RESPONSE_FILENAME);
  if (myfile)
  {
    while (std::getline(myfile, line, '\r'))
    {
      boost::split(result, line, boost::is_any_of(","), boost::token_compress_on);
      if (isdigit(result[0][0]) && stoi(result[0]) % 50 == 0)
      {
        cv::Scalar sub = {stof(result[3]), stof(result[2]), stof(result[1]), 0.0};
        this->camera_response.push_back(sub);
      }
    }
  }
}


void Scene::load_jerlov_water_data(std::string JERLOV_WATER_FILENAME, std::string WATER_TYPE)
{
  std::string line;
  std::vector<std::string> result;

  std::ifstream myfile(JERLOV_WATER_FILENAME);
  if (myfile)
  {
    bool at_correct_jerlov = false;

    while (std::getline(myfile, line, '\r'))
    {
      boost::split(result, line, boost::is_any_of(","), boost::token_compress_on);

      // Completed going through the requested water type data
      if (!isdigit(result[0][0]) && at_correct_jerlov)
      {
        at_correct_jerlov = false;
        break;
      }

      // At requested water type data
      if (at_correct_jerlov && isdigit(result[0][0]) && stoi(result[0]) % 50 == 0)
      {
        this->K_d.push_back(stof(result[1]));
        this->b_abs.push_back(stof(result[2]));
        this->b_sca.push_back(stof(result[3]));
        this->b_att.push_back(this->b_abs.back() + this->b_sca.back());
        this->irradiance.push_back(this->IRRADIANCE_0 * exp(-this->K_d.back() * this->depth));
        this->veiling_light.push_back((this->b_sca.back() * this->irradiance.back()) / this->b_att.back());
      }

      // Found requested water type data
      if (result[0] == WATER_TYPE)
      {
        at_correct_jerlov = true;
      }
    }
  }
}

}  // namespace underwater_color_enhance
