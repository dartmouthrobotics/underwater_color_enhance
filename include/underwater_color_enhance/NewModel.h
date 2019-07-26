/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_NEWMODEL_H
#define UNDERWATER_COLOR_ENHANCE_NEWMODEL_H

#include "underwater_color_enhance/Method.h"

#include <map>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace underwater_color_enhance
{

class NewModel : public Method
{
public:
  NewModel() {}
  ~NewModel() {}

  cv::Mat color_correct(cv::Mat& img) override;
  void end_file(std::string output_filename) override;
  void load_data(std::string input_filename) override;

private:
  int color_1_truth [3] = {242, 243, 243};  // white
  int color_2_truth [3] = {52, 52, 52};     // black

  float backscatter_att [3];
  float direct_signal_att [3];

  std::map<double, std::vector<double>> att_map;

  cv::Scalar calc_wideband_veiling_light();
  void calc_attenuation(cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light);
  void est_attenuation();

  void initialize_file();
  void set_data_to_file();
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_NEWMODEL_H
