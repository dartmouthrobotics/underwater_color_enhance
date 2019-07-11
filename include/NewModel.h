#ifndef NEWMODEL_H
#define NEWMODEL_H

#include "Method.h"

#include <opencv2/opencv.hpp>

class NewModel : public Method {
public:
  NewModel(){};

  cv::Mat color_correct( cv::Mat& img ) override;

private:
  int color_1_truth [3] = {242, 243, 243};  // white
  int color_2_truth [3] = {52, 52, 52};     // black

  float backscatter_att [3];
  float direct_signal_att [3];

  cv::Scalar calc_wideband_veiling_light();
  void calc_attenuation( cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light );
  void est_attenuation();

  void initialize_file();
  void set_data_to_file();
  void end_file();

};

#endif  // NEWMODEL_H
