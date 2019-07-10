#ifndef NEWMODEL_H
#define NEWMODEL_H

#include "Method.h"

#include <opencv2/opencv.hpp>

class NewModel : public Method {
public:
  NewModel(){};

  cv::Mat color_correct( cv::Mat& img ) override;
  // cv::Mat color_correct2( cv::Mat& img );

private:
  int white_truth [3] = {242, 243, 243};
  int black_truth [3] = {52, 52, 52};

  float backscatter_att [3];
  float direct_signal_att [3];

  cv::Scalar calc_wideband_veiling_light();
  void calc_attenuation( int white_obs[], int black_obs[], cv::Scalar wideband_veiling_light );
  void est_attenuation();

};

#endif  // NEWMODEL_H
