#pragma once
#include "Method.h"

#include <opencv2/opencv.hpp>

using namespace cv;

class NewModel : public Method {
public:
  NewModel(){};

  Mat color_correct( Mat& img ) override;

private:
  int white_truth [3] = {242, 243, 243};
  int black_truth [3] = {52, 52, 52};

  float backscatter_att [3];
  float direct_signal_att [3];

  void calc_wideband_veiling_light( int *wideband_veiling_light[] );
  void calc_attenuation( int white_obs[], int black_obs[], int wideband_veiling_light[] );
  void est_attenuation();

};
