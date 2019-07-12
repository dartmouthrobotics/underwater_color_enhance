#ifndef NEWMODEL_H
#define NEWMODEL_H

#include "Method.h"

#include <map>
#include <vector>

#include <opencv2/opencv.hpp>

class NewModel : public Method {
public:
  NewModel(){};

  cv::Mat color_correct( cv::Mat& img ) override;
  void end_file(std::string output_filename) override;
  void load_data(std::string input_filename) override;

private:
  int color_1_truth [3] = {242, 243, 243};  // white
  int color_2_truth [3] = {52, 52, 52};     // black

  float backscatter_att [3];
  float direct_signal_att [3];

  std::map<double, std::vector<double>> att_map;

  cv::Scalar calc_wideband_veiling_light();
  void calc_attenuation( cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light );
  void est_attenuation();

  void initialize_file();
  void set_data_to_file();

};

#endif  // NEWMODEL_H
