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

/** New Model handler class.
 *  Handles the methodology based on the Recently Revised Underwater Image Formation Model.
 *  See paper in README.
 */

class NewModel : public Method
{
public:
  /** Constructor.
   */
  NewModel() {}
  ~NewModel() {}

  /** See functions in Method class
   */
  cv::Mat color_correct(cv::Mat& img) override;
  cv::Mat color_correct_slam(cv::Mat& img, std::vector<cv::Point2f> point_data,
    std::vector<float> distance_data) override;

  /** See functions in Method class
   */
  void end_file(std::string output_filename) override;
  void load_data(std::string input_filename) override;

private:
  int color_1_truth [3] = {242, 243, 243};  /**< White patch ground truth in BGR */
  int color_2_truth [3] = {52, 52, 52};     /**< Black patch ground turth in BGR */

  float backscatter_att [3];    /**< Contains the backscatter attenuation values for that depth */
  float direct_signal_att [3];  /**< Contains the direct signal attenuation values for that depth */

  std::map<float, std::vector<double>> att_map; /** Contains the mapping of depth to pre calculated att values */

  /** Functions for calculating or estimating parameters vital to the enhancement algorithm.
   */
  cv::Scalar calc_wideband_veiling_light();
  void calc_attenuation(cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light);
  void est_attenuation();

  /** Helper functions for preparing file usage.
   */
  void initialize_file();
  void set_data_to_file();
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_NEWMODEL_H
