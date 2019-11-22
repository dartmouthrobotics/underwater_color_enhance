/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_METHOD_H
#define UNDERWATER_COLOR_ENHANCE_METHOD_H

#include "underwater_color_enhance/Scene.h"

#include <opencv2/opencv.hpp>
#include <tinyxml.h>
#include <string>
#include <vector>

namespace underwater_color_enhance
{

/** Method handler class.
 *  Handles the universal parameters and funcitons for any color enhancement method.
 */

class Method
{
public:
  /** Constructor
   */
  Method() {}
  ~Method() {}

  bool EST_VEILING_LIGHT; /**< true: estimate as background color in image; false: calculate */
  bool OPTIMIZE;  /**< true: optimize attenuation values; false: calculate per image */
  Scene *scene;   /**< contains the physical underwater properties. */
  float depth;    /**< current altitude depth measurement. */

  bool PRIOR_DATA;  /**< true: use data that is loaded. false: calculate attenuation values */

  std::clock_t begin;
  std::clock_t end;
  bool CHECK_TIME;  /**< true: track and print time latencies. false: do not */

  bool LOG_SCREEN;  /**< true: print log statements to screen. false: do not */

  /** Parameters used for writing attenuation values to file.
   */
  TiXmlDocument out_doc;
  bool SAVE_DATA; /**< true: write attenuation values. false: do not */
  bool file_initialized;

  /** Functions for applying the color enhancement method
   */
  virtual cv::Mat color_correct(cv::Mat& img) = 0;
  virtual cv::Mat color_correct_slam(cv::Mat& img, std::vector<cv::Point2f> point_data,
    std::vector<float> distance_data) = 0;

  /** Functions for handling file reading/loading/closing.
   */
  virtual void end_file(std::string output_filename) = 0;
  virtual void load_data(std::string input_filename) = 0;

private:
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_METHOD_H
