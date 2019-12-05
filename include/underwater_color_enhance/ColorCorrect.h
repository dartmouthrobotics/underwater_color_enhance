/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_COLORCORRECT_H
#define UNDERWATER_COLOR_ENHANCE_COLORCORRECT_H

#include "underwater_color_enhance/Scene.h"
#include "underwater_color_enhance/Method.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace underwater_color_enhance
{

/** Calibration color correction class.
 *  Handles the set up for the choice of color correction method.
 */

class ColorCorrect
{
public:
  /** Constructor.
   *  Initializes the parameters and sets up the color correction method.
   *
   *  \param underwater_scene - see below.
   *  \param METHOD_ID decides what color enhancement method to apply.
   *      0:    NewModel
   *      else: NewModel (safety measures)
   *  \param EST_VEILING_LIGHT parameter for method object.
   *  \param OPTIMIZE - true: optimize attenuation values.
   *  \param SAVE_DATA - true: write attenuation values to file.
   *  \param CHECK_TIME - true: track and publish time periods.
   *  \param LOG_SCREEN - true: print log statements.
   *  \param PRIOR_DATA - true: calculate or load attenuaiton values.
   *  \param INPUT_FILENAME - name of the file that contains pre calculated attenuation values.
   *  \param OUTPUT_FILENAME - see below.
   */
  ColorCorrect() {}
  ColorCorrect(Scene& underwater_scene, int METHOD_ID, bool EST_VEILING_LIGHT, bool OPTIMIZE,
    bool SAVE_DATA, bool CHECK_TIME, bool LOG_SCREEN, bool PRIOR_DATA, std::string INPUT_FILENAME,
    std::string OUTPUT_FILENAME);
  ~ColorCorrect() {}

  bool OPTIMIZE;  /**< determines if this program will be calculating optimized attenuation values */

  void optimize(cv::Mat& img);

  /** Functions that lead to the current color enhancement methods.
   */
  cv::Mat enhance(cv::Mat& img);      /** requires image and depth **/
  cv::Mat enhance_slam(cv::Mat& img,       /** requires image, depth, and SLAM points **/
    std::vector<cv::Point2f> point_data, std::vector<float> distance_data);

  /** Calculated attenuation values are saved to the OUTPUT_FILENAME.
   */
  void save_final_data();

  /** Functions for retrieving and setting altitude depth measurements.
   */
  double get_depth() {return underwater_scene.get_depth();}
  void set_depth(double new_depth) {underwater_scene.set_depth(new_depth);}

private:
  Scene underwater_scene; /**< object that contains pysical scene properties over changes in depth */
  Method *method;         /**< object that contains the set up color enhancement method.*/

  std::string OUTPUT_FILENAME;  /**< name of the file that will contain the save attenuation values */
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_COLORCORRECT_H
