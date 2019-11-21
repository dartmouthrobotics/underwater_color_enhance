/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_SCENE_H
#define UNDERWATER_COLOR_ENHANCE_SCENE_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace underwater_color_enhance
{

/** Scene handler class.
 *  Handles phsyical properties of the underwater environment.
 */

class Scene
{
public:
  float DISTANCE; /**< Measured distance of the camera to the color chart. */

  std::vector<int> BACKGROUND_SAMPLE; /**< Used for estimating wideband veiling light. */

  /** Parameters used for calculating the wideband veiling light.
   */
  std::vector<cv::Scalar> camera_response;  /**< rows: wavelengths; columns: BGR */
  std::vector<float> K_d;                   /**< Diffuse downwelling attenuation coefficient. */
  std::vector<float> b_abs;                 /**< Beam absorption coefficient. */
  std::vector<float> b_sca;                 /**< Beam scattering coefficient. */
  std::vector<float> b_att;                 /**< Beam attenuation coefficient. */
  std::vector<float> irradiance;            /**< E */
  std::vector<float> veiling_light;         /**< B^inf */

  // TO DO: Set this as a parameter from a YAML file.
  float K = 0.1;                            /**< Camera image exposure and camera pixel geometry */

  /** Parameters used for conducting integral calculations.
   */
  std::vector<int> WAVELENGTHS = {400, 450, 500, 550, 600, 650, 700};
  float WAVELENGTHS_SUB = (this->WAVELENGTHS.back() - this->WAVELENGTHS[0]) / (this->WAVELENGTHS.size() - 1) / 2;

  /** Parameters if using color chart for calculating attenuation values.
   */
  std::vector<int> COLOR_1_SAMPLE;
  std::vector<int> COLOR_2_SAMPLE;

  /** Functions for handling altitude depth measurements.
   */
  void set_depth(float new_depth);
  float get_depth() {return this->depth;}

  /** Function for recalculating environment properties, which occurs when depth changes.
   */
  void reset_data();

  /** Functions for loading camera response data and jerlov water physical properties.
   */
  void load_camera_response_data(std::string CAMERA_RESPONSE_FILENAME);
  void load_jerlov_water_data(std::string JERLOV_WATER_FILENAME, std::string WATER_TYPE);

private:
  float depth = 0.01;        /**< Current altitude depth measurement. Let set_depth() handle checks. */
  float IRRADIANCE_0 = 1.0;  /**< Irradiance (E) at the surface */

  float MIN_DEPTH = 0.01;    /**< Minimum altitude depth measurement. */
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_SCENE_H
