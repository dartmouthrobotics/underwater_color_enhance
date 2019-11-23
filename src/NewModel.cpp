/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/NewModel.h"

#include <math.h>
#include <opencv2/opencv.hpp>
#include <dlib/optimization.h>
#include <utility>
#include <string>
#include <vector>

namespace underwater_color_enhance
{

/** No SLAM implementation
 */
cv::Mat NewModel::color_correct(cv::Mat& img)
{
  if (this->CHECK_TIME)
  {
    this->begin = clock();
  }

  // Split BGR image to a Mat array of each color channel
  cv::Mat bgr[3];
  split(img, bgr);

  std::vector<cv::Mat> corrected_bgr(3);

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Set image for processing complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Set image for processing complete" << std::endl;
  }

  // Calculate or estimate wideband veiling light
  cv::Scalar wideband_veiling_light;
  // FUTURE: average wideband veiling light be calculates using image processing techniques
  if (this->EST_VEILING_LIGHT)  // Estimate wideband veiling light as average background value
  {
    cv::Rect region_of_interest(this->scene->BACKGROUND_SAMPLE[0], this->scene->BACKGROUND_SAMPLE[1],
      this->scene->BACKGROUND_SAMPLE[2], this->scene->BACKGROUND_SAMPLE[3]);
    cv::Mat background = img(region_of_interest);

    // TO DO: this mean is done independently for each channel
    // Should I take the average pixel color instead?
    wideband_veiling_light = mean(background);
  }
  else
  {
    wideband_veiling_light = calc_wideband_veiling_light();
  }

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Veiling light calculation complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Veiling light calculation complete" << std::endl;
  }

  if (this->PRIOR_DATA)  // Use prior data to retrieve backscatter and direct signal attenauation values
  {
    est_attenuation();
  }
  else  // Must calculate the attenuation values using a color chart
  {
    // TO DO: Could have these rectangles initialized ahead of time
    // Create rectangles of the regions of interest
    cv::Rect patch_1_region(this->scene->COLOR_1_SAMPLE[0], this->scene->COLOR_1_SAMPLE[1],
      this->scene->COLOR_1_SAMPLE[2], this->scene->COLOR_1_SAMPLE[3]);
    cv::Rect patch_2_region(this->scene->COLOR_2_SAMPLE[0], this->scene->COLOR_2_SAMPLE[1],
       this->scene->COLOR_2_SAMPLE[2], this->scene->COLOR_2_SAMPLE[3]);
    // Splice regions from the image
    cv::Mat color_1_region = img(patch_1_region);
    cv::Mat color_2_region = img(patch_2_region);

    // TO DO: this mean is done independently for each channel. Should I take the average pixel color instead?
    // mean pixel value of observed colors
    cv::Scalar color_1_obs = mean(color_1_region);
    cv::Scalar color_2_obs = mean(color_2_region);

    calc_attenuation(color_1_obs, color_2_obs, wideband_veiling_light);

    if (this->OPTIMIZE)
    {
      std::cout << "LOG WARNING: Optimization is not currently implemented." << std::endl;
      // if (this->depth < this->depth_max_range && this->depth > this->depth_max_range - this->RANGE)
      // {
        // opt_vector observed_input;

        // std::vector<std::pair<opt_vector, double>> observed_samples_green;
        // std::vector<std::pair<opt_vector, double>> observed_samples_red;

        // observed_input(0) = (double)color_1_obs[0];
        // observed_input(1) = (double)wideband_veiling_light[0];
        //
        // this->observed_samples_blue.push_back(std::make_pair(observed_input, this->COLOR_1_TRUTH[0]));
        //
        // observed_input(0) = (double)color_2_obs[0];
        //
        // this->observed_samples_blue.push_back(std::make_pair(observed_input, this->COLOR_2_TRUTH[0]));

        // observed_input(0) = (double)color_1_obs[1];
        // observed_input(1) = (double)wideband_veiling_light[1];
        //
        // observed_samples_green.push_back(std::make_pair(this->observed_input, this->COLOR_1_TRUTH[1]));
        //
        // this->observed_input(0) = (double)color_2_obs[1];
        //
        // this->observed_samples_green.push_back(std::make_pair(this->observed_input, this->COLOR_2_TRUTH[1]));
        //
        // this->observed_input(0) = (double)color_1_obs[2];
        // this->observed_input(1) = (double)wideband_veiling_light[2];
        //
        // this->observed_samples_red.push_back(std::make_pair(this->observed_input, this->COLOR_1_TRUTH[2]));
        //
        // this->observed_input(0) = (double)color_2_obs[2];
        //
        // this->observed_samples_red.push_back(std::make_pair(this->observed_input, this->COLOR_2_TRUTH[2]));
      // }
      // else
      // {
      //   opt_vector optimized_att_blue;
      //   optimized_att_blue = 1;
      //   dlib::solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
      //                                 &NewModel::residual,
      //                                 dlib::derivative(&NewModel::residual),
      //                                 &NewModel::observed_samples_blue,
      //                                 optimized_att_blue);
      // }
    }
  }

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Attenuation calculation complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Attenuation calculation complete" << std::endl;
  }

  // Calculate backscatter and direct signal values
  float blue_backscatter_val = 1.0 - exp(-1.0 * this->backscatter_att[0] * this->scene->DISTANCE);
  float green_backscatter_val = 1.0 - exp(-1.0 * this->backscatter_att[1] * this->scene->DISTANCE);
  float red_backscatter_val = 1.0 - exp(-1.0 * this->backscatter_att[2] * this->scene->DISTANCE);

  float blue_direct_signal_val = exp(-1.0 * this->direct_signal_att[0] * this->scene->DISTANCE);
  float green_direct_signal_val = exp(-1.0 * this->direct_signal_att[1] * this->scene->DISTANCE);
  float red_direct_signal_val = exp(-1.0 * this->direct_signal_att[2] * this->scene->DISTANCE);

  // Implement color enhancement.
  corrected_bgr[0] = (bgr[0] - (wideband_veiling_light[0] * blue_backscatter_val)) / blue_direct_signal_val;
  corrected_bgr[1] = (bgr[1] - (wideband_veiling_light[1] * green_backscatter_val)) / green_direct_signal_val;
  corrected_bgr[2] = (bgr[2] - (wideband_veiling_light[2] * red_backscatter_val)) / red_direct_signal_val;

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: New method enhancment complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: New method enhancment complete" << std::endl;
  }

  // Merge the BGR channels into normal image format.
  cv::Mat corrected_img;
  merge(corrected_bgr, corrected_img);

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Merge image complete. Time: " << double(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Merge image complete." << std::endl;
  }

  if (this->SAVE_DATA)
  {
    // Add declaration to the top of the XML file
    if (!this->file_initialized)
    {
      initialize_file();
    }
    set_data_to_file();
  }

  return corrected_img;
}


/** SLAM implementation that utilizes feature points
 */
cv::Mat NewModel::color_correct_slam(cv::Mat& img, std::vector<cv::Point2f> point_data,
  std::vector<float> distance_data)
{
  if (this->CHECK_TIME)
  {
    this->begin = clock();
  }

  // Setting up for Voronoi Diagram algorithm
  cv::Rect rect(0, 0, img.cols, img.rows);
  cv::Subdiv2D subdiv(rect);

  for (int i = 0; i < distance_data.size(); i++)
  {
    subdiv.insert(point_data.at(i));
  }

  // Could initialize all of these ahead of time
  cv::Mat img_voronoi = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);

  std::vector<std::vector<cv::Point2f> > facets;
  std::vector<cv::Point2f> centers;
  subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

  std::vector<cv::Point> ifacet;
  for (size_t i = 0; i < facets.size(); i++)
  {
    ifacet.resize(facets[i].size());
    for (size_t j = 0; j < facets[i].size(); j++)
    {
      ifacet[j] = facets[i][j];
    }
    fillConvexPoly(img_voronoi, ifacet, cv::Scalar(distance_data[i]), 0, 0);
  }

  // Split BGR image to a Mat array of each color channel
  cv::Mat bgr[3];
  split(img, bgr);

  std::vector<cv::Mat> corrected_bgr(3);

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Set image for processing complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Set image for processing complete" << std::endl;
  }

  // Calculate or estimate wideband veiling light
  cv::Scalar wideband_veiling_light;
  // FUTURE: average wideband veiling light be calculates using image processing techniques
  if (this->EST_VEILING_LIGHT)  // Estimate wideband veiling light as average background value
  {
    cv::Rect region_of_interest(this->scene->BACKGROUND_SAMPLE[0], this->scene->BACKGROUND_SAMPLE[1],
      this->scene->BACKGROUND_SAMPLE[2], this->scene->BACKGROUND_SAMPLE[3]);
    cv::Mat background = img(region_of_interest);

    // TO DO: this mean is done independently for each channel
    // Should I take the average pixel color instead?
    wideband_veiling_light = mean(background);
    // cv::Scalar wideband_veiling_light = cv::Scalar(255, 255, 255);
  }
  else  // FUTURE: implement calculation for wideband veiling light
  {
    wideband_veiling_light = calc_wideband_veiling_light();
  }

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Veiling light calculation complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Veiling light calculation complete" << std::endl;
  }

  if (this->PRIOR_DATA)  // Use prior data to retrieve backscatter and direct signal attenauation values
  {
    est_attenuation();
  }
  else  // Must calculate the attenuation values using a color chart
  {
    // TO DO: Could have these rectangles initialized ahead of time
    // Create rectangles of the regions of interest
    cv::Rect patch_1_region(this->scene->COLOR_1_SAMPLE[0], this->scene->COLOR_1_SAMPLE[1],
      this->scene->COLOR_1_SAMPLE[2], this->scene->COLOR_1_SAMPLE[3]);
    cv::Rect patch_2_region(this->scene->COLOR_2_SAMPLE[0], this->scene->COLOR_2_SAMPLE[1],
       this->scene->COLOR_2_SAMPLE[2], this->scene->COLOR_2_SAMPLE[3]);
    // Splice regions from the image
    cv::Mat color_1_region = img(patch_1_region);
    cv::Mat color_2_region = img(patch_2_region);

    // TO DO: this mean is done independently for each channel. Should I take the average pixel color instead?
    // mean pixel value of observed colors
    cv::Scalar color_1_obs = mean(color_1_region);
    cv::Scalar color_2_obs = mean(color_2_region);

    calc_attenuation(color_1_obs, color_2_obs, wideband_veiling_light);
  }

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Attenuation calculation complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Attenuation calculation complete" << std::endl;
  }

  // Calculate backscatter and direct signal values
  cv::Mat blue_backscatter_val = -1.0 * this->backscatter_att[0] * img_voronoi;
  cv::exp(blue_backscatter_val, blue_backscatter_val);
  blue_backscatter_val = 1.0 - blue_backscatter_val;
  cv::Mat green_backscatter_val = -1.0 * this->backscatter_att[1] * img_voronoi;
  cv::exp(green_backscatter_val, green_backscatter_val);
  green_backscatter_val = 1.0 - green_backscatter_val;
  cv::Mat red_backscatter_val = -1.0 * this->backscatter_att[2] * img_voronoi;
  cv::exp(red_backscatter_val, red_backscatter_val);
  red_backscatter_val = 1.0 - red_backscatter_val;

  cv::Mat blue_direct_signal_val = -1.0 * this->direct_signal_att[0] * img_voronoi;
  cv::exp(blue_direct_signal_val, blue_direct_signal_val);
  cv::Mat green_direct_signal_val = -1.0 * this->direct_signal_att[1] * img_voronoi;
  cv::exp(green_direct_signal_val, green_direct_signal_val);
  cv::Mat red_direct_signal_val = -1.0 * this->direct_signal_att[2] * img_voronoi;
  cv::exp(red_direct_signal_val, red_direct_signal_val);

  // Implement color enhancement.
  corrected_bgr[0] = (bgr[0] - (wideband_veiling_light[0] * blue_backscatter_val)) / blue_direct_signal_val;
  corrected_bgr[1] = (bgr[1] - (wideband_veiling_light[1] * green_backscatter_val)) / green_direct_signal_val;
  corrected_bgr[2] = (bgr[2] - (wideband_veiling_light[2] * red_backscatter_val)) / red_direct_signal_val;

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: New method enhancment complete. Time: " <<
      static_cast<double>(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;

    this->begin = clock();
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: New method enhancment complete" << std::endl;
  }

  // Merge the BGR channels into normal image format.
  cv::Mat corrected_img;
  merge(corrected_bgr, corrected_img);

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "LOG: Merge image complete. Time: " << double(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;
  }
  else if (this->LOG_SCREEN)
  {
    std::cout << "LOG: Merge image complete." << std::endl;
  }

  if (this->SAVE_DATA)
  {
    // Add declaration to the top of the XML file
    if (!this->file_initialized)
    {
      initialize_file();
    }
    set_data_to_file();
  }

  return corrected_img;
}


double model(const NewModel::opt_vector& input, const NewModel::opt_vector& params)
{
  const double backscatter_val = params(0);
  const double direct_signal_val = params(1);

  const double observed_color = input(0);
  const double wideband_veiling_light = input(1);

  const double correct_color = (observed_color - (wideband_veiling_light * backscatter_val)) / direct_signal_val;

  return correct_color;
}


double NewModel::residual(const std::pair<opt_vector, double>& data, const opt_vector& params)
{
  return model(data.first, params) - data.second;
}


/** Calculate background pixel using known characteristics of camera and underwater_scene
 */
cv::Scalar NewModel::calc_wideband_veiling_light()
{
  float temp_cur = this->scene->b_sca[0] * this->scene->irradiance[0] / this->scene->b_att[0];
  cv::Scalar wideband_veiling_light = this->scene->camera_response[0] * temp_cur;

  for (int i = 1; i < this->scene->WAVELENGTHS.size() - 1; i++)
  {
    temp_cur = this->scene->b_sca[i] * this->scene->irradiance[i] / this->scene->b_att[i];
    wideband_veiling_light +=  2.0 * this->scene->camera_response[i] * temp_cur;
  }

  temp_cur = this->scene->b_sca.back() * this->scene->irradiance.back() / this->scene->b_att.back();
  wideband_veiling_light +=  this->scene->camera_response.back() * temp_cur;

  wideband_veiling_light *= 1.0 / this->scene->K * this->scene->WAVELENGTHS_SUB;

  return wideband_veiling_light;
}


void NewModel::calc_attenuation(cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light)
{
  // Calculate backscatter attenuation for each channel
  float channel_bs;
  for (int i = 0; i < 3; i++)
  {
    channel_bs =  (this->COLOR_1_TRUTH[i] * color_2_obs[i]) - (this->COLOR_2_TRUTH[i] * color_1_obs[i]) +
      (this->COLOR_2_TRUTH[i] - this->COLOR_1_TRUTH[i]) * wideband_veiling_light[i];
    channel_bs = channel_bs / ((this->COLOR_2_TRUTH[i] - this->COLOR_1_TRUTH[i]) * wideband_veiling_light[i]);
    this->backscatter_att[i] = -1.0 * log(channel_bs) / this->scene->DISTANCE;
  }

  // Calculate direct signal attenuation for each channel
  float channel_ds;
  for (int i = 0; i < 3; i++)
  {
    channel_ds =  color_2_obs[i] - wideband_veiling_light[i] *
      (1.0 - exp(-1.0 * this->backscatter_att[i] * this->scene->DISTANCE));
    channel_ds = channel_ds / this->COLOR_2_TRUTH[i];
    this->direct_signal_att[i] = -1.0 * log(channel_ds) / this->scene->DISTANCE;
  }
}


/** Set attenuation values from pre calculated attenuation values.
 */
void NewModel::est_attenuation()
{
  this->backscatter_att[0] = this->att_map[this->depth][0];
  this->backscatter_att[1] = this->att_map[this->depth][1];
  this->backscatter_att[2] = this->att_map[this->depth][2];

  this->direct_signal_att[0] = this->att_map[this->depth][3];
  this->direct_signal_att[1] = this->att_map[this->depth][4];
  this->direct_signal_att[2] = this->att_map[this->depth][5];
}


void NewModel::initialize_file()
{
  TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
  this->out_doc.LinkEndChild(decl);

  this->file_initialized = true;
}


void NewModel::set_data_to_file()
{
  TiXmlElement * data_depth = new TiXmlElement("Depth");
  this->out_doc.LinkEndChild(data_depth);
  data_depth->SetDoubleAttribute("val", static_cast<double>(this->depth));

  TiXmlElement * data_backscatter_att = new TiXmlElement("Backscatter_Attenuation");
  data_depth->LinkEndChild(data_backscatter_att);
  data_backscatter_att->SetDoubleAttribute("blue", this->backscatter_att[0]);
  data_backscatter_att->SetDoubleAttribute("green", this->backscatter_att[1]);
  data_backscatter_att->SetDoubleAttribute("red", this->backscatter_att[2]);

  TiXmlElement * data_direct_signal_att = new TiXmlElement("Direct_Signal_Attenuation");
  data_depth->LinkEndChild(data_direct_signal_att);
  data_direct_signal_att->SetDoubleAttribute("blue", this->direct_signal_att[0]);
  data_direct_signal_att->SetDoubleAttribute("green", this->direct_signal_att[1]);
  data_direct_signal_att->SetDoubleAttribute("red", this->direct_signal_att[2]);
}


void NewModel::end_file(std::string OUTPUT_FILENAME)
{
  this->out_doc.SaveFile(OUTPUT_FILENAME);
}


void NewModel::load_data(std::string INPUT_FILENAME)
{
  TiXmlDocument in_doc(INPUT_FILENAME);
  if (!in_doc.LoadFile()) exit(EXIT_FAILURE);

  double m_depth;
  double bs_blue, bs_green, bs_red;
  double ds_blue, ds_green, ds_red;

  TiXmlHandle hDoc(&in_doc);
  TiXmlHandle hRoot(0);

  TiXmlElement* pDepthNode = hDoc.FirstChild("Depth").Element();
  for (pDepthNode; pDepthNode; pDepthNode = pDepthNode->NextSiblingElement())
  {
    pDepthNode->QueryDoubleAttribute("val", &m_depth);

    hRoot = TiXmlHandle(pDepthNode);
    TiXmlElement* pAttNode = hRoot.FirstChild("Backscatter_Attenuation").Element();
    if (pAttNode)
    {
      pAttNode->QueryDoubleAttribute("blue", &bs_blue);
      pAttNode->QueryDoubleAttribute("green", &bs_green);
      pAttNode->QueryDoubleAttribute("red", &bs_red);
    }

    pAttNode = hRoot.FirstChild("Direct_Signal_Attenuation").Element();
    if (pAttNode)
    {
      pAttNode->QueryDoubleAttribute("blue", &ds_blue);
      pAttNode->QueryDoubleAttribute("green", &ds_green);
      pAttNode->QueryDoubleAttribute("red", &ds_red);
    }

    this->att_map[m_depth] = {bs_blue, bs_green, bs_red, ds_blue, ds_green, ds_red};
  }
}

}  // namespace underwater_color_enhance
