/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/NewModel.h"

#include <math.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace underwater_color_enhance
{

cv::Mat NewModel::color_correct(cv::Mat& img)
{
  // CLOCK
  std::clock_t begin;
  std::clock_t end;
  if (this->check_time)
  {
    begin = clock();
  }

  // split BGR image to a Mat array of each color channel
  cv::Mat bgr[3];
  split(img, bgr);

  std::vector<cv::Mat> corrected_bgr(3);

  // CLOCK
  if (this->check_time)
  {
    std::clock_t end = clock();
    std::cout << "LOG: Set image for processing complete. Time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

    begin = clock();
  }
  else
  {
    std::cout << "LOG: Set image for processing complete" << std::endl;
  }

  // Calculate or estimate wideband veiling light
  cv::Scalar wideband_veiling_light;
  // FUTURE: average wideband veiling light be calculates using image processing techniques
  if (this->est_veiling_light)  // Estimate wideband veiling light as average background value
  {
    cv::Rect region_of_interest(this->scene.background_sample[0], this->scene.background_sample[1],
      this->scene.background_sample[2], this->scene.background_sample[3]);
    cv::Mat background = img(region_of_interest);

    // TO DO: this mean is done independently for each channel
    // Should I take the average pixel color instead?
    wideband_veiling_light = mean(background);
  }
  else  // FUTURE: implement calculation for wideband veiling light
  {
    wideband_veiling_light = calc_wideband_veiling_light();
  }

  // CLOCK
  if (this->check_time)
  {
    end = clock();
    std::cout << "LOG: Veiling light calculation complete. Time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

    begin = clock();
  }
  else
  {
    std::cout << "LOG: Veiling light calculation complete" << std::endl;
  }

  if (this->prior_data)  // Use prior data to retrieve backscatter and direct signal attenauation values
  {
    est_attenuation();
  }
  else  // Must calculate the attenuation values using a color chart
  {
    // create rectangles of the regions of interest
    cv::Rect patch_1_region(this->scene.color_1_sample[0], this->scene.color_1_sample[1],
      this->scene.color_1_sample[2], this->scene.color_1_sample[3]);
    cv::Rect patch_2_region(this->scene.color_2_sample[0], this->scene.color_2_sample[1],
       this->scene.color_2_sample[2], this->scene.color_2_sample[3]);
    // splice regions from the image
    cv::Mat color_1_region = img(patch_1_region);
    cv::Mat color_2_region = img(patch_2_region);

    // TO DO: this mean is done independently for each channel. Should I take the average pixel color instead?
    // mean pixel value of observed colors
    cv::Scalar color_1_obs = mean(color_1_region);
    cv::Scalar color_2_obs = mean(color_2_region);

    calc_attenuation(color_1_obs, color_2_obs, wideband_veiling_light);
  }

  // CLOCK
  if (this->check_time)
  {
    end = clock();
    std::cout << "LOG: Attenuation calculation complete. Time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

    begin = clock();
  }
  else
  {
    std::cout << "LOG: Attenuation calculation complete" << std::endl;
  }

  float blue_backscatter_val = 1.0 - exp(-1.0 * this->backscatter_att[0] * this->scene.distance);
  float green_backscatter_val = 1.0 - exp(-1.0 * this->backscatter_att[1] * this->scene.distance);
  float red_backscatter_val = 1.0 - exp(-1.0 * this->backscatter_att[2] * this->scene.distance);

  float blue_direct_signal_val = exp(-1.0 * this->direct_signal_att[0] * this->scene.distance);
  float green_direct_signal_val = exp(-1.0 * this->direct_signal_att[1] * this->scene.distance);
  float red_direct_signal_val = exp(-1.0 * this->direct_signal_att[2] * this->scene.distance);

  corrected_bgr[0] = (bgr[0] - (wideband_veiling_light[0] * blue_backscatter_val)) / blue_direct_signal_val;
  corrected_bgr[1] = (bgr[1] - (wideband_veiling_light[1] * green_backscatter_val)) / green_direct_signal_val;
  corrected_bgr[2] = (bgr[2] - (wideband_veiling_light[2] * red_backscatter_val)) / red_direct_signal_val;

  // CLOCK
  if (this->check_time)
  {
    end = clock();
    std::cout << "LOG: New method enhancment complete. Time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

    begin = clock();
  }
  else
  {
    std::cout << "LOG: New method enhancment complete" << std::endl;
  }

  cv::Mat corrected_img;
  merge(corrected_bgr, corrected_img);

  // CLOCK
  if (this->check_time)
  {
    end = clock();
    std::cout << "LOG: Merge image complete. Time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
  }
  else
  {
    std::cout << "LOG: Merge image complete." << std::endl;
  }

  if (this->save_data)
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


// FUTURE: implement calculation for wideband veiling light
cv::Scalar NewModel::calc_wideband_veiling_light()
{
  // Calculate background pixel using known characteristics of camera and underwater_scene
  cv::Scalar wideband_veiling_light = {169, 153, 130, 0};

  return wideband_veiling_light;
}


void NewModel::calc_attenuation(cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light)
{
  // Calculate backscatter attenuation for each channel
  float channel_bs;
  for (int i = 0; i < 3; i++)
  {
    channel_bs =  (this->color_1_truth[i] * color_2_obs[i]) - (this->color_2_truth[i] * color_1_obs[i]) +
      (this->color_2_truth[i] - this->color_1_truth[i]) * wideband_veiling_light[i];
    channel_bs = channel_bs / ((this->color_2_truth[i] - this->color_1_truth[i]) * wideband_veiling_light[i]);
    this->backscatter_att[i] = -1.0 * log(channel_bs) / this->scene.distance;
  }

  // Calculate direct signal attenuation for each channel
  float channel_ds;
  for (int i = 0; i < 3; i++)
  {
    channel_ds =  color_2_obs[i] - wideband_veiling_light[i] *
      (1.0 - exp(-1.0 * this->backscatter_att[i] * this->scene.distance));
    channel_ds = channel_ds / this->color_2_truth[i];
    this->direct_signal_att[i] = -1.0 * log(channel_ds) / this->scene.distance;
  }
}


void NewModel::est_attenuation()
{
  this->backscatter_att[0] = this->att_map[this->scene.depth][0];
  this->backscatter_att[1] = this->att_map[this->scene.depth][1];
  this->backscatter_att[2] = this->att_map[this->scene.depth][2];

  this->direct_signal_att[0] = this->att_map[this->scene.depth][3];
  this->direct_signal_att[1] = this->att_map[this->scene.depth][4];
  this->direct_signal_att[2] = this->att_map[this->scene.depth][5];
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
  data_depth->SetDoubleAttribute("val", this->scene.depth);

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


void NewModel::end_file(std::string output_filename)
{
  this->out_doc.SaveFile(output_filename);
}


void NewModel::load_data(std::string input_filename)
{
  TiXmlDocument in_doc(input_filename);
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
