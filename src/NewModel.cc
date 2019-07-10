#include "NewModel.h"

#include <math.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


cv::Mat NewModel::color_correct( cv::Mat& img )
{
  std::clock_t begin = clock();
  // split BGR image to a Mat array of each color channel
  cv::Mat bgr[3];
  split(img, bgr);

  std::vector<cv::Mat> corrected_bgr(3);

  std::clock_t end = clock();

  std::cout << "Initialize parameters time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  begin = clock();

  // Calculate or estimate wideband veiling light
  cv::Scalar wideband_veiling_light;

  // FUTURE: average wideband veiling light be calculates using image processing techniques
  if(this->est_veiling_light) // Estimate wideband veiling light as average background value
  {
    cv::Rect region_of_interest(this->scene.background_sample[0], this->scene.background_sample[1], this->scene.background_sample[2], this->scene.background_sample[3]);
    cv::Mat background = img(region_of_interest);
    // NOTE: this mean is done independently for each channel
    // Should I take the average pixel color instead?
    wideband_veiling_light = mean(background);
  }
  else  // FUTURE: implement calculation for wideband veiling light
  {
    wideband_veiling_light = calc_wideband_veiling_light();
  }

  end = clock();

  std::cout << "Calc veiling light time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  begin = clock();

  // Must calculate the attenuation values using a color chart
  if( false == this->prior )
  {
    // create rectangles of the regions of interest
    cv::Rect patch_1_region(this->scene.color_1_sample[0], this->scene.color_1_sample[1], this->scene.color_1_sample[2], this->scene.color_1_sample[3]);
    cv::Rect patch_2_region(this->scene.color_2_sample[0], this->scene.color_2_sample[1], this->scene.color_2_sample[2], this->scene.color_2_sample[3]);
    // splice regions from the image
    cv::Mat color_1_region = img(patch_1_region);
    cv::Mat color_2_region = img(patch_2_region);

    // NOTE: this mean is done independently for each channel. Should I take the average pixel color instead?
    // mean pixel value of observed colors
    cv::Scalar color_1_obs = mean(color_1_region);
    cv::Scalar color_2_obs = mean(color_2_region);

    calc_attenuation(color_1_obs, color_2_obs, wideband_veiling_light);
  }
  else // Must estimate attenuation values using prior data
  {
    // TO DO: implement retrieval of pre calculated attenuation values
    est_attenuation();
  }

  end = clock();

  std::cout << "Calc attenuation time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  begin = clock();

  float blue_backscatter_val = 1.0 - exp( -1.0 * this->backscatter_att[0] * this->scene.distance );
  float green_backscatter_val = 1.0 - exp( -1.0 * this->backscatter_att[1] * this->scene.distance );
  float red_backscatter_val = 1.0 - exp( -1.0 * this->backscatter_att[2] * this->scene.distance );

  float blue_direct_signal_val = exp( -1.0 * this->direct_signal_att[0] * this->scene.distance );
  float green_direct_signal_val = exp( -1.0 * this->direct_signal_att[1] * this->scene.distance );
  float red_direct_signal_val = exp( -1.0 * this->direct_signal_att[2] * this->scene.distance );

  corrected_bgr[0] = ( bgr[0] - ( wideband_veiling_light[0] * blue_backscatter_val ) ) / blue_direct_signal_val;
  corrected_bgr[1] = ( bgr[1] - ( wideband_veiling_light[1] * green_backscatter_val ) ) / green_direct_signal_val;
  corrected_bgr[2] = ( bgr[2] - ( wideband_veiling_light[2] * red_backscatter_val ) ) / red_direct_signal_val;

  end = clock();

  std::cout << "Color correct time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  begin = clock();

  cv::Mat corrected_img;
  merge(corrected_bgr, corrected_img);

  end = clock();

  std::cout << "Merge image time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  return corrected_img;
}


// TO DO: Move these to another file? Library file?

// FUTURE: implement calculation for wideband veiling light
cv::Scalar NewModel::calc_wideband_veiling_light()
{
  // Calculate background pixel using known characteristics of camera and underwater_scene
  cv::Scalar wideband_veiling_light = {169, 153, 130, 0};

  return wideband_veiling_light;
}


void NewModel::calc_attenuation( cv::Scalar color_1_obs, cv::Scalar color_2_obs, cv::Scalar wideband_veiling_light )
{
  // Calculate backscatter attenuation for each channel
  float channel_bs;
  for(int i = 0; i < 3; i++)
  {
    channel_bs =  ( this->color_1_truth[i] * color_2_obs[i] ) - ( this->color_2_truth[i] * color_1_obs[i] ) + ( this->color_2_truth[i] - this->color_1_truth[i] ) * wideband_veiling_light[i];
    channel_bs = channel_bs / ( ( this->color_2_truth[i] - this->color_1_truth[i] ) * wideband_veiling_light[i] );
    this->backscatter_att[i] = -1.0 * log( channel_bs ) / this->scene.distance;
  }

  // Calculate direct signal attenuation for each channel
  float channel_ds;
  for(int i = 0; i < 3; i++)
  {
    channel_ds =  color_2_obs[i] - wideband_veiling_light[i] * ( 1.0 - exp( -1.0 * this->backscatter_att[i] * this->scene.distance ) );
    channel_ds = channel_ds / this->color_2_truth[i];
    this->direct_signal_att[i] = -1.0 * log( channel_ds ) / this->scene.distance;
  }
}


void NewModel::est_attenuation()
{
  // TO DO: implement mapping of depth to attenuation values
  this->backscatter_att[0] = 5.0;
  this->backscatter_att[1] = 5.0;
  this->backscatter_att[2] = 5.0;

  this->direct_signal_att[0] = 6.0;
  this->direct_signal_att[1] = 6.0;
  this->direct_signal_att[2] = 6.0;
}
