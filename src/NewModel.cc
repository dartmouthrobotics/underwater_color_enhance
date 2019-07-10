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

  if(this->est_veiling_light)
  {
    cv::Rect region_of_interest(this->scene.background_sample[0], this->scene.background_sample[1], this->scene.background_sample[2], this->scene.background_sample[3]);
    cv::Mat background = img(region_of_interest);
    // NOTE: this mean is done independently for each channel
    // Should I take the average pixel color instead?
    wideband_veiling_light = mean(background);
  }
  else
  {
    // TO DO: implement calculation for wideband veiling light
    wideband_veiling_light = calc_wideband_veiling_light();
  }

  end = clock();

  std::cout << "Calc veiling light time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  begin = clock();

  // Must calculate the attenuation values using a color chart
  if( false == this->prior )
  {
    // To Do: Need to sample more from the same patch!

    // retrieve observed white and black colors from their patches
    // float white_avg_obs = img.at<double>(307, 259);

    // current sampled points from the shipwreck_depth_000606.png
    int white_obs [3] = {194, 190, 65};
    int black_obs [3] = {17, 30, 38};

    calc_attenuation(white_obs, black_obs, wideband_veiling_light);
  }
  else // Must estimate attenuation values using prior data
  {
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



// cv::Mat NewModel::color_correct2( cv::Mat& img )
// {
//   std::clock_t begin = clock();
//   // split BGR image to a Mat array of each color channel
//   cv::Mat bgr[3];
//   split(img, bgr);
//
//   Eigen::MatrixXd blue;
//   Eigen::MatrixXd green;
//   Eigen::MatrixXd red;
//
//   // convert each image channel to an eigen matrix
//   cv::cv2eigen(bgr[0], blue);
//   cv::cv2eigen(bgr[1], green);
//   cv::cv2eigen(bgr[2], red);
//
//   std::clock_t end = clock();
//
//   std::cout << "Initialize parameters time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
//
//   begin = clock();
//
//   // Calculate or estimate wideband veiling light
//   int *wideband_veiling_light = new int[3];
//   calc_wideband_veiling_light(&wideband_veiling_light);
//
//   end = clock();
//
//   std::cout << "Calc veiling light time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
//
//   begin = clock();
//
//   // Must calculate the attenuation values using a color chart
//   if( false == this->prior )
//   {
//     // To Do: Need to sample more from the same patch!
//
//     // retrieve observed white and black colors from their patches
//     // float white_avg_obs = img.at<double>(307, 259);
//
//     // current sampled points from the shipwreck_depth_000606.png
//     int white_obs [3] = {194, 190, 65};
//     int black_obs [3] = {17, 30, 38};
//
//     calc_attenuation(white_obs, black_obs, wideband_veiling_light);
//   }
//   else // Must estimate attenuation values using prior data
//   {
//     est_attenuation();
//   }
//
//   end = clock();
//
//   std::cout << "Calc attenuation time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
//
//   begin = clock();
//
//   // auto n = blue.size();
//   int r = blue.rows();
//   int c = blue.cols();
//
//   // expand wideband veiling light for each color channel to be an eigen matrix
//   Eigen::MatrixXd blue_wv_light = Eigen::MatrixXd::Constant(r, c, wideband_veiling_light[0]);
//   Eigen::MatrixXd green_wv_light = Eigen::MatrixXd::Constant(r, c, wideband_veiling_light[1]);
//   Eigen::MatrixXd red_wv_light = Eigen::MatrixXd::Constant(r, c, wideband_veiling_light[2]);
//
//   float blue_backscatter_val = 1.0 - exp( -1.0 * this->backscatter_att[0] * this->scene.distance );
//   float green_backscatter_val = 1.0 - exp( -1.0 * this->backscatter_att[1] * this->scene.distance );
//   float red_backscatter_val = 1.0 - exp( -1.0 * this->backscatter_att[2] * this->scene.distance );
//
//   float blue_direct_signal_val = exp( -1.0 * this->direct_signal_att[0] * this->scene.distance );
//   float green_direct_signal_val = exp( -1.0 * this->direct_signal_att[1] * this->scene.distance );
//   float red_direct_signal_val = exp( -1.0 * this->direct_signal_att[2] * this->scene.distance );
//
//   Eigen::MatrixXd blue_enhanced = ( blue - ( blue_wv_light * blue_backscatter_val ) ) / blue_direct_signal_val / 255.0;
//   Eigen::MatrixXd green_enhanced = ( green - ( green_wv_light * green_backscatter_val ) ) / green_direct_signal_val / 255.0;
//   Eigen::MatrixXd red_enhanced = ( red - ( red_wv_light * red_backscatter_val ) ) / red_direct_signal_val / 255.0;
//
//   // need to convert all elements in matrix to be a value between 0 and 1
//   blue = blue / 255.0;
//   green = green / 255.0;
//   red = red / 255.0;
//
//   end = clock();
//
//   std::cout << "Color correct time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
//
//   begin = clock();
//
//   std::vector<cv::Mat> corrected_bgr(3);
//   cv::eigen2cv(blue_enhanced, corrected_bgr[0]);
//   cv::eigen2cv(green_enhanced, corrected_bgr[1]);
//   cv::eigen2cv(red_enhanced, corrected_bgr[2]);
//
//   cv::Mat corrected_img;
//   merge(corrected_bgr, corrected_img);
//
//   end = clock();
//
//   std::cout << "Merge image time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
//
//   return corrected_img;
// }
//

// TO DO!!!!!
// Move these to another file? Library file?

cv::Scalar NewModel::calc_wideband_veiling_light()
{
  // Calculate background pixel using known characteristics of camera and underwater_scene
  cv::Scalar wideband_veiling_light = {169, 153, 130, 0};

  return wideband_veiling_light;
}


void NewModel::calc_attenuation( int white_obs[], int black_obs[], cv::Scalar wideband_veiling_light )
{
  // Calculate backscatter attenuation for each channel
  float channel_bs;
  for(int i = 0; i < 3; i++)
  {
    channel_bs =  ( this->white_truth[i] * black_obs[i] ) - ( this->black_truth[i] * white_obs[i] ) + ( this->black_truth[i] - this->white_truth[i] ) * wideband_veiling_light[i];
    channel_bs = channel_bs / ( ( this->black_truth[i] - this->white_truth[i] ) * wideband_veiling_light[i] );
    this->backscatter_att[i] = -1.0 * log( channel_bs ) / this->scene.distance;
  }

  // Calculate direct signal attenuation for each channel
  float channel_ds;
  for(int i = 0; i < 3; i++)
  {
    channel_ds =  black_obs[i] - wideband_veiling_light[i] * ( 1.0 - exp( -1.0 * this->backscatter_att[i] * this->scene.distance ) );
    channel_ds = channel_ds / this->black_truth[i];
    this->direct_signal_att[i] = -1.0 * log( channel_ds ) / this->scene.distance;
  }
}


void NewModel::est_attenuation()
{
  // To Do: implement mapping of depth to attenuation values
  this->backscatter_att[0] = 5.0;
  this->backscatter_att[1] = 5.0;
  this->backscatter_att[2] = 5.0;

  this->direct_signal_att[0] = 6.0;
  this->direct_signal_att[1] = 6.0;
  this->direct_signal_att[2] = 6.0;
}
