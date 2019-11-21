/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/ImageHandler.h"

#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

namespace underwater_color_enhance
{

ImageHandler::ImageHandler(underwater_color_enhance::ColorCorrect correction_method,
  bool SLAM_INPUT, bool SAVE_DATA, bool SHOW_IMAGE, bool CHECK_TIME, std::string CAMERA_TOPIC, std::string DEPTH_TOPIC)
{
  this->correction_method = correction_method;
  this->SAVE_DATA = SAVE_DATA;
  this->SHOW_IMAGE = SHOW_IMAGE;
  this->CHECK_TIME = CHECK_TIME;

  this->img_pub_ = nh_.advertise<sensor_msgs::Image>("/image_enhancement/output_image", 1);

  this->img_sub_.subscribe(nh_, CAMERA_TOPIC, 1);
  this->depth_sub_.subscribe(nh_, DEPTH_TOPIC, 1);

  if (!SLAM_INPUT)  // No ORB-SLAM features utilized
  {
    this->sync.reset(new Sync(SyncPolicy(2), this->img_sub_, this->depth_sub_));
    this->sync->registerCallback(boost::bind(&ImageHandler::camera_depth_callback, this, _1, _2));
  }
  else  // ORB-SLAM features utilized
  {
    // TO DO: make this rostopic name string to be retrieved from yaml file
    this->orb_slam2_sub_.subscribe(nh_, "/orb_slam2/escalibr_data", 1);
    this->sync_slam.reset(new SyncSLAM(SyncPolicySLAM(20), this->img_sub_, this->depth_sub_, this->orb_slam2_sub_));
    this->sync_slam->registerCallback(boost::bind(&ImageHandler::camera_depth_slam_callback, this, _1, _2, _3));
  }
}


void ImageHandler::camera_depth_callback(const sensor_msgs::ImageConstPtr& img_msg,
  const mavros_msgs::VFR_HUD::ConstPtr& depth_msg)
{
  if (this->CHECK_TIME)
  {
    this->begin = clock();
  }

  // Convert ROS image to CV Mat image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Altitude depth measurement
  this->correction_method.set_depth(depth_msg->altitude);

  // Color enhance image
  cv::Mat corrected_frame = this->correction_method.enhance(cv_ptr->image);

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "Enhancement complete. Total time: " << double(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;
  }

  if (this->SAVE_DATA)
  {
    this->correction_method.save_final_data();
  }

  if (this->SHOW_IMAGE)
  {
    cv::imshow("Original", cv_ptr->image);
    cv::imshow("Corrected", corrected_frame);
    cv::waitKey(1);
  }

  if (ros::ok())
  {
    sensor_msgs::Image new_img;
    this->img_pub_.publish(new_img);
  }
}


void ImageHandler::camera_depth_slam_callback(const sensor_msgs::ImageConstPtr& img_msg,
  const mavros_msgs::VFR_HUD::ConstPtr& depth_msg,
  const ORB_SLAM2::Points::ConstPtr& orb_slam2_msg)
{
  if (this->CHECK_TIME)
  {
    this->begin = clock();
  }

  // Convert ROS image to CV Mat image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Altitude depth measurement
  this->correction_method.set_depth(depth_msg->altitude);

  // ORB-SLAM features
  std::vector<cv::Point2f> point_data;
  std::vector<float> distance_data;
  for (size_t i = 0; i < sizeof(orb_slam2_msg->points); i++)
  {
    point_data.push_back(cv::Point2f(orb_slam2_msg->points[i].x, orb_slam2_msg->points[i].y));
    distance_data.push_back(orb_slam2_msg->distances[i]);
  }

  // Color enhance image
  cv::Mat corrected_frame = this->correction_method.enhance_slam(cv_ptr->image, point_data, distance_data);

  if (this->CHECK_TIME)
  {
    this->end = clock();
    std::cout << "Enhancement complete. Total time: " << double(this->end - this->begin) / CLOCKS_PER_SEC << std::endl;
  }

  if (this->SAVE_DATA)
  {
    this->correction_method.save_final_data();
  }

  if (this->SHOW_IMAGE)
  {
    cv::imshow("Original", cv_ptr->image);
    cv::imshow("Corrected", corrected_frame);
    cv::waitKey(1);
  }

  if (ros::ok())
  {
    sensor_msgs::Image new_img;
    this->img_pub_.publish(new_img);
  }
}

}  // namespace underwater_color_enhance
