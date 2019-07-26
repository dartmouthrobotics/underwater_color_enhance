/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef UNDERWATER_COLOR_ENHANCE_IMAGEHANDLER_H
#define UNDERWATER_COLOR_ENHANCE_IMAGEHANDLER_H

#include "underwater_color_enhance/ColorCorrect.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <mavros_msgs/VFR_HUD.h>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace underwater_color_enhance
{

class ImageHandler
{
public:
  ImageHandler(ColorCorrect correction_method, bool save_data, bool show_image, bool check_time,
    std::string camera_topic, std::string depth_topic);
  ~ImageHandler() {}

private:
  ros::NodeHandle nh_;
  ColorCorrect correction_method;
  ros::Publisher img_pub_;

  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<mavros_msgs::VFR_HUD> depth_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, mavros_msgs::VFR_HUD> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  bool save_data;
  bool show_image;
  bool check_time;

  void camera_depth_callback(const sensor_msgs::ImageConstPtr& img_msg,
    const mavros_msgs::VFR_HUD::ConstPtr& depth_msg);
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_IMAGEHANDLER_H
