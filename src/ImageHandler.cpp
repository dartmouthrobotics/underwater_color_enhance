/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "underwater_color_enhance/ImageHandler.h"

#include <cv_bridge/cv_bridge.h>
#include <string>

namespace underwater_color_enhance
{

ImageHandler::ImageHandler(underwater_color_enhance::ColorCorrect correction_method,
  bool save_data, bool show_image, bool check_time, std::string camera_topic, std::string depth_topic)
{
  this->correction_method = correction_method;
  this->save_data = save_data;
  this->show_image = show_image;
  this->check_time = check_time;

  this->img_pub_ = nh_.advertise<sensor_msgs::Image>("/image_enhancement/output_image", 1);

  this->img_sub_.subscribe(nh_, camera_topic, 1);
  this->depth_sub_.subscribe(nh_, depth_topic, 1);

  this->sync.reset(new Sync(SyncPolicy(1), this->img_sub_, this->depth_sub_));
  this->sync->registerCallback(boost::bind(&ImageHandler::camera_depth_callback, this, _1, _2));

  std::cout << "LOG: Image handler initialization complete" << std::endl;
}


void ImageHandler::camera_depth_callback(const sensor_msgs::ImageConstPtr& img_msg,
  const mavros_msgs::VFR_HUD::ConstPtr& depth_msg)
{
  // CLOCK
  std::clock_t begin;
  std::clock_t end;
  if (this->check_time)
  {
    begin = clock();
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    int i = 0;
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  this->correction_method.set_depth(abs(depth_msg->altitude));

  cv::Mat corrected_frame = this->correction_method.enhance(cv_ptr->image);

  // CLOCK
  if (this->check_time)
  {
    end = clock();
    std::cout << "Enhancement complete. Total time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;
  }

  if (this->save_data)
  {
    this->correction_method.save_final_data();
  }

  if (this->show_image)
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
