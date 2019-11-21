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
#include <ORB_SLAM2/Points.h>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace underwater_color_enhance
{

/** Image handler class.
 *  Handles ROS messages (images, altitude depth measurements, and ORB-SLAM features),
 *  and calls appropriate color enhancement method.
 */

class ImageHandler
{
public:
  /** Constructor.
   *  Initializes the parameters and sets up the ROS subscribers and callbacks.
   *
   *  \param correction_method object includes the focused color enhancement method.
   *  \param SLAM_INPUT - true: utilize ORB-SLAM features.
   *  \param SAVE_DATA - see below.
   *  \param SHOW_IMAGE - see below.
   *  \param CHECK_TIME - see below.
   *  \param CAMERA_TOPIC is the name of the topic for camera images.
   *  \param DEPTH_TOPIC is the name of the topic for the altitude depth measurements.
   */
  ImageHandler(ColorCorrect correction_method, bool SLAM_INPUT, bool SAVE_DATA, bool SHOW_IMAGE, bool CHECK_TIME,
    std::string CAMERA_TOPIC, std::string DEPTH_TOPIC);
  ~ImageHandler() {}

private:
  ros::NodeHandle nh_;

  ros::Publisher img_pub_;          /**< publisher for current enhanced image */

  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<mavros_msgs::VFR_HUD> depth_sub_;
  message_filters::Subscriber<ORB_SLAM2::Points> orb_slam2_sub_;

  ColorCorrect correction_method;   /**< handles current color enhancement method */

  /** With no SLAM implementation (SLAM_INPUT == false).
    * Only handles camera images and altitude depth measurements.
    */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, mavros_msgs::VFR_HUD> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  /** With SLAM implementation (SLAM_INPUT == true).
   *  Only handles camera images, altitude depth measurements, and ORB-SLAM features.
   */
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, mavros_msgs::VFR_HUD,
    ORB_SLAM2::Points> SyncPolicySLAM;
  typedef message_filters::Synchronizer<SyncPolicySLAM> SyncSLAM;
  boost::shared_ptr<SyncSLAM> sync_slam;

  bool SAVE_DATA;     /**< true: save attenuation values to output file */
  bool SHOW_IMAGE;    /**< true: visualize images, both raw and corrected */

  std::clock_t begin;
  std::clock_t end;
  bool CHECK_TIME;    /**< true: track and publish time periods */

  /** Callback for image and depth measurements.
   *  Handles processing of messages and calls color enhancement method.
   *
   *  \param img_msg is the message from the camera/image topic.
   *  \param depth_msg is the message from the depth sensor topic.
   */
  void camera_depth_callback(const sensor_msgs::ImageConstPtr& img_msg,
    const mavros_msgs::VFR_HUD::ConstPtr& depth_msg);

  /** Callback for image, depth measurements, and ORB-SLAM features.
   *  Handles processing of messages and calls color enhancment method.
   *
   *  \param img_msg is the message from the camera/image topic.
   *  \param depth_msg is the message from the depth sensor topic.
   *  \param orb_slam2_msg is the message from the ORB-SLAM topic.
   */
  void camera_depth_slam_callback(const sensor_msgs::ImageConstPtr& img_msg,
    const mavros_msgs::VFR_HUD::ConstPtr& depth_msg,
    const ORB_SLAM2::Points::ConstPtr& orb_slam2_msg);
};

}  // namespace underwater_color_enhance

#endif  // UNDERWATER_COLOR_ENHANCE_IMAGEHANDLER_H
