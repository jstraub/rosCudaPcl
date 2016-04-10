/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace enc = sensor_msgs::image_encodings;
using std::cout;
using std::endl;

class RosGrabber
{
public:

  RosGrabber(ros::NodeHandle& nh): 
    nh_(nh), it_(nh_), spinner(4)
  {};

  virtual ~RosGrabber()
  {};
   
  virtual void depth_cb(const uint16_t * depth, uint32_t w, uint32_t h)
  {
      cout<<"depth"<<endl;
  };  

  virtual void rgb_cb(const uint8_t* rgb, uint32_t w, uint32_t h)
  {
    cout<<"rgb"<<endl;
  };  

  virtual void run ()
  {
    this->run_impl();
    while (42) boost::this_thread::sleep (boost::posix_time::seconds (1));
    this->run_cleanup_impl();
  }
//void RealtimeMF::run ()
//{
//  boost::thread visualizationThread(&RealtimeMF::visualizePc,this); 
//
//  this->run_impl();
//  while (42) boost::this_thread::sleep (boost::posix_time::seconds (1));
//  this->run_cleanup_impl();
//  visualizationThread.join();
//}

protected:
  virtual void rgb_cb_(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {// share the image means we are directly accessing
      // the message - no copying happening
      rgbPtr = cv_bridge::toCvShare(msg, enc::BGR8); 
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    rgb_ = rgbPtr->image.clone();
    rgb_cb(rgb_.data,rgb_.cols,rgb_.rows);
  }

  virtual void d_cb_(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr dPtr;
    try{
      dPtr = cv_bridge::toCvShare(msg, enc::TYPE_16UC1);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    uint32_t h = dPtr->image.rows;
    uint32_t w = dPtr->image.cols;
    // rotation between MF and depth image
    depth_cb((const uint16_t*)dPtr->image.data, w,h);
  }

  void run_impl ()
  {
    //image_pub_ = it_.advertise("out", 1);
    rgb_sub_ = it_.subscribe("/camera/rgb/image_raw",
        1, &RosGrabber::rgb_cb_, this);
    d_sub_ = it_.subscribe("/camera/depth/image_raw", 
        1, &RosGrabber::d_cb_, this);
    // start receiving point clouds
    spinner.start();
  }
  void run_cleanup_impl()
  {
    // stop the grabber
    spinner.stop();
  }
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::AsyncSpinner spinner;

  image_transport::Subscriber rgb_sub_;
  image_transport::Subscriber d_sub_;
  cv_bridge::CvImageConstPtr rgbPtr;

  cv::Mat rgb_;
private:
};

