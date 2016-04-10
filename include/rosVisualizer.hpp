/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <rosGrabber.hpp>

#define USE_PCL_VIEWER
#include <pcl/common/common_headers.h>
#ifdef USE_PCL_VIEWER
  #pragma GCC system_header
  #include <pcl/visualization/cloud_viewer.h>
#endif

using std::cout;
using std::endl;

class RosVisualizer : public RosGrabber
{
public:
  RosVisualizer(ros::NodeHandle& nh, bool visualize=true)
    : RosGrabber(nh), update_(false), vis_(visualize), pc_(new
        pcl::PointCloud<pcl::PointXYZRGB>(1,1))
  {};

  virtual ~RosVisualizer() 
  {};

  virtual void depth_cb(const uint16_t * depth, uint32_t w, uint32_t h)
  {
    cv::Mat dMap = cv::Mat(h,w,CV_16U,const_cast<uint16_t*>(depth));
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    dColor_ = colorizeDepth(dMap,30.,4000.);
    update_=true;
  };  

  static cv::Mat colorizeDepth(const cv::Mat& dMap, float min, float max);
  static cv::Mat colorizeDepth(const cv::Mat& dMap);
  virtual void run();

protected:
  bool update_;
  bool vis_;
  boost::mutex updateModelMutex;
  cv::Mat dColor_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_;

  virtual void visualize_();
  virtual void visualizeRGB();
  virtual void visualizeD();
  virtual void visualizePc();

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
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    this->rgb_ = rgbPtr->image.clone();
    rgb_cb(this->rgb_.data,this->rgb_.cols,this->rgb_.rows);
    cout<<"vis: rgb "<<vis_<<endl;
  }

#ifdef USE_PCL_VIEWER
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
#endif
private:
  void visualizerThread();
};

// ------------------------------------ impl ---------------------------------

void RosVisualizer::visualize_()
{
  visualizeRGB();
  visualizeD();
//  visualizePc();
};

void RosVisualizer::visualizeRGB()
{
  if (this->rgb_.rows > 0 && this->rgb_.cols > 0)
    cv::imshow("rgb",this->rgb_);
};

void RosVisualizer::visualizeD()
{
  if (dColor_.rows > 0 && dColor_.cols > 0)
    cv::imshow("d",dColor_);
};

void RosVisualizer::visualizePc()
{
#ifdef USE_PCL_VIEWER
  if(!viewer_->updatePointCloud(pc_, "pc"))
    viewer_->addPointCloud(pc_, "pc");
#endif
}

void RosVisualizer::visualizerThread()
{
  if(!vis_) return;
#ifdef USE_PCL_VIEWER
  // prepare visualizer named "viewer"
  viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
      new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_->initCameraParameters ();
  viewer_->setBackgroundColor (255, 255, 255);
  viewer_->addCoordinateSystem (1.0);
  //  viewer_->setPosition(0,0);
//  viewer_->setSize(1000,1000);

  while (!viewer_->wasStopped ())
  {
    viewer_->spinOnce (10);
#else
  while (42)
  {
#endif
    cv::waitKey(10);
    // Get lock on the boolean update and check if cloud was updated
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    if (update_)
    {
      visualize_();
      update_ = false;
    }
  }
}

void RosVisualizer::run ()
{
  boost::thread visualizationThread(
      &RosVisualizer::visualizerThread,this); 
  this->run_impl();
  while (42) boost::this_thread::sleep (boost::posix_time::seconds (1));
  this->run_cleanup_impl();
  visualizationThread.join();
};

cv::Mat RosVisualizer::colorizeDepth(const cv::Mat& dMap, float min,
    float max)
{

  double Min,Max;
  cv::minMaxLoc(dMap,&Min,&Max);
//  cout<<"min/max "<<min<<" " <<max<<" actual min/max "<<Min<<" " <<Max<<endl;
  cv::Mat d8Bit = cv::Mat::zeros(dMap.rows,dMap.cols,CV_8UC1);
  cv::Mat dColor;
  dMap.convertTo(d8Bit,CV_8UC1, 255./(max-min));
  cv::applyColorMap(d8Bit,dColor,cv::COLORMAP_JET);
  return dColor;
}

cv::Mat RosVisualizer::colorizeDepth(const cv::Mat& dMap)
{
  double min,max;
  cv::minMaxLoc(dMap,&min,&max);
//  cout<<" computed actual min/max "<<min<<" " <<max<<endl;
  cv::Mat d8Bit = cv::Mat::zeros(dMap.rows,dMap.cols,CV_8UC1);
  cv::Mat dColor;
  dMap.convertTo(d8Bit,CV_8UC1, 255./(max-min));
  cv::applyColorMap(d8Bit,dColor,cv::COLORMAP_JET);
  return dColor;
}

