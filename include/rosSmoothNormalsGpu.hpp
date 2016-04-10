/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <rosVisualizer.hpp>
#include <cudaPcl/normalExtractSimpleGpu.hpp>
#include <cudaPcl/depthGuidedFilter.hpp>
#include <jsCore/timerLog.hpp>

using std::cout;
using std::endl;

class RosSmoothNormalsGpu : public RosVisualizer
{
public:
  RosSmoothNormalsGpu(ros::NodeHandle& nh, double f_d, double eps,
      uint32_t B, bool visualize=true)
    : RosVisualizer(nh,visualize), f_d_(f_d), eps_(eps),B_(B),
    depthFilter(NULL), normalExtract(NULL),
    tLog_("/scratch/evalRtmf/timerSurfNormals.log",2,10,"TimerLog")
  {
    normals_pub_ = this->it_.advertise("/surfNormals/normals_raw", 1);
    cout<<" advertising  /surfNormals/normals_raw "<<endl;
  };

  virtual ~RosSmoothNormalsGpu() 
  {
    if(depthFilter) delete depthFilter;
    if(normalExtract) delete normalExtract;
  };

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
    const uint16_t * depth = (const uint16_t*)dPtr->image.data;
    if(!this->depthFilter)
    {
      this->depthFilter = new cudaPcl::DepthGuidedFilterGpu<float>(w,h,
          eps_,B_);
      normalExtract = new cudaPcl::NormalExtractSimpleGpu<float>(f_d_,
          w,h, true);
    }
    cv::Mat dMap = cv::Mat(h,w,CV_16U,const_cast<uint16_t*>(depth));

    tLog_.tic(-1);
    this->depthFilter->filter(dMap);
    tLog_.toctic(0,1);
    normalExtract->computeGpu(this->depthFilter->getDepthDevicePtr());
    tLog_.toc(1);
    tLog_.logCycle();
    normals_cb(normalExtract->d_normalsImg(), normalExtract->d_haveData(),
        w,h);

//    cout<<"depth_cb size: "<<w<<" "<<h<<endl;
    normalsImg_ = normalExtract->normalsImg();

    cv_bridge::CvImage msgOut;
    msgOut.image = normalsImg_.clone();
    msgOut.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
    msgOut.header.seq = msg->header.seq;
    msgOut.header.stamp = msg->header.stamp;
    normals_pub_.publish(msgOut.toImageMsg());
  };  

  virtual void normals_cb(float* d_normalsImg, uint8_t* d_haveData, uint32_t w, uint32_t h)
  {
//    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr nDispPtr = normalExtract->normalsPc();
    boost::mutex::scoped_lock updateLock(updateModelMutex);
//    nDisp_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>(*nDispPtr));
    // update viewer
    this->update_ = true;
  };

protected:

  double f_d_;
  double eps_;
  uint32_t B_;
  cudaPcl::DepthGuidedFilterGpu<float> * depthFilter;
  cudaPcl::NormalExtractSimpleGpu<float> * normalExtract;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nDisp_;
  cv::Mat normalsImg_;

  image_transport::Publisher normals_pub_;
  jsc::TimerLog tLog_; // timer

  virtual void visualizeD();
  virtual void visualizePc();

private:
};

// ------------------------------------ impl ---------------------------------
void RosSmoothNormalsGpu::visualizeD()
{
  if (this->depthFilter && this->vis_)
  {
    cv::Mat dSmooth = this->depthFilter->getOutput();
    this->dColor_ = colorizeDepth(dSmooth,0.3,4.0);
    cv::imshow("d",dColor_);

    cv::Mat nI (normalsImg_.rows,normalsImg_.cols, CV_8UC3);
    cv::Mat nIRGB(normalsImg_.rows,normalsImg_.cols,CV_8UC3);
    normalsImg_.convertTo(nI, CV_8UC3, 127.5,127.5);
    cv::cvtColor(nI,nIRGB,CV_RGB2BGR);
    cv::imshow("normals",nIRGB);             
  }
};

void RosSmoothNormalsGpu::visualizePc()
{
  if (! this->vis_) return;
//  cv::Mat nI(nDisp->height,nDisp->width,CV_32FC3); 
//  for(uint32_t i=0; i<nDisp->width; ++i)
//    for(uint32_t j=0; j<nDisp->height; ++j)
//    {
//      // nI is BGR but I want R=x G=y and B=z
//      nI.at<cv::Vec3f>(j,i)[0] = (1.0f+nDisp->points[i+j*nDisp->width].z)*0.5f; // to match pc
//      nI.at<cv::Vec3f>(j,i)[1] = (1.0f+nDisp->points[i+j*nDisp->width].y)*0.5f; 
//      nI.at<cv::Vec3f>(j,i)[2] = (1.0f+nDisp->points[i+j*nDisp->width].x)*0.5f; 
//      nDisp->points[i+j*nDisp->width].rgb=0;
//    }
  cv::Mat nI (normalsImg_.rows,normalsImg_.cols, CV_8UC3);
  cv::Mat nIRGB(normalsImg_.rows,normalsImg_.cols,CV_8UC3);                              
  normalsImg_.convertTo(nI, CV_8UC3, 127.5,127.5);
  cv::cvtColor(nI,nIRGB,CV_RGB2BGR);
  cv::imshow("normals",nIRGB);             

#ifdef USE_PCL_VIEWER
  //copy again
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nDisp(
      new pcl::PointCloud<pcl::PointXYZRGB>(*nDisp_));
  this->pc_ = nDisp;
//  this->pc_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(nDisp);
  
  if(!this->viewer_->updatePointCloud(pc_, "pc"))
    this->viewer_->addPointCloud(pc_, "pc");
#endif
}
