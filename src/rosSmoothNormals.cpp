/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>
#include <string>
#include <stdio.h>
#include <iostream>
#include <signal.h>                                                             

#include <rosSmoothNormalsGpu.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

int main(int argc, char** argv)
{
  findCudaDevice(argc,(const char**)argv);
  ros::init(argc, argv, "rosCudaPcl");

  ros::NodeHandle nh;
  string mode;
  bool visualize = true;
  double f_d = 540.;
  double eps = 0.2;
  int B = 10;
  nh.param<std::string>("mode", mode, "");
  nh.param<bool>("visualize", visualize, false);
  nh.param<double>("f_d", f_d, 540.);
  nh.param<double>("sqrtEps", eps, 0.2);
  nh.param<int>("B", B, 10);
  eps = eps*eps;

  cout<<"visualize = "<<visualize<<" mode = "<<mode<<endl;
  cout<<"f_d = "<<f_d<<endl;
  cout<<"eps = "<<eps<<endl;
  cout<<"B = "<<B<<endl;

  RosSmoothNormalsGpu rosNormals(nh,f_d,eps,B, visualize);
  rosNormals.run();
  cout<<cudaDeviceReset()<<endl;
  return 0;
}
