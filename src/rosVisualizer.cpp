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

#include <rosVisualizer.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace std;

int main(int argc, char** argv)
{
//  findCudaDevice(argc,(const char**)argv);

  // exit cleanly on CTL-C                                                      
//  struct sigaction new_action;                                                  
//  new_action.sa_sigaction = sig_action;                                         
//  sigemptyset(&new_action.sa_mask);                                             
//  new_action.sa_flags = 0;                                                      
//  sigaction(SIGINT, &new_action, NULL);                                         
//  sigaction(SIGTERM, &new_action, NULL);                                        
//  sigaction(SIGHUP, &new_action, NULL);    

  ros::init(argc, argv, "rosVisualizer");

  ros::NodeHandle nh;
  string mode;
  nh.param<std::string>("/rosVisualizer/mode", mode, "");

  RosVisualizer rosViz(nh);
  rosViz.run();
//  cout<<cudaDeviceReset()<<endl;
  return 0;
}
