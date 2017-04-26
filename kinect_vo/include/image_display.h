
 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */
/*!
 *  \file image_display.h
 *  \author Robert Leishman
 *  \date March 2012
 *
 *  \brief This file docuemnts a dummy class for testing - is simply shows images using HighGUI
*/

#ifndef IMAGEPROC_H
#define IMAGEPROC_H

#include <opencv2/core/core_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImageDisplay
{
  //just a dummy class to test by showing images

public:
  ImageDisplay(std::string window_name);
  ~ImageDisplay();

  void displayImage(cv::Mat image);

protected:
  std::string name;


};


#endif
