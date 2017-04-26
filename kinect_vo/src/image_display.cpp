 /* \copyright This work was completed by Robert Leishman while performing official duties as 
  * a federal government employee with the Air Force Research Laboratory and is therefore in the 
  * public domain (see 17 USC § 105). Public domain software can be used by anyone for any purpose,
  * and cannot be released under a copyright license
  */
#include "image_display.h"


ImageDisplay::ImageDisplay(std::string window_name):name(window_name)
{
  cv::namedWindow(name);



}

void ImageDisplay::displayImage(cv::Mat image)
{
  cv::imshow(name, image);
  cv::waitKey(1);
}


ImageDisplay::~ImageDisplay()
{
  cv::destroyAllWindows();
}
