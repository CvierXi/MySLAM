//
// Created by sunxi on 2/24/20.
//

#ifndef MYSLAM_UTIL_H
#define MYSLAM_UTIL_H

#include "core/common.h"

namespace myslam {

double imgName2Timestamp(std::string img_name);

void cvtColorGray2Color(cv::Mat& img);
void cvtColorColor2Gray(cv::Mat& img);

}

#endif //MYSLAM_UTIL_H
