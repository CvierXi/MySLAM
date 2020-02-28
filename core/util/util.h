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

void recoverPoseFromHomography(const M3d& H, M3d& R, V3d& t);

}

#endif //MYSLAM_UTIL_H
