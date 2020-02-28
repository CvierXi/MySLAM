//
// Created by sunxi on 2/23/20.
//

#ifndef MYSLAM_COMMON_H
#define MYSLAM_COMMON_H

#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace myslam {

typedef Eigen::Vector3d V3d;
typedef Eigen::Matrix3d M3d;
typedef Eigen::Quaterniond Q4d;

}

#endif //MYSLAM_COMMON_H
