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

typedef Eigen::Vector3f V3f;
typedef Eigen::Matrix3f M3f;
typedef Eigen::Quaternionf Q4f;

}

#endif //MYSLAM_COMMON_H
