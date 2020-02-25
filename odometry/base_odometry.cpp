//
// Created by sunxi on 2/25/20.
//

#include "base_odometry.h"

using namespace std;
using namespace myslam;

BaseOdometry::BaseOdometry(const string& config_file_path) {
}

void BaseOdometry::imgCallback(const myslam::ImgData &img_data) {
    img_ = img_data.img;
    if (img_.channels() > 1) {
        cv::cvtColor(img_, img_, cv::COLOR_BGR2GRAY);
    }
}

void BaseOdometry::imuCallback(const myslam::ImuData &imu_data) {
}