//
// Created by sunxi on 2/25/20.
//

#include "base_odometry.h"
#include "core/util/util.h"

using namespace std;
using namespace myslam;

Pose::Pose() {
    p = V3f(0, 0, 0);
    q = Q4f(1, 0, 0 ,0);
}

Pose::Pose(V3f& _p, Q4f& _q) : p(_p), q(_q) {
}

BaseOdometry::BaseOdometry(const string& config_file_path) {
}

void BaseOdometry::imgCallback(const myslam::ImgData &img_data) {
    img_ = img_data.img;
    cvtColorColor2Gray(img_);
}

void BaseOdometry::imuCallback(const myslam::ImuData &imu_data) {
}

Pose BaseOdometry::getCameraPose() {
    return {};
}