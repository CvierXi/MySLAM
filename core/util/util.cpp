//
// Created by sunxi on 2/24/20.
//

#include "util.h"

using namespace std;

namespace myslam {

double imgName2Timestamp(std::string img_name) {
    img_name.resize(img_name.find('.'));
    double timestamp = atof(img_name.c_str()) / 1e9;
    return timestamp;
}

void cvtColorGray2Color(cv::Mat& img) {
    if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }
}

void cvtColorColor2Gray(cv::Mat& img) {
    if (img.channels() > 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
}

void recoverPoseFromHomography(const M3d& H, M3d& R, V3d& t) {
    V3d r1 = H.col(0);
    const float scale1 = r1.norm();
    r1 = r1 / scale1;

    V3d r2 = H.col(1);
    r2 -= r2.dot(r1) * r1;
    const float scale2 = r2.norm();
    r2 = r2 / scale2;

    V3d r3 = r1.cross(r2);

    M3d rotation;
    rotation << r1, r2, r3;

    Eigen::JacobiSVD<M3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R.noalias() = svd.matrixU() * svd.matrixV().transpose();
    t = H.col(2) / (scale1 + scale2) * 2.f;
}

}

