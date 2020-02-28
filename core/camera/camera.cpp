//
// Created by sunxi on 2/27/20.
//

#include "camera.h"

using namespace std;
using namespace myslam;

Camera::Camera(float fx, float fy, float cx, float cy) {
    K_ << fx, 0, cx,
          0, fy, cy,
          0,  0,  1;
}

Camera::Camera(M3d& K) : K_(K) {
}

void Camera::setK(M3d &K) {
    K_ = K;
}

M3d Camera::K() {
    return K_;
}

float Camera::fx() {
    return K_(0, 0);
}

float Camera::fy() {
    return K_(1, 1);
}

float Camera::cx() {
    return K_(0, 2);
}

float Camera::cy() {
    return K_(1, 2);
}
