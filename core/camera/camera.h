//
// Created by sunxi on 2/27/20.
//

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "core/common.h"

namespace myslam {

class Camera {
public:
    explicit Camera(float fx, float fy, float cx, float cy);
    explicit Camera(M3f& K);
    void setK(M3f& K);
    M3f K();
    float fx();
    float fy();
    float cx();
    float cy();

private:
    M3f K_;
};

}

#endif //MYSLAM_CAMERA_H
