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
    explicit Camera(M3d& K);
    void setK(M3d& K);
    M3d K();
    float fx();
    float fy();
    float cx();
    float cy();

private:
    M3d K_;
};

}

#endif //MYSLAM_CAMERA_H
