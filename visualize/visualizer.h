//
// Created by sunxi on 2/28/20.
//

#ifndef MYSLAM_VISUALIZER_H
#define MYSLAM_VISUALIZER_H

#ifdef HAVE_VIZ

#include "core/common.h"

using namespace cv;

namespace myslam {

class Visualizer {
public:
    Visualizer();

    void addCamera(const M3d& K, const Point3d& init_camera_pos);
    void addCamera(const M3d& K, const V3d& p_r);
    void updateCameraPose(const M3d& r_R_c, const V3d& r_t_c);
    void hold();

private:
    Point3d getPointInWorld(const V3d& p_r);

    viz::Viz3d myWindow_;
    viz::WCameraPosition cpw_; // Coordinate axes
    viz::WCameraPosition cpw_frustum_; // Camera frustum
    bool have_camera_;
    Point3d last_camera_pos_;
    size_t line_id = 0;
};

}

#endif

#endif //MYSLAM_VISUALIZER_H
