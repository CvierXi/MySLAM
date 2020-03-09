//
// Created by sunxi on 2/28/20.
//

#include "visualizer.h"

using namespace std;
using namespace myslam;

Visualizer::Visualizer() {
    /// Create a window
    myWindow_ = viz::Viz3d("World Frame");

    /// Add coordinate axes
    myWindow_.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    /// Start event loop.
    myWindow_.spinOnce();
}

void Visualizer::addCamera(const M3d& K) {
    cpw_ = viz::WCameraPosition(0.5);
    cv::Matx33d cam_K;
    cv::eigen2cv(K, cam_K);
    cpw_frustum_ = viz::WCameraPosition(cam_K);
    have_camera_ = true;
}

void Visualizer::updateCameraPose(const M3d& R_rc, const V3d& t_rc) {
    if (!have_camera_) {
        return;
    }

    V3d cam_z_unit_r = R_rc * V3d(0, 0, 1) + t_rc;
    V3d cam_y_unit_r = R_rc * V3d(0, 1, 0);

    cv::Point3d cam_pos = getPointInWorld(t_rc);
    cv::Vec3d cam_focal_point = getPointInWorld(cam_z_unit_r);
    cv::Vec3d cam_y_dir = getPointInWorld(cam_y_unit_r);

//    if (line_id > 0) {
//        viz::WLine line(last_camera_pos_, cam_pos, cv::viz::Color::green());
//        myWindow_.showWidget("LINE" + std::to_string(line_id), line);
//    }
    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    last_camera_pos_ = cam_pos;
    line_id++;

    myWindow_.showWidget("CPW", cpw_, cam_pose);
    myWindow_.showWidget("CPW_FRUSTUM", cpw_frustum_, cam_pose);
    myWindow_.spinOnce();
}

void Visualizer::hold() {
    myWindow_.spin();
}


Point3d Visualizer::getPointInWorld(const V3d& p_r) {
    return {p_r(0),-p_r(1),-p_r(2)};
}
