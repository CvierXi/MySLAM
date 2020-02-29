//
// Created by sunxi on 2/28/20.
//

#include "visualizer.h"

using namespace std;
using namespace myslam;

Visualizer::Visualizer() {
    /// Create a window
//    myWindow_ = viz::Viz3d("Coordinate Frame");
    myWindow_ = viz::Viz3d("World Frame");

    /// Add coordinate axes
    myWindow_.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    /// Start event loop.
    myWindow_.spinOnce();
}

void Visualizer::addCamera(const M3d& K, const V3d& p_r) {
    cpw_ = viz::WCameraPosition(0.5);
    cv::Matx33d cam_K;
    cv::eigen2cv(K, cam_K);
    cpw_frustum_ = viz::WCameraPosition(cam_K);
    Point3d init_camera_pos(p_r(0), p_r(1), p_r(2));
    last_camera_pos_ = init_camera_pos;
    have_camera_ = true;
}

void Visualizer::addCamera(const M3d& K, const Point3d& init_camera_pos) {
    cpw_ = viz::WCameraPosition(0.5);
    cv::Matx33d cam_K;
    cv::eigen2cv(K, cam_K);
    cpw_frustum_ = viz::WCameraPosition(cam_K);
    last_camera_pos_ = init_camera_pos;
    have_camera_ = true;
}

void Visualizer::updateCameraPose(const M3d& R_wc, const V3d& t_wc) {
    if (!have_camera_) {
        return;
    }

    line_id++;

    V3d cam_z_unit_w = R_wc * V3d(0, 0, 1) + t_wc;
    V3d cam_y_unit_w = R_wc * V3d(0, 1, 0);

    cv::Point3d cam_pos(t_wc(0), t_wc(1), t_wc(2));
    cv::Vec3f cam_focal_point(cam_z_unit_w[0], cam_z_unit_w[1], cam_z_unit_w[2]);
    cv::Vec3f cam_y_dir(cam_y_unit_w[0], cam_y_unit_w[1], cam_y_unit_w[2]);

    viz::WLine line(last_camera_pos_, cam_pos, cv::viz::Color::green());
    myWindow_.showWidget("LINE" + std::to_string(line_id), line);
    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

    myWindow_.showWidget("CPW", cpw_, cam_pose);
    myWindow_.showWidget("CPW_FRUSTUM", cpw_frustum_, cam_pose);
    myWindow_.spinOnce();
    
    last_camera_pos_ = cam_pos;
}

void Visualizer::hold() {
    myWindow_.spin();
}


Point3d Visualizer::getPointInWorld(const V3d& p_r) {
    return Point3d(0,0,0);
}
