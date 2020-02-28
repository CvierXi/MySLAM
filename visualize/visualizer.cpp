//
// Created by sunxi on 2/28/20.
//

#include "visualizer.h"

using namespace std;
using namespace myslam;

Visualizer::Visualizer() {
    /// Create a window
    myWindow_ = viz::Viz3d("Coordinate Frame");

    /// Add coordinate axes
    myWindow_.showWidget("Coordinate Widget", viz::WCoordinateSystem());

    /// Start event loop.
    myWindow_.spinOnce();

    last_camera_pos_ = Point3d(0,0,0);
}

void Visualizer::addCamera(const M3d& K) {
    cpw_ = viz::WCameraPosition(0.5);
    cv::Matx33d cam_K;
    cv::eigen2cv(K, cam_K);
    cpw_frustum_ = viz::WCameraPosition(cam_K);
    have_camera_ = true;
}

void Visualizer::updateCameraPose(const M3d& R_wc, const V3d& t_wc) {
    if (!have_camera_) {
        return;
    }

    line_id++;

    V3d cam_z_w = R_wc * V3d(0, 0, 1) + t_wc;
    V3d cam_y_w = R_wc * V3d(0, 1, 0);

    cv::Point3d cam_pos(t_wc(0), t_wc(1), t_wc(2));
    cv::Vec3f cam_focal_point(cam_z_w[0], cam_z_w[1], cam_z_w[2]);
    cv::Vec3f cam_y_dir(cam_y_w[0], cam_y_w[1], cam_y_w[2]);

    viz::WLine line(last_camera_pos_, cam_pos, cv::viz::Color::green());
    myWindow_.showWidget("LINE" + std::to_string(line_id), line);
    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//    viz::WCameraPosition cpw(0.5); // Coordinate axes
//    viz::WCameraPosition cpw_frustum(Vec2f(0.889484, 0.523599)); // Camera frustum
    myWindow_.showWidget("CPW", cpw_, cam_pose);
    myWindow_.showWidget("CPW_FRUSTUM", cpw_frustum_, cam_pose);
    myWindow_.spinOnce();
    last_camera_pos_ = cam_pos;
}

void Visualizer::hold() {
    myWindow_.spin();
}
