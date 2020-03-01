//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_VO_HOMO_ODOMETRY_H
#define MYSLAM_VO_HOMO_ODOMETRY_H

#include "base_odometry.h"

using namespace std;

namespace myslam {

class VoHomoOdometry : public BaseOdometry {
public:
    explicit VoHomoOdometry(const std::string& dataset_path);
    void runOdometry() override;

protected:
    void imgCallback(const ImgData& img_data) override;
    Pose getCameraPose() override;

private:
    void drawAR(cv::Mat& img);

    const char* TAG = "HomoOdometry";
    bool imshow_pause_ = true;
    M3d cur_H_ = M3d::Identity();
    cv::Point3i click_pt_;
    V3d anchor_pixel_ = V3d::Zero();
    V3d anchor_pixel_1 = V3d::Zero();
    V3d anchor_pixel_2 = V3d::Zero();

    void DrawCube(cv::Mat& image,
                  const Eigen::Matrix3d& wRc,
                  const Eigen::Vector3d& wTc,
                  const Eigen::Matrix3d& wRm,
                  const Eigen::Vector3d& wTm,
                  const Eigen::Matrix3d& K,
                  float size) {
        Eigen::Matrix<double, 3, 9> points;
        points << 0, 0, size, size, 0, 0, size, size, size / 2, 0, size, size, 0, 0, size, size, 0,
                size / 2, 0, 0, 0, 0, size, size, size, size, 0;
        points.colwise() -= Eigen::Vector3d(size / 2, size / 2, 0);
        Eigen::Matrix3d cRw = wRc.transpose();
        points = K * cRw * ((wRm * points).colwise() + (wTm - wTc));
        points.array().rowwise() /= points.row(2).array();

        vector<vector<cv::Point>> all_points(2);
        for (int i = 0; i < 8; ++i) {
            all_points[i / 4].emplace_back(points(0, i), points(1, i));
        }

        cv::Rect img_rect(0, 0, image.cols, image.rows);
        cv::Point2f pos(points(0, 8), points(1, 8));
        if (img_rect.contains(pos)) {
            cv::drawContours(image, all_points, 0, cv::Scalar(0, 255, 0), -3);
            for (int i = 0; i < 4; ++i) {
                cv::line(image, all_points[0][i], all_points[1][i], cv::Scalar(255, 0, 0), 3);
            }
            cv::drawContours(image, all_points, 1, cv::Scalar(0, 0, 255), 3);
        } else {
            cv::Point2f origin((image.cols - 1) * 0.5f, (image.rows - 1) * 0.5f);
            cv::Point2f direction = pos - origin;
            direction *= (100.0 / cv::norm(direction));
            cv::arrowedLine(image, origin, origin + direction, cv::Scalar(0, 255, 0), 2);
        }
    }
};

}

#endif //MYSLAM_VO_HOMO_ODOMETRY_H
