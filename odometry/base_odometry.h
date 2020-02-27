//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_BASE_ODOMETRY_H
#define MYSLAM_BASE_ODOMETRY_H

#include "core/common.h"
#include "core/dataset_parse/base_dataset_parser.h"
#include "core/front_track/front_tracker.h"

namespace myslam {

class BaseOdometry {
public:
    explicit BaseOdometry(const std::string& config_file_path);
    virtual ~BaseOdometry() = default;
    virtual void runOdometry() = 0;

protected:
    virtual void imgCallback(const ImgData& img_data);
    virtual void imuCallback(const ImuData& imu_data);
    std::unique_ptr<BaseDatasetParser> dataset_parser_;
    std::unique_ptr<FrontTracker> front_tracker_;
    cv::Mat img_;

private:
    const char* TAG = "BaseOdometry";
};

}

#endif //MYSLAM_BASE_ODOMETRY_H
