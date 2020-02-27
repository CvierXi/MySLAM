//
// Created by sunxi on 2/26/20.
//

#ifndef MYSLAM_FRONT_TRACKER_H
#define MYSLAM_FRONT_TRACKER_H

#include "core/common.h"

namespace myslam {

class FrontTracker {
public:
    struct FrontConfig {
        int feature_threshold;
        int feature_max_num;
        int pyr_max_level;
        int pyr_win_size;
    };

    explicit FrontTracker(FrontConfig& config);
    void imgCallback(const cv::Mat& img);

private:
    void trackFeatures();
    void addFeatures();
    void drawOpticalFlow();

    const char* TAG = "FrontTracker";
    FrontConfig config_;
    cv::Ptr<cv::FastFeatureDetector> fast_detector_;
    cv::Size pyr_win_size_;
    std::vector<cv::Mat> prev_pyr_, next_pyr_;
    std::vector<cv::Point2f> prev_pts_, next_pts_;
    std::vector<cv::Point2f> track_ref_pts_, track_src_pts_;
};

}

#endif //MYSLAM_FRONT_TRACKER_H
