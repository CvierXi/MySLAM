//
// Created by sunxi on 2/26/20.
//

#include <algorithm>
#include <numeric>

#include "front_tracker.h"
#include "core/util/logging.h"
#include "core/util/util.h"

using namespace std;
using namespace myslam;

#define FRONT_FEATURE_TH_MIN 10
#define FRONT_FEATURE_TH_MAX 30
#define FRONT_PYR_LEVEL_MIN 0
#define FRONT_PYR_LEVEL_MAX 3
#define FRONT_PYR_WIN_SIZE_DEFAULT 21

FrontTracker::FrontTracker(FrontConfig &config) : config_(config) {
    if (config_.feature_threshold < FRONT_FEATURE_TH_MIN || config_.feature_threshold > FRONT_FEATURE_TH_MAX) {
        LOGE(TAG, "Input parameter - 'feature_threshold' must in [%d, %d]; now use default value: %d", FRONT_FEATURE_TH_MIN, FRONT_FEATURE_TH_MAX, FRONT_FEATURE_TH_MIN);
        config_.feature_threshold = FRONT_FEATURE_TH_MIN;
    }
    fast_detector_ = cv::FastFeatureDetector::create(config_.feature_threshold);
    if (config_.pyr_max_level < FRONT_PYR_LEVEL_MIN || config_.pyr_max_level > FRONT_PYR_LEVEL_MAX) {
        LOGE(TAG, "Input parameter - 'pyr_max_level' must in [%d, %d]; now use default value: %d", FRONT_PYR_LEVEL_MIN, FRONT_PYR_LEVEL_MAX, FRONT_PYR_LEVEL_MAX);
        config_.pyr_max_level = FRONT_PYR_LEVEL_MAX;
    }
    if (config_.pyr_win_size % 2 == 0) {
        LOGE(TAG, "Input parameter - 'pyr_win_size' must be odd; now use default value: %d", FRONT_PYR_WIN_SIZE_DEFAULT);
        config_.pyr_win_size = FRONT_PYR_WIN_SIZE_DEFAULT;
    }
    pyr_win_size_ = cv::Size(config_.pyr_win_size, config_.pyr_win_size);
}

void FrontTracker::imgCallback(const cv::Mat &img) {
    prev_pyr_ = next_pyr_;
    prev_pts_ = next_pts_;
    next_pyr_.clear();
    cv::buildOpticalFlowPyramid(img, next_pyr_, pyr_win_size_, config_.pyr_max_level);
    if (!prev_pyr_.empty()) {
        trackFeatures();
//        LOGD(TAG, "trackFeatures result: %d -> %d", prev_pts_.size(), track_src_pts_.size());
        drawOpticalFlow();
    }
    addFeatures();
}

void FrontTracker::getTrackPoints(vector<cv::Point2f>& track_src_pts, vector<cv::Point2f>& track_dst_pts) {
//    track_src_pts.assign(track_src_pts_.begin(), track_src_pts_.end());
//    track_dst_pts.assign(track_dst_pts_.begin(), track_dst_pts_.end());
    track_src_pts = track_src_pts_;
    track_dst_pts = track_dst_pts_;
}

void FrontTracker::getTrackPointsNormalized(vector<cv::Point2f>& track_src_pts, vector<cv::Point2f>& track_dst_pts) {
    int num_pts = track_src_pts_.size();
    if (num_pts == 0) {
        return;
    }
    int img_w = prev_pyr_[0].cols;
    int img_h = prev_pyr_[0].rows;
    for (int i = 0; i < num_pts; i++) {
        cv::Point2f track_src_pt(2.0f * track_src_pts_[i].x / img_w - 1, 2.0f * track_src_pts_[i].y / img_h - 1);
        cv::Point2f track_dst_pt(2.0f * track_dst_pts_[i].x / img_w - 1, 2.0f * track_dst_pts_[i].y / img_h - 1);
        track_src_pts.push_back(track_src_pt);
        track_dst_pts.push_back(track_dst_pt);
    }
}


void FrontTracker::trackFeatures() {
    int n = prev_pts_.size();
    track_src_pts_.clear();
    track_dst_pts_.clear();
    vector<cv::Point2f> klt_next_pts, klt_back_pts;
    vector<uchar> klt_next_status, klt_back_status;
    cv::calcOpticalFlowPyrLK(prev_pyr_, next_pyr_, prev_pts_, klt_next_pts, klt_next_status, cv::noArray(), pyr_win_size_, config_.pyr_max_level);
    cv::calcOpticalFlowPyrLK(next_pyr_, prev_pyr_, klt_next_pts, klt_back_pts, klt_back_status, cv::noArray(), pyr_win_size_, config_.pyr_max_level);
    for (int i = 0; i < n; i++) {
        if (klt_next_status[i] && klt_back_status[i]) {
            double back_error = (klt_back_pts[i] - prev_pts_[i]).dot((klt_back_pts[i] - prev_pts_[i]));
            if (back_error < 0.25) {
                track_src_pts_.push_back(prev_pts_[i]);
                track_dst_pts_.push_back(klt_next_pts[i]);
            }
        }
    }
}

void FrontTracker::addFeatures() {
    next_pts_.clear();
//    next_pts_.assign(track_dst_pts_.begin(), track_dst_pts_.end());
    int num_pts = next_pts_.size();
    vector<cv::KeyPoint> kps;
    fast_detector_->detect(next_pyr_[0], kps);

    vector<int> kp_responses;
    for (const auto& kp : kps) {
        kp_responses.push_back(kp.response);
    }
    vector<int> idx(kp_responses.size());
    iota(std::begin(idx), std::end(idx), 0);
    cv::sortIdx(kp_responses, idx, CV_SORT_DESCENDING);

    int num_new_kps = kps.size();
    int num_new_features = min(config_.feature_max_num - num_pts, num_new_kps);
//    LOGD(TAG, "addFeatures request: %d", num_new_features);
    for (int i = 0; i < num_new_features; i++) {
        next_pts_.emplace_back(kps[idx[i]].pt.x, kps[idx[i]].pt.y);
    }
//    LOGD(TAG, "addFeatures result: %d -> %d", num_pts, next_pts_.size());
}

void FrontTracker::drawOpticalFlow() {
    if (!config_.enable_show_optical_flow) {
        return;
    }

    cv::Mat dis_track, dis_opt_flow;
//    dis_opt_flow = next_pyr_[0].clone();
    cv::hconcat(prev_pyr_[0], next_pyr_[0], dis_track);
    cvtColorGray2Color(dis_track);
//    cvtColorGray2Color(dis_opt_flow);
    int n = track_src_pts_.size();
    for (int i = 0; i < n; i++) {
        cv::Point2f ref_pt(track_src_pts_[i].x, track_src_pts_[i].y);
        cv::Point2f src_pt(track_dst_pts_[i].x, track_dst_pts_[i].y);
        cv::Point2f dis_pt(src_pt.x + prev_pyr_[0].cols, src_pt.y);
        cv::circle(dis_track, ref_pt, 3, cv::Scalar(255, 0, 0));
        cv::circle(dis_track, dis_pt, 3, cv::Scalar(255, 0, 0));
        cv::line(dis_track, ref_pt, dis_pt, cv::Scalar(0, 255, 0));
    }
    cv::imshow("dis_track", dis_track);
}
