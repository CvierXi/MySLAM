//
// Created by sunxi on 2/26/20.
//

#include <numeric>

#include "front_tracker.h"
#include "core/util/logging.h"
#include "core/util/util.h"

using namespace std;
using namespace myslam;

FrontTracker::FrontTracker(FrontConfig &config) : config_(config) {
    fast_detector_ = cv::FastFeatureDetector::create(config_.feature_threshold);
    int win_size = (config_.pyr_win_size % 2 != 0) ? config_.pyr_win_size : 21;
    pyr_win_size_ = cv::Size(win_size, win_size);
}

void FrontTracker::imgCallback(const cv::Mat &img) {
    prev_pyr_ = next_pyr_;
    prev_pts_ = next_pts_;
    next_pyr_.clear();
    cv::buildOpticalFlowPyramid(img, next_pyr_, pyr_win_size_, config_.pyr_max_level);
    if (!prev_pyr_.empty()) {
        trackFeatures();
        LOGD(TAG, "trackFeatures result: %d -> %d", prev_pts_.size(), track_src_pts_.size());
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


void FrontTracker::trackFeatures() {
    int n = prev_pts_.size();
    track_src_pts_.clear();
    track_dst_pts_.clear();
    vector<cv::Point2f> klt_next_kps;
    vector<uchar> klt_status;
    cv::calcOpticalFlowPyrLK(prev_pyr_, next_pyr_, prev_pts_, klt_next_kps, klt_status, cv::noArray(), pyr_win_size_, config_.pyr_max_level);
    for (int i = 0; i < n; i++) {
        if (klt_status[i]) {
            track_src_pts_.push_back(prev_pts_[i]);
            track_dst_pts_.push_back(klt_next_kps[i]);
        }
    }
}

void FrontTracker::addFeatures() {
    next_pts_.clear();
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

    for (int i = 0; i< config_.feature_max_num; i++) {
        next_pts_.emplace_back(kps[idx[i]].pt.x, kps[idx[i]].pt.y);
    }
    LOGD(TAG, "addFeatures result: %d -> %d", num_pts, next_pts_.size());
}

void FrontTracker::drawOpticalFlow() {
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
