//
// Created by sunxi on 2/25/20.
//

#include "vo_homo_odometry.h"
#include "core/config_parse/config_parser.h"
#include "core/dataset_parse/vo_dataset_parser.h"
#include "core/util/logging.h"
#include "core/util/util.h"

using namespace std;
using namespace myslam;

HomoOdometry::HomoOdometry(const string& config_file_path) : BaseOdometry(config_file_path) {
    if (!ConfigParser::setParameterFile(config_file_path)) {
        LOGE(TAG, "Invalid config_file_path");
        return;
    }
    string dataset_path = ConfigParser::get<string>("dataset_path");
    dataset_parser_ = make_unique<VoDatasetParser>(dataset_path);
    if (!dataset_parser_->parseData()) {
        dataset_parser_ = nullptr;
        return;
    }

    auto fx = ConfigParser::get<float>("camera.fx");
    auto fy = ConfigParser::get<float>("camera.fy");
    auto cx = ConfigParser::get<float>("camera.cx");
    auto cy = ConfigParser::get<float>("camera.cy");
    camera_ = make_unique<Camera>(fx, fy, cx, cy);

    FrontTracker::FrontConfig config{};
    config.feature_threshold = ConfigParser::get<int>("feature_threshold");
    config.feature_max_num = ConfigParser::get<int>("feature_max_num");
    config.pyr_max_level = ConfigParser::get<int>("pyr_max_level");
    config.pyr_win_size = ConfigParser::get<int>("pyr_win_size");
    config.enable_show_optical_flow = ConfigParser::get<int>("enable_show_optical_flow");
    front_tracker_ = make_unique<FrontTracker>(config);

#ifdef HAVE_VIZ
    viser_ = make_unique<Visualizer>();
    viser_->addCamera(camera_->K());
#endif
}

void HomoOdometry::runOdometry() {
    LOGI(TAG, "runOdometry ...");
    if (dataset_parser_ == nullptr) {
        return;
    }
    int img_num = dataset_parser_->getImgNum();
    for (int i = 0; i < img_num; i++) {
        ImgData img_data = dataset_parser_->getImgDataAt(i);
        if (!img_data.is_valid) {
            continue;
        }
//        cv::imshow("img_raw", img_data.img);
        imgCallback(img_data);
    }
    viser_->hold();
    LOGI(TAG, "runOdometry finished.");
}

void HomoOdometry::imgCallback(const ImgData &img_data) {
    BaseOdometry::imgCallback(img_data);
    cv::imshow("img_", img_);
    front_tracker_->imgCallback(img_);
    Pose pose = getCameraPose();
#ifdef HAVE_VIZ
    cv::Point3d cam_pos(pose.p(0), pose.p(1), pose.p(2));
    viser_->updateCameraPose(pose.q.toRotationMatrix(), pose.p);
#endif

    int ch = imshow_pause_ ? cv::waitKey() : cv::waitKey(50);
    if (ch == 'p' || ch == 'P') {
        imshow_pause_ = !imshow_pause_;
    }
}

Pose HomoOdometry::getCameraPose() {
    vector<cv::Point2f> track_src_pts, track_dst_pts;
    front_tracker_->getTrackPoints(track_src_pts, track_dst_pts);
    if (track_src_pts.empty()) {
        return BaseOdometry::getCameraPose();
    }

    /// p2 = H * p1
    cv::Mat H_cv = cv::findHomography(track_src_pts, track_dst_pts, cv::RANSAC, 2);
    M3d H, R;
    V3d t;
    cv::cv2eigen(H_cv, H);
    cur_H_ = H * cur_H_;
    recoverPoseFromHomography(cur_H_, R, t);
    return {t / 100, Q4d(R)};
}

