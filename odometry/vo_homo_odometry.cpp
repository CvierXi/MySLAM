//
// Created by sunxi on 2/25/20.
//

#include "vo_homo_odometry.h"
#include "core/config_parse/config_parser.h"
#include "core/dataset_parse/vo_dataset_parser.h"
#include "core/util/logging.h"

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
    }

    M3f K;
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
    front_tracker_ = make_unique<FrontTracker>(config);
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
        cv::imshow("img_raw", img_data.img);
        imgCallback(img_data);
    }
    LOGI(TAG, "runOdometry finished.");
}

void HomoOdometry::imgCallback(const ImgData &img_data) {
    BaseOdometry::imgCallback(img_data);
    cv::imshow("img_", img_);
    front_tracker_->imgCallback(img_);
    Pose pose = getCameraPose();

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

    cv::Mat K_cv;
    cv::eigen2cv(camera_->K(), K_cv);

//    cv::Mat H_cv = cv::findHomography(track_src_pts, track_dst_pts, cv::RANSAC, 2);
//    vector<cv::Mat> rotations, translations, normals;
//    cv::decomposeHomographyMat(H_cv, K_cv, rotations, translations, normals);
//    if (rotations.size() > 1) {
//        LOGE(TAG, "Failed to decomposeHomographyMat, #solutions = %d", rotations.size());
//        return BaseOdometry::getCameraPose();
//    }
//    cv::Mat R_cv = rotations[0];
//    cv::Mat t_cv = translations[0];

    cv::Mat R_cv, t_cv, mask_cv;
    cv::Mat E_cv = cv::findEssentialMat(track_src_pts, track_dst_pts, K_cv, cv::RANSAC, 0.999, 1.0, mask_cv);
    cv::recoverPose(E_cv, track_src_pts, track_dst_pts, K_cv, R_cv, t_cv, mask_cv);

    M3f R;
    V3f p;
    cv::cv2eigen(R_cv, R);
    cv::cv2eigen(t_cv, p);
    Q4f q(R);
    cout << p.transpose() << endl;
    return {p, q};
}

