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
    visualizer_ = make_unique<Visualizer>();
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
        if (!is_inited_) {
            is_inited_ = true;
        }
    }
    LOGI(TAG, "runOdometry finished.");
#ifdef HAVE_VIZ
    visualizer_->hold();
#endif
}

void HomoOdometry::imgCallback(const ImgData &img_data) {
    BaseOdometry::imgCallback(img_data);
    cv::imshow("imgCallback", img_);
    front_tracker_->imgCallback(img_);
    Pose pose = getCameraPose();

#ifdef HAVE_VIZ
    if (!is_inited_) {
//        visualizer_->addCamera(camera_->K(), cv::Point3f(pose.p(0), pose.p(1), pose.p(2)));
        visualizer_->addCamera(camera_->K(), pose.p);
    } else {
        visualizer_->updateCameraPose(pose.q.toRotationMatrix(), pose.p);
    }
#endif

//    cv::Mat img_raw = img_data.img;
//    Q4d q_m;
//    q_m.setIdentity();
//    V3d p_m(0, 0, 0);
//    DrawCube(img_raw, pose.q.toRotationMatrix(), pose.p, q_m.toRotationMatrix(), p_m, camera_->K(), 1);
//    cv::imshow("ar", img_raw);

    int ch = imshow_pause_ ? cv::waitKey() : cv::waitKey(50);
    if (ch == 'p' || ch == 'P') {
        imshow_pause_ = !imshow_pause_;
    }
}

Pose HomoOdometry::getCameraPose() {
    vector<cv::Point2f> track_src_pts, track_dst_pts;
    front_tracker_->getTrackPointsNormalized(track_src_pts, track_dst_pts);
    if (is_inited_ && !track_src_pts.empty()) {
        /// p2 = H * p1
        /// But notice that, camera moving right, pts in image moving left.
        cv::Mat H_cv = cv::findHomography(track_src_pts, track_dst_pts, cv::RANSAC, 0.1);
        M3d H;
        cv::cv2eigen(H_cv, H);
        cur_H_ = H * cur_H_;
    }

    M3d R;
    V3d t;
    recoverPoseFromHomography(cur_H_, R, t);
    /// in image0 coordinate
    return {-t, Q4d(R).inverse()};
}

