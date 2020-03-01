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

VoHomoOdometry::VoHomoOdometry(const string& config_file_path) : BaseOdometry(config_file_path) {
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
    visualizer_->addCamera(camera_->K());
#endif
}

void VoHomoOdometry::runOdometry() {
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
    LOGI(TAG, "runOdometry finished.");
#ifdef HAVE_VIZ
    visualizer_->hold();
#endif
}

void VoHomoOdometry::imgCallback(const ImgData &img_data) {
    BaseOdometry::imgCallback(img_data);
//    cv::imshow("imgCallback", img_);
    front_tracker_->imgCallback(img_);
    Pose pose = getCameraPose();

#ifdef HAVE_VIZ
    visualizer_->updateCameraPose(pose.q.toRotationMatrix(), pose.p);
#endif

    Mat img_ar = img_data.img;
    drawAR(img_ar);
//    Q4d wqm;
//    wqm.setIdentity();
//    DrawCube(img_ar, pose.q.toRotationMatrix(), pose.p, wqm.toRotationMatrix(), V3d(0,0,-0.5), camera_->K(), 0.5);
//    cv::imshow("Ar", img_ar);
//    cv::waitKey();
}

Pose VoHomoOdometry::getCameraPose() {
    vector<cv::Point2f> track_src_pts, track_dst_pts;
    front_tracker_->getTrackPointsNormalized(track_src_pts, track_dst_pts);
    if (!track_src_pts.empty()) {
        /// p2 = H * p1
        /// But notice that, camera moving right, pts in image moving left.
        cv::Mat H_cv = cv::findHomography(track_src_pts, track_dst_pts, cv::RANSAC, 0.1);
        M3d H;
        cv::cv2eigen(H_cv, H);
        cur_H_ = H * cur_H_;
        if (anchor_pixel_(2) > 0.5) {
            anchor_pixel_ = H * anchor_pixel_;
            anchor_pixel_ /= anchor_pixel_(2);
            anchor_pixel_1 = H * anchor_pixel_1;
            anchor_pixel_1 /= anchor_pixel_1(2);
            anchor_pixel_2 = H * anchor_pixel_2;
            anchor_pixel_2 /= anchor_pixel_2(2);
        }
    }

    M3d R;
    V3d t;
    recoverPoseFromHomography(cur_H_, R, t);
    /// in image0 coordinate
    return {-t, Q4d(R).inverse()};
}


void onMouseClick(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN) {
        auto* data = (cv::Point3i*)userdata;
        data->x = x;
        data->y = y;
        data->z = 1;
    } else {
        return;
    }
}

void VoHomoOdometry::drawAR(cv::Mat &img) {
    string window_name = "AR";
    if (anchor_pixel_(2) > 0.5) {
        cv::Point2f pt((anchor_pixel_(0) + 1) * img.cols / 2, (anchor_pixel_(1) + 1) * img.rows / 2);
        circle(img, pt, 3, Scalar(0, 255, 0));
        cv::Point2f pt1((anchor_pixel_1(0) + 1) * img.cols / 2, (anchor_pixel_1(1) + 1) * img.rows / 2);
        cv::Point2f pt2((anchor_pixel_2(0) + 1) * img.cols / 2, (anchor_pixel_2(1) + 1) * img.rows / 2);
        cv::line(img, pt, pt1, Scalar(255, 0, 0), 5);
        cv::line(img, pt, pt2, Scalar(0, 0, 255), 5);
    }
    imshow(window_name, img);
    setMouseCallback(window_name, onMouseClick, (void*)& click_pt_);

    int ch = imshow_pause_ ? cv::waitKey() : cv::waitKey(30);
    if (ch == 'p' || ch == 'P') {
        imshow_pause_ = !imshow_pause_;
    }

    if (click_pt_.z == 1) {
        anchor_pixel_(0) = 2.0 * click_pt_.x / img.cols - 1;
        anchor_pixel_(1) = 2.0 * click_pt_.y / img.rows - 1;
        anchor_pixel_(2) = 1.0;
        anchor_pixel_1(0) = anchor_pixel_(0) + 0.3;
        anchor_pixel_1(1) = anchor_pixel_(1);
        anchor_pixel_1(2) = 1.0;
        anchor_pixel_2(0) = anchor_pixel_(0);
        anchor_pixel_2(1) = anchor_pixel_(1) + 0.4;
        anchor_pixel_2(2) = 1.0;
        click_pt_.z = 0;
    }
//    cout << "===" << anchor_pixel_.transpose() << endl;
}
