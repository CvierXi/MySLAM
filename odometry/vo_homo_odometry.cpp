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
}

void HomoOdometry::runOdometry() {
    LOGI(TAG, "runOdometry ...");
    if (dataset_parser_ == nullptr) {
        return;
    }
    int img_num = dataset_parser_->getImgNum();
    for (int i=0;i<img_num;i++) {
        ImgData img_data = dataset_parser_->getImgDataAt(i);
        if (!img_data.is_valid) {
            continue;
        }
        cv::imshow("img", img_data.img);
        cv::waitKey(50);
        imgCallback(img_data);
    }
    LOGI(TAG, "runOdometry finished.");
}

void HomoOdometry::imgCallback(const ImgData &img_data) {
    BaseOdometry::imgCallback(img_data);

}
