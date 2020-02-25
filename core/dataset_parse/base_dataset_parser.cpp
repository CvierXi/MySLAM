//
// Created by sunxi on 2/25/20.
//

#include <fstream>
#include "base_dataset_parser.h"
#include "core/util/logging.h"
#include "core/util/util.h"

using namespace std;
using namespace myslam;

ImgData::ImgData(double _timestamp, cv::Mat& _img) {
    timestamp = _timestamp;
    img = _img;
    is_valid = !_img.empty();
}

BaseDatasetParser::BaseDatasetParser(const string& dataset_path) : dataset_path_(dataset_path) {
}

bool BaseDatasetParser::parseImgData() {
    string img_list_path = dataset_path_ + "/list.txt";
    ifstream in(img_list_path);
    if (!in.is_open()) {
        LOGE(TAG, "Invalid img_list_path");
        return false;
    }
    string img_name;
    while (getline(in, img_name)) {
        img_names_.push_back(img_name);
    }
    return true;
}

bool BaseDatasetParser::parseImuData() {
    return false;
}

int BaseDatasetParser::getImgNum() {
    return img_names_.size();
}

ImgData BaseDatasetParser::getImgDataAt(int img_index) {
    string img_name = img_names_[img_index];
    string img_path;
    img_path.append(dataset_path_).append("/cam/").append(img_name);
    cv::Mat img = cv::imread(img_path);
    double timestamp = imgName2Timestamp(img_name);
    return ImgData(timestamp, img);
}
