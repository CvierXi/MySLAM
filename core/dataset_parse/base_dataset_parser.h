//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_BASE_DATASET_PARSER_H
#define MYSLAM_BASE_DATASET_PARSER_H

#include "core/common.h"

namespace myslam {

struct ImgData {
    double timestamp;
    cv::Mat img;
    bool is_valid = false;

    ImgData(double _timestamp, cv::Mat& _img);
};

struct ImuData {
    double timestamp;
};

class BaseDatasetParser {
public:
    explicit BaseDatasetParser(const std::string& dataset_path);
    virtual ~BaseDatasetParser() = default;
    virtual bool parseData() = 0;

    int getImgNum();
    ImgData getImgDataAt(int img_index);

protected:
    virtual bool parseImgData();
    virtual bool parseImuData();
    const std::string dataset_path_;

private:
    const char* TAG = "BaseDatasetParser";
    std::vector<std::string> img_names_;
};

}

#endif //MYSLAM_BASE_DATASET_PARSER_H
