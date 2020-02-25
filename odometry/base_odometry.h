//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_BASE_ODOMETRY_H
#define MYSLAM_BASE_ODOMETRY_H

#include "core/common.h"
#include "core/dataset_parse/base_dataset_parser.h"

namespace myslam {

class BaseOdometry {
public:
    explicit BaseOdometry(std::string config_file_path) {}
    virtual ~BaseOdometry() = default;
    virtual void runOdometry() = 0;

protected:
    std::unique_ptr<BaseDatasetParser> dataset_parser_;
};

}

#endif //MYSLAM_BASE_ODOMETRY_H
