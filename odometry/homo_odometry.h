//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_HOMO_ODOMETRY_H
#define MYSLAM_HOMO_ODOMETRY_H

#include "base_odometry.h"
#include "core/dataset_parse/vo_dataset_parser.h"

using namespace std;

namespace myslam {

class HomoOdometry : public BaseOdometry {
public:
    explicit HomoOdometry(std::string dataset_path) : BaseOdometry(dataset_path) {
        dataset_parser_ = make_unique<VoDatasetParser>("hhh");
        dataset_parser_->parseData();
    }
    void runOdometry() override {
        cout << "runHomoOdometry" << endl;
    }
};

}

#endif //MYSLAM_HOMO_ODOMETRY_H
