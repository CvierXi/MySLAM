//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_VO_HOMO_ODOMETRY_H
#define MYSLAM_VO_HOMO_ODOMETRY_H

#include "base_odometry.h"

using namespace std;

namespace myslam {

class HomoOdometry : public BaseOdometry {
public:
    explicit HomoOdometry(const std::string& dataset_path);
    void runOdometry() override;

protected:
    void imgCallback(const ImgData& img_data);

private:
    const char* TAG = "HomoOdometry";
};

}

#endif //MYSLAM_VO_HOMO_ODOMETRY_H
