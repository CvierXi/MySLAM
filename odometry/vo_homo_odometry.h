//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_VO_HOMO_ODOMETRY_H
#define MYSLAM_VO_HOMO_ODOMETRY_H

#include "base_odometry.h"

using namespace std;

namespace myslam {

class VoHomoOdometry : public BaseOdometry {
public:
    explicit VoHomoOdometry(const std::string& dataset_path);
    void runOdometry() override;

protected:
    void imgCallback(const ImgData& img_data) override;
    Pose getCameraPose() override;

private:
    const char* TAG = "HomoOdometry";
    bool imshow_pause_ = true;
    M3d cur_H_ = M3d::Identity();
    
};

}

#endif //MYSLAM_VO_HOMO_ODOMETRY_H
