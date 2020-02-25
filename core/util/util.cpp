//
// Created by sunxi on 2/24/20.
//

#include "util.h"

using namespace std;

namespace myslam {

double imgName2Timestamp(std::string img_name) {
    img_name.resize(img_name.find('.'));
    double timestamp = atof(img_name.c_str()) / 1e9;
    return timestamp;
}

}

