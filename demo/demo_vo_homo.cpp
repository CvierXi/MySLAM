#include <iostream>

#include "core/config_parse/config_parser.h"
#include "odometry/vo_homo_odometry.h"

using namespace std;
using namespace myslam;

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;

    if (argc < 2) {
        cerr << "Input parameter file path!" << endl;
        return -1;
    }

    string config_file_path = argv[1];

    unique_ptr<BaseOdometry> odo = make_unique<HomoOdometry>(config_file_path);
    odo->runOdometry();

    cout << "Done!" << endl;

    return 0;
}
