#include <iostream>

#include "core/config_parse/config_parser.h"
#include "odometry/homo_odometry.h"
#include "core/dataset_parse/vo_dataset_parser.h"

using namespace std;
using namespace myslam;

class A {
public:
    explicit A(std::string dataset_path) {}
//    virtual ~A() = default;
    virtual void parseData() {};
};

class B : A {
public:
    explicit B(std::string dataset_path) : A(dataset_path) {}
    void parseData() override {
        std::cout << "parseData" << std::endl;
    }
};

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;

    if (argc < 2) {
        cerr << "Input parameter file path!" << endl;
        return -1;
    }

    string config_file_path = argv[1];
    if (!myslam::ConfigParser::setParameterFile(config_file_path)) {
        cerr << "parameter file '" << config_file_path << "' does not exist." << endl;
        return -2;
    }

    string dataset_path = myslam::ConfigParser::get<string>("dataset_path");
    std::cout << "dataset dir is: " << dataset_path << std::endl;

    unique_ptr<BaseOdometry> odo = make_unique<HomoOdometry>(config_file_path);
    odo->runOdometry();

    return 0;

    string img_list_path = dataset_path + "/list.txt";
    ifstream in(img_list_path);
    if (!in.is_open()) {
        cerr << "No image list file!" << endl;
        return -3;
    }
    string img_name;

    cout << "Done!" << endl;

    return 0;
}
