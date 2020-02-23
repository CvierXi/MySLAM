#include <iostream>

#include "../config_parse/config_parser.h"

using namespace std;

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

    string dataset_dir = myslam::ConfigParser::get<string>("dataset_dir");
    std::cout << "dataset dir is: " << dataset_dir << std::endl;

    string img_list_path = dataset_dir + "/list.txt";
    ifstream in(img_list_path);
    if (!in.is_open()) {
        cerr << "No image list file!" << endl;
        return -3;
    }
    string img_name;
    while (getline(in, img_name)) {
        cout << "===== " << img_name << endl;
        string img_path;
        img_path.append(dataset_dir).append("/cam/").append(img_name);
        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            continue;
        }
        cv::imshow("img", img);
        cv::waitKey(100);
    }

    cout << "Done!" << endl;

    return 0;
}
