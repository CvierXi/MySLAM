//
// Created by sunxi on 2/22/20.
//

#ifndef MYSLAM_CONFIG_PARSER_H
#define MYSLAM_CONFIG_PARSER_H

#include <iostream>
#include <memory>

#include "opencv2/opencv.hpp"

namespace myslam {

class ConfigParser {
public:
    ~ConfigParser();

    static bool setParameterFile(const std::string& file_path);

    template <typename T>
    static T get(const std::string& key) {
        return T(ConfigParser::config_parser_->file_[key]);
    }

private:
    static std::shared_ptr<ConfigParser> config_parser_;
    cv::FileStorage file_;

    ConfigParser() {
    }
};

}

#endif //MYSLAM_CONFIG_PARSER_H
