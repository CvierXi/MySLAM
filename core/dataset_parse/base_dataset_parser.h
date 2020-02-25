//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_BASE_DATASET_PARSER_H
#define MYSLAM_BASE_DATASET_PARSER_H

#include "core/common.h"

namespace myslam {

class BaseDatasetParser {
public:
    explicit BaseDatasetParser(const std::string& dataset_path);
    virtual ~BaseDatasetParser() = default;
    virtual void parseData() = 0;

protected:
    const std::string dataset_path_;

private:
    const char* TAG = "BaseDatasetParser";
};

}

#endif //MYSLAM_BASE_DATASET_PARSER_H
