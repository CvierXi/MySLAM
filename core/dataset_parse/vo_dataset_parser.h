//
// Created by sunxi on 2/25/20.
//

#ifndef MYSLAM_VO_DATASET_PARSER_H
#define MYSLAM_VO_DATASET_PARSER_H

#include "base_dataset_parser.h"

namespace myslam {

class VoDatasetParser : public BaseDatasetParser {
public:
    explicit VoDatasetParser(const std::string& dataset_path);
    void parseData() override;

private:
    const char* TAG = "VoDatasetParser";
};

}

#endif //MYSLAM_VO_DATASET_PARSER_H
