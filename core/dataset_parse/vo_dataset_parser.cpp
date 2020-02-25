//
// Created by sunxi on 2/25/20.
//

#include "vo_dataset_parser.h"
#include "core/util/logging.h"

using namespace std;
using namespace myslam;

VoDatasetParser::VoDatasetParser(const string& dataset_path) : BaseDatasetParser(dataset_path) {

}

void VoDatasetParser::parseData() {
    LOGI(TAG, "parseData ...");
}
