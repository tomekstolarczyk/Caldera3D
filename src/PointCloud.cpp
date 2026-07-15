#include "PointCloud.hpp"
#include <fstream>   
#include <iostream> 
#include <string>

bool PointCloud::loadFromPLY(const std::string& filename) {

    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "error :( " << filename << std::endl;
        return false;
    }

    std::string line;
    bool header_ended = false;

    while (std::getline(file, line)) {
        if (line == "end_header") {
            header_ended = true;
            break;
        }
    }

    if (!header_ended) {
        std::cerr << "err" << std::endl;
        return false;
    }

    float x, y, z;
    while (file >> x >> y >> z) {
        points.push_back({x, y, z});
    }

    file.close();
    return true;
}