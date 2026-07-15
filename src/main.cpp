#include <iostream>
#include "PointCloud.hpp"

int main() {
    std::cout << "Caldera3D starting..." << std::endl;
    PointCloud cloud;
    
    if (cloud.loadFromPLY("data/test_cloud.ply")) {
        std::cout << "yay :) read " << cloud.points.size() << " points " << std::endl;
        
        for (const auto& p : cloud.points) {
            std::cout << "Point >> X: " << p.x << ", Y: " << p.y << ", Z: " << p.z << std::endl;
        }
    }

    return 0;
}