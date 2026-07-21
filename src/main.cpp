#include "PointCloud.h"

int main() 
{
    PointCloud cloud;
    
    cloud.addPoint(1.0f, 2.0f, 3.0f);
    cloud.addPoint(4.0f, 5.0f, 6.0f);
    
    std::cout << "Caldera3D Engine started." << std::endl;
    std::cout << "Loaded points: " << cloud.size() << std::endl;
    
    return 0;
}