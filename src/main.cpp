#include <iostream>
#include "PointCloud.h"

int main() 
{
    std::cout << "===================" << std::endl;
    std::cout << "     Caldera3D     " << std::endl;
    std::cout << "===================" << std::endl;

    PointCloud cloud;
    
    if (cloud.loadData("data/table_scene_lms400.pcd"))
    {
        std::cout << "[SUCCESS] Dane w pamieci" << std::endl;
        std::cout << "[INFO] Liczba punktow : " << cloud.size() << std::endl;
    }
    else
    {
        std::cout << "[FATAL] :(" << std::endl;
    }
    
    return 0;
}