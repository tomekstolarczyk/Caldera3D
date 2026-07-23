#include <iostream>
#include "PointCloud.h"

int main() 
{
    std::cout << "===================" << std::endl;
    std::cout << "     Caldera3D     " << std::endl;
    std::cout << "===================" << std::endl;

    PointCloud cloud;
    
    if (cloud.loadData("data/test_cloud.ply"))
    {
        std::cout << "[SUCCESS] Dane w pamieci! Letsgo" << std::endl;
        std::cout << "Liczba punktow : " << cloud.size() << std::endl;

        std::cout << "Pierwsze 5 punktow chmury:" << std::endl;
        const std::vector<Point3D>& pts = cloud.getPoints();
        for(int i = 0; i<5; i++)
        {
            std::cout << "Punkt [" << i << "]: X=" << pts[i].x << ", Y=" << pts[i].y << ", Z=" << pts[i].z << std::endl; 
        }

    }
    else
    {
        std::cout << "[FATAL] :(" << std::endl;
    }
    
    return 0;
}