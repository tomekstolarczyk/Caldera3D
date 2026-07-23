#include "PointCloud.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

bool PointCloud::loadData(const std::string& filepath)
{
    // open file
    std::ifstream file(filepath);

    std::string line;
    int vertexCount = 0;
    bool isPCD = false;

    if (!file.is_open())
    {
        std::cerr << "Unable to open file :(" << std::endl;
        return false;
    }

    // obsluga headera
    while(std::getline(file,line))
    {
        if(line.find("element vertex") != std::string::npos)
        {
            std::istringstream iss(line);
            std::string temp1, temp2;
            iss >> temp1 >> temp2 >> vertexCount;
        }
        else if (line.find("POINTS") != std::string::npos) // Składnia PCD
        {
            std::istringstream iss(line);
            std::string temp1;
            iss >> temp1 >> vertexCount;
            isPCD = true;
        }

        if (line == "end_header" || line.find("DATA ascii") != std::string::npos) {break;}
    }

    // reserwujemy in advance zeby uniknac reallocow caly czas
    points.clear();
    points.reserve(vertexCount);

    // wczytujemy wlasciwe punkty
    float x,y,z;
    for(int i = 0; i<vertexCount; i++)
    {
        file >> x >> y >> z; // ">>" ignoruje spacje
        addPoint(x, y, z);
        std::getline(file,line); // przechodzimy do nowej linii
    }

    file.close();

    std::cout << "[INFO] Format: " << (isPCD ? "PCD" : "PLY") 
    << " | Wczytano punktow: " << points.size() << " / " << vertexCount << std::endl;

    return points.size() > 0;

}