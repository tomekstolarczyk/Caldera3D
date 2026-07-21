#include "PointCloud.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

bool PointCloud::loadFromPLY(const std::string& filepath)
{
    // open file
    std::ifstream file(filepath);

    std::string line;
    int vertexCount = 0;

    if(file.is_open())
    {

        // obsluga headera
        while(std::getline(file,line))
        {
            if(line.find("element vertex") != std::string::npos)
            {
                std::istringstream iss(line);
                std::string temp1, temp2;
                iss >> temp1 >> temp2 >> vertexCount;
            }
            if(line == "end_header") {break;}
        }

        // reserwujemy in advance zeby uniknac reallocow caly czas
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
    }
    else
    {
        std::cerr << "Unable to open file :(" << std::endl;
        return false;
    }

    return true;

}