#include "PointCloud.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

// ply file format
bool PointCloud::loadData(const std::string &filepath)
{
    // open file
    std::ifstream file(filepath);

    std::string line;
    int vertexCount = 0;

    if (!file.is_open())
    {
        std::cerr << "Unable to open file :(" << std::endl;
        return false;
    }

    // obsluga headera
    while (std::getline(file, line))
    {
        if (line.find("element vertex") != std::string::npos)
        {
            std::istringstream iss(line);
            std::string temp1, temp2;
            iss >> temp1 >> temp2 >> vertexCount;
        }

        if (line == "end_header")
        {
            break;
        }
    }

    // reserwujemy in advance zeby uniknac reallocow caly czas
    points.clear();
    points.reserve(vertexCount);

    // wczytujemy wlasciwe punkty
    float x, y, z;
    for (int i = 0; i < vertexCount; i++)
    {
        file >> x >> y >> z; // ">>" ignoruje spacje
        addPoint(x, y, z);
        std::getline(file, line); // przechodzimy do nowej linii
    }

    file.close();

    return points.size() > 0;
}

bool PointCloud::saveData(const std::string &filepath) const
{
    std::ofstream file(filepath);

    if (!file.is_open())
    {
        std::cerr << "Unable to open file for writing :(" << std::endl;
        return false;
    }

    // header
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    // zrzucamy punkty
    for (const auto &p : points)
    {
        file << p.x << " " << p.y << " " << p.z << "\n";
    }

    file.close();
    return true;
}