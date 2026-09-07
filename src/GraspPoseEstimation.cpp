#include "GraspPoseEstimation.hpp"

GraspPose computeGraspPose(const OBB &box, float tableA, float tableB, float tableC, float tableD)
{
    GraspPose pose;

    // 1 wyznaczamy wektor normalny stolu
    Eigen::Vector3f stol(tableA, tableB, tableC);
    stol.normalize(); // ensures that the dot product will work correctly

    // 2 wyciagamy wektory wlasne znalezione z PCA - osie dl/szer/wys oraz wymiary
    std::vector<Eigen::Vector3f> axes = {box.rotationAxes.col(0), box.rotationAxes.col(1),
                                         box.rotationAxes.col(2)};
    std::vector<float> extents = {box.height, box.width, box.length};

    // 3 sprawdzamy ktory z tych wektorow wlasnych jest najbardziej rownolegly do wektora normalnego
    // stolu - ten z nich stanie sie osia Z - kierunkiem natarcia chwytaka
    int idx = 0;
    float maxDot = -1.0f;
    for (int i = 0; i < axes.size(); i++)
    {
        float docik = std::abs(axes[i].dot(stol));
        if (docik > maxDot)
        {
            maxDot = docik;
            idx = i;
        }
    }
    Eigen::Vector3f approachZ = axes[idx];
    float heightZ = extents[idx];

    // ZABEZPIECZENIE - jesli Z patrzy w stol, odwracamy go do gory
    if (approachZ.dot(stol) < 0.0f)
    {
        approachZ = -approachZ;
    }

    // 4 ustalamy pozostale 2 osie - te lezace w poziomie
    std::vector<int> horizontalIndices;
    for (int i = 0; i < 3; i++)
    {
        if (i != idx)
        {
            horizontalIndices.push_back(i);
        }
    }
    int idxA = horizontalIndices[0];
    int idxB = horizontalIndices[1];

    int shorterIdx = (extents[idxA] <= extents[idxB]) ? idxA : idxB;
    int longerIdx = (extents[idxA] <= extents[idxB]) ? idxB : idxA;

    Eigen::Vector3f graspX = axes[shorterIdx];
    float widthX = extents[shorterIdx];
    Eigen::Vector3f graspY = axes[longerIdx];
    float lengthY = extents[longerIdx];

    // ZABEZPIECZENIE - Right-Hand Rule
    graspY = approachZ.cross(graspX).normalized();
    graspX = graspY.cross(approachZ).normalized();

    // 5 grasp point - srodek gornej scianki paczki
    float graspDepth = 0.02f;
    Eigen::Vector3f center(box.centroid.x, box.centroid.y, box.centroid.z);
    pose.position = center + approachZ * (heightZ * 0.5f - graspDepth);

    // 6 wypelniamy finalna ramke wynikowa
    pose.orientation.col(0) = graspX;    // Kierunek zacisku szczek
    pose.orientation.col(1) = graspY;    // dluzszy bok poziomy
    pose.orientation.col(2) = approachZ; // Kierunek odejscia od stolu (do gory)
    pose.targetWidth = widthX;
    pose.targetLength = lengthY;

    return pose;
}