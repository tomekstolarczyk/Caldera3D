#include "TargetMatching.hpp"
#include <algorithm>
#include <cmath>

int matchTargetBox(const std::vector<OBB> &boxes, float targetDim1, float targetDim2,
                   float targetDim3, float tolerance)
{

    std::vector<float> targetBox = {targetDim1, targetDim2, targetDim3};
    std::sort(targetBox.begin(), targetBox.end());

    for (int i = 0; i < boxes.size(); i++)
    {

        if (std::abs(targetBox[0] - boxes[i].height) <= tolerance &&
            std::abs(targetBox[1] - boxes[i].width) <= tolerance &&
            std::abs(targetBox[2] - boxes[i].length) <= tolerance)
        {
            return i;
        }
    }

    return -1;
}