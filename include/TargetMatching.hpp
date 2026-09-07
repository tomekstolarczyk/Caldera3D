#pragma once

#include "PCA.hpp"

// Zwraca indeks pasujacego pudelka z wektora boxes lub -1 w przypadku braku dopasowania
int matchTargetBox(const std::vector<OBB> &boxes, float targetDim1, float targetDim2,
                   float targetDim3, float tolerance);
