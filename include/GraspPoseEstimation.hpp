#pragma once

#include "TargetMatching.hpp"
#include <Eigen/Dense>

// teraz jak juz mamy paczke znelziona to musimy wyliczyc ostateczny zestaw punktow i orientacji w
// przestrzeni ktore przekazujemy do dedykowanego manipulatora

// choosing a top-down grasping approach - w zadaniach bin-picking (wyciaganie z kosza/ze stolu)
// chwyt od boku to niemal gwarantowana kolizja ramienia robota z blatem lub sasiednimi paczkami

struct GraspPose
{
    // 1 grasp point - nie interesuje nas centroid (sam srodek paczki) tylko srodek jej gornej
    // scianki miejsce gdzie nastapi kontakt z narzedziem
    Eigen::Vector3f position;

    // 2 Macierz orientacji paczki (3x3):
    // - col(0) -> os X (kierunek wzdluz szerokosci) - docelowy kierunek szczek
    // - col(1) -> os Y (kierunek wzdluz dlugosci)
    // - col(2) -> os Z - wektor podejscia chwytaka, skierowany w gore/od stolu
    Eigen::Matrix3f orientation;

    // 3 wymiary xy obiektu - pod badanie stroke limits chwytaka - czy w ogole da rade zlapac
    float targetWidth;
    float targetLength;
};

// funkcja wyznaczajaca uniwersalna pozycje chwytu
GraspPose computeGraspPose(const OBB &box, float tableA, float tableB, float tableC, float tableD);