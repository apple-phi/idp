#pragma once
// #include <ArxContainer.h>

namespace Direction
{
    extern int N, E, S, W;
    bool isRightTurn(int oldDir, int newDir);
    bool isLeftTurn(int oldDir, int newDir);

    extern int dir_matrix[20][20];
    extern int nav_matrix[20][4];
    // extern arx::map<arx::pair<int, int>, int> navigation_map;
}