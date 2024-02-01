#pragma once

namespace Direction
{
    extern int N, E, S, W;
    bool isRightTurn(int oldDir, int newDir);
    bool isLeftTurn(int oldDir, int newDir);
    bool isStraight(int oldDir, int newDir);
    bool is180(int oldDir, int newDir);

    extern int dir_matrix[20][20];
    extern int nav_matrix[20][4];
}