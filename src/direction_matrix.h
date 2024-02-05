#pragma once

namespace Direction
{
    extern int N, E, S, W;
    bool isRightTurn(int oldDir, int newDir);
    bool isLeftTurn(int oldDir, int newDir);
    bool isStraight(int oldDir, int newDir);
    bool is180(int oldDir, int newDir);

    int nextNode(int currNode, int dir);
    int nextDir(int currNode, int targetNode);
    bool isNodeBeforeTarget(int node, int dir, int targetNode);

    int nextNextNode(int currNode, int dir, int targetNode);
    int nextNextDir(int currNode, int dir, int targetNode);
    bool isNodeBeforeNodeBeforeTarget(int node, int dir, int targetNode);

    extern int dir_matrix[20][20];
    extern int nav_matrix[20][4];
}